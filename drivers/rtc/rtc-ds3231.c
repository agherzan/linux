/*
 * I2C driver for Maxim/Dallas DS3231 Real-Time Clock
 *
 * Copyright (C) 2019 Andrei Gherzan <andrei@gherzan.ro>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bcd.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/rtc.h>
#include <linux/slab.h>

/* Time registers */
#define DS3231_REG_SEC		0x00
#define DS3231_REG_MIN		0x01
#define DS3231_REG_HOUR		0x02
#define DS3231_REG_DAY		0x03
#define DS3231_REG_DATE		0x04
#define DS3231_REG_MONTH	0x05
#define DS3231_REG_YEAR		0x06

/* Time registers range */
#define DS3231_REG_TIMESTART	DS3231_REG_SEC
#define DS3231_REG_TIMESIZE	DS3231_REG_YEAR + 1

/* Century */
#define DS3231_REG_CENTURY	DS3231_REG_MONTH
#define DS3231_MASK_CENTURY	0X80

/* Alarm 1 registers */
#define DS3231_A1_SEC_REG	0x07 /* seconds register */
#define DS3231_A1_SEC_MASK	0x7f /* seconds mask */
#define DS3231_A1_MIN_REG	0x08 /* minutes register */
#define DS3231_A1_MIN_MASK	0x7f /* minutes mask */
#define DS3231_A1_HOUR_REG	0x09 /* hour register */
#define DS3231_A1_HOUR_MASK	0x3f /* hour mask */
#define DS3231_A1_DATE_REG	0x0a /* date register */
#define DS3231_A1_DATE_MASK	0x3f /* date mask */
#define DS3231_A1_BASE_REG	DS3231_A1_SEC_REG
#define DS3231_A1_SIZE		DS3231_A1_DATE_REG - DS3231_A1_BASE_REG + 1

/* Alarm 1 register mask bits */
#define DS3231_A1M1_REG		DS3231_A1_REG_SEC
#define DS3231_A1M1_MASK	0x80
#define DS3231_A1M2_REG		DS3231_A1_REG_MIN
#define DS3231_A1M2_MASK	0x80
#define DS3231_A1M3_REG		DS3231_A1_REG_HOUR
#define DS3231_A1M3_MASK	0x80
#define DS3231_A1M4_REG		DS3231_A1_REG_DAY
#define DS3231_A1M4_MASK	0x80

/* Alarm 1 day/date register */
#define DS3231_DYDT_REG		DS3231_A1_REG_DAY
#define DS3231_DYDT_MASK	0x40

/* Temperature sensor */
#define DS3231_REG_TEMP		0x11
#define DS3231_TEMP_RESOLUTION	25

/* Control/status registers */
#define DS3231_CTL_REG		0x0e
#define DS3231_CTL_A1IE_MASK	0x01
#define DS3231_STATUS_REG	0x0f
#define DS3231_STATUS_A1F_MASK	0x01

/**
 * struct ds3231 - private data for the ds3231 device
 * @rtc: Associated RTC device structure
 * @hwmon: Associated temperature sensor device
 */
struct ds3231 {
	struct rtc_device *rtc;
	struct device *hwmon;
	struct device *dev;
};

static const struct i2c_device_id ds3231_id[] = {
	{ "ds3231", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds3231_id);

static const struct of_device_id ds3231_of_match[] = {
	{ .compatible = "maxim,ds3231" },
	{ }
};
MODULE_DEVICE_TABLE(of, ds3231_of_match);

/**
 * ds3231_read_regs() - Read registers from a ds3231 i2c client
 * @client: The i2c client used for data transfer
 * @reg: Register address to start reading from
 * @data: Return data buffer
 * @n: Size of the data buffer
 *
 * Data transfer happens in chunks of 8 bits. This function writes the target
 * register (read pointer) and afterwards switches in read mode and gets the
 * data starting from it. On each 8 bits chunk sent, the read pointer register
 * gets incremented automatically.
 */
static int ds3231_read_regs(struct i2c_client *client, uint8_t reg,
			uint8_t *data, size_t n) {
	int ret;
	struct i2c_msg msgs[] = {
		{
			/* Set read pointer */
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			/* Read registers */
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = data,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: IO error [ret=%d]\n", __func__, ret);
		return -EIO;
	}
	return 0;
}

/**
 * ds3231_write_regs() - Write registers to a ds3231 i2c client
 * @client: The i2c client used for data transfer
 * @data: Data to be written
 * @n: Size of the data buffer
 *
 * Data transfer happens in chunks of 8 bits. The first 8 bits represent the
 * destination register. This means that first entry in the data array needs to
 * contain this information. The next entries get written starting with the
 * that register. On each new chunk sent, the destination register gets
 * incremented automatically.
 */
static int ds3231_write_regs(struct i2c_client *client, int8_t *data, size_t n) {
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = n,
			.buf = data,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: IO error [ret=%d]\n", __func__, ret);
		return -EIO;
	}
	return 0;
}

/**
 * ds3231_rtc_read_time - Read time from RTC device
 * @device: The associated device node
 * @time: Returned rtc_time structure
 */
static int ds3231_rtc_read_time(struct device *device, struct rtc_time *time) {
	uint8_t buf[DS3231_REG_TIMESIZE];
	struct i2c_client *client = container_of(device, struct i2c_client, dev);
	int ret;
	bool century = false;

	ret = ds3231_read_regs(client, DS3231_REG_TIMESTART, buf, sizeof(buf));
	if (ret)
		return ret;

	if (bcd2bin(buf[DS3231_REG_CENTURY] & DS3231_MASK_CENTURY))
		century = true;

	dev_dbg(&client->dev, "%s: registers time: sec=%02x, min=%02x, "
		       "hour=%02x, day=%02x, date=%02x, month=%02x, year=%02x, "
		       "century=%d",
		       __func__,
		       buf[DS3231_REG_SEC],
		       buf[DS3231_REG_MIN],
		       buf[DS3231_REG_HOUR],
		       buf[DS3231_REG_DAY],
		       buf[DS3231_REG_DATE],
		       buf[DS3231_REG_MONTH],
		       buf[DS3231_REG_YEAR],
		       century);

	/* Set the final time structure */
	time->tm_sec = bcd2bin(buf[DS3231_REG_SEC] & 0x7f);
	time->tm_min = bcd2bin(buf[DS3231_REG_MIN] & 0x7f);
	time->tm_hour = bcd2bin(buf[DS3231_REG_HOUR] & 0x3f);
	/* Convert 1-7 to 0-6 */
	time->tm_wday = bcd2bin(buf[DS3231_REG_DAY] & 0x07) - 1;
	time->tm_mday = bcd2bin(buf[DS3231_REG_DATE] & 0x3f);
	/* Convert 1-12 to 0-11 */
	time->tm_mon = bcd2bin(buf[DS3231_REG_MONTH] & 0x1f) - 1;
	/* The rtc stores year as 0-99 plus a century bit */
	time->tm_year = bcd2bin(buf[DS3231_REG_YEAR]);
	if (century)
		time->tm_year += 100;

	dev_dbg(&client->dev, "%s: read time: sec=%d, min=%d, "
			"hour=%d, day=%d, date=%d, month=%d, year=%d",
			__func__,
			time->tm_sec, time->tm_min, time->tm_hour, time->tm_wday,
			time->tm_mday, time->tm_mon, time->tm_year);

	return 0;
}

/**
 * ds3231_rtc_set_time - Set time to RTC device
 * @device: The associated device node
 * @time: Time to set
 */
static int ds3231_rtc_set_time(struct device *device, struct rtc_time *time) {
	int ret, tmp;
	/* Destination register needs to be included as well */
	uint8_t buf[DS3231_REG_TIMESIZE + 1];
	struct i2c_client *client = container_of(device, struct i2c_client, dev);

	dev_dbg(&client->dev, "%s: set time: sec=%d, min=%d, "
			"hour=%d, day=%d, date=%d, month=%d, year=%d",
			__func__,
			time->tm_sec, time->tm_min, time->tm_hour, time->tm_wday,
			time->tm_mday, time->tm_mon, time->tm_year);

	/* Shift all indexes by 1 because the first index needs to be the
	 * destination register */
	buf[0] = DS3231_REG_TIMESTART;
	buf[DS3231_REG_SEC + 1] = bin2bcd(time->tm_sec);
	buf[DS3231_REG_MIN + 1] = bin2bcd(time->tm_min);
	buf[DS3231_REG_HOUR + 1] = bin2bcd(time->tm_hour);
	/* Convert from 0-6 to 1-7 */
	buf[DS3231_REG_DATE + 1] = bin2bcd(time->tm_wday + 1);
	buf[DS3231_REG_DATE + 1] = bin2bcd(time->tm_mday);
	/* Convert from 0-11 to 1-12 */
	buf[DS3231_REG_MONTH + 1] = bin2bcd(time->tm_mon + 1);
	/* The rtc stores year as 0-99 plus a century bit */
	tmp = time->tm_year;
	if (time->tm_year > 99) {
		tmp %= 100;
		buf[DS3231_REG_CENTURY + 1] |= DS3231_MASK_CENTURY;
	}
	buf[DS3231_REG_YEAR + 1] = bin2bcd(tmp);

	ret = ds3231_write_regs(client, buf, sizeof(buf));
	if (ret)
		return ret;

	return 0;
}


/**
 * ds3231_rtc_read_a1_cs - Read alarm 1, control and status registers from RTC
 * 			 - device
 * device @device: The associated device node
 * @a1_buf: Array buffer for alarm 1 registers
 * @cs_buf: Array buffer for control and status registers
 */
static int ds3231_read_a1_cs(struct device *device,
			     uint8_t a1_buf[], uint8_t cs_buf[]) {
	int ret;
	struct i2c_client *client = container_of(device, struct i2c_client, dev);

	/* Read all alarm 1 registers */
	ret = ds3231_read_regs(client, DS3231_A1_BASE_REG, a1_buf,
			       DS3231_A1_SIZE);
	if (ret) {
		dev_err(device, "%s: failed to read alarm 1 registers [%d]\n",
			__func__, ret);
		return ret;
	}
	dev_dbg(device, "%s: alarm 1 registers: sec=%02x, min=%02x, hour=%02x, date=%02x",
		__func__, a1_buf[0], a1_buf[1], a1_buf[2], a1_buf[3]);

	/* Read control and status registers - array of two registers */
	ret = ds3231_read_regs(client, DS3231_CTL_REG, cs_buf, 2);
	if (ret) {
		dev_err(device, "%s: failed to read control/status registers [%d]\n",
			__func__, ret);
		return ret;
	}
	dev_dbg(device, "%s: control/status registers: control=%02x, status=%02x",
		__func__, cs_buf[0], cs_buf[1]);

	return 0;
}

/**
 * ds3231_rtc_read_alarm - Read alarm from RTC device
 * @device: The associated device node
 * @alarm: Returned rtc_wkalrm structure
 */
static int ds3231_rtc_read_alarm(struct device *device, struct rtc_wkalrm *alarm) {
	int ret;
	uint8_t a1_buf[DS3231_A1_SIZE]; /* alarm 1 registers */
	uint8_t cs_buf[2];	/* control and status registers */

	/* Read alarm 1, control and status registers */
	ret = ds3231_read_a1_cs(device, a1_buf, cs_buf);
	if (ret)
		return ret;

	/* Construct alarm structure assuming:
	 *	- hour in 24 format
	 * 	- day/date register set to day of the month
	 */
	alarm->time.tm_sec = bcd2bin(a1_buf[0] & DS3231_A1_SEC_MASK);
	alarm->time.tm_min = bcd2bin(a1_buf[1] & DS3231_A1_MIN_MASK);
	alarm->time.tm_hour = bcd2bin(a1_buf[2] & DS3231_A1_HOUR_MASK);
	alarm->time.tm_mday = bcd2bin(a1_buf[3] & DS3231_A1_DATE_MASK);
	alarm->enabled = bcd2bin(cs_buf[0] & DS3231_CTL_A1IE_MASK);
	alarm->pending = bcd2bin(cs_buf[1] & DS3231_STATUS_A1F_MASK);
	dev_dbg(device, "%s: read alarm 1: sec=%d, min=%d, hour=%d, date=%d, enabled=%d, pending=%d",
		__func__, alarm->time.tm_sec, alarm->time.tm_min,
		alarm->time.tm_hour, alarm->time.tm_mday, alarm->enabled,
		alarm->pending);

	return 0;
}

/**
 * ds3231_rtc_set_alarm - Set alarm for RTC device
 * @device: The associated device node
 * @alarm: Alarm to set
 */
static int ds3231_rtc_set_alarm(struct device *device, struct rtc_wkalrm *alarm) {
	int ret;
	/* We use the same array for reading and writing the registers. When
	 * writing, we need the first value to be the destination address,
	 * that's why we allocate an additional element. */
	uint8_t a1_buf[DS3231_A1_SIZE + 1]; /* alarm 1 registers */
	uint8_t cs_buf[2 + 1];	/* control and status registers */
	struct i2c_client *client = container_of(device, struct i2c_client, dev);

	dev_dbg(device, "%s: set alarm 1: sec=%d, min=%d, hour=%d, date=%d, enabled=%d, pending=%d",
			__func__, alarm->time.tm_sec, alarm->time.tm_min,
			alarm->time.tm_hour, alarm->time.tm_mday,
			alarm->enabled, alarm->pending);

	/* Read alarm 1, control and status registers */
	ret = ds3231_read_a1_cs(device, &a1_buf[1], &cs_buf[1]);
	if (ret)
		return ret;

	/* Disable alarm to make sure no interrupts happen while setting a new
	 * alarm */
	cs_buf[0] = DS3231_CTL_REG;
	cs_buf[1] &= ~DS3231_CTL_A1IE_MASK;
	cs_buf[2] &= ~DS3231_STATUS_A1F_MASK;
	ret = ds3231_write_regs(client, cs_buf, sizeof(cs_buf));
	if (ret) {
		dev_err(device, "%s: Failed to disable alarm [%d]", __func__,
			ret);
		return ret;
	}

	/* Set the requested alarm */
	a1_buf[0] = DS3231_A1_BASE_REG;
	a1_buf[1] |= bin2bcd(alarm->time.tm_sec) & DS3231_A1_SEC_MASK;
	a1_buf[2] |= bin2bcd(alarm->time.tm_sec) & DS3231_A1_MIN_MASK;
	a1_buf[3] |= bin2bcd(alarm->time.tm_sec) & DS3231_A1_HOUR_MASK;
	a1_buf[4] |= bin2bcd(alarm->time.tm_sec) & DS3231_A1_DATE_MASK;
	ret = ds3231_write_regs(client, a1_buf, sizeof(a1_buf));
	if (ret) {
		dev_err(device, "%s: Failed to set the new alarm [%d]",
				__func__, ret);
		return ret;
	}

	/* When requested, enable the alarm - we only work with alarm 1 */
	if (alarm->enabled) {
		cs_buf[1] |= DS3231_CTL_A1IE_MASK;
		ret = ds3231_write_regs(client, cs_buf, sizeof(cs_buf));
		if (ret)
			dev_warn(device, "%s: Failed to enable alarm [%d]",
					__func__, ret);
	}

	return 0;
}

static int ds3231_alarm_irq_enable(struct device *dev, unsigned int enabled) {
	return 0;
}

/**
 * ds3231_rtc_ops: RTC operations for DS3231
 */
static const struct rtc_class_ops ds3231_rtc_ops = {
	.read_time = ds3231_rtc_read_time,
	.set_time = ds3231_rtc_set_time,
	.read_alarm = ds3231_rtc_read_alarm,
	.set_alarm = ds3231_rtc_set_alarm,
	.alarm_irq_enable = ds3231_alarm_irq_enable,
};

/**
 * ds3231_temp_show - Read the temperature sensor
 * @device: The associated device node
 * @attr: Device attribute
 * @buf: Temperature in millidegrees Celsius
 */
static ssize_t ds3231_temp_show(struct device *device,
				struct device_attribute *attr, char *buf)
{
	uint8_t temp_buf[2];
	int temp, ret;
	struct ds3231 *ds3231 = dev_get_drvdata(device);
	struct i2c_client *client = container_of(ds3231->dev, struct i2c_client, dev);

	ret = ds3231_read_regs(client, DS3231_REG_TEMP, temp_buf,
			sizeof(temp_buf));
	if (ret)
		return ret;

	/* The device represents the temperature as 10 bit code with a 0.25C
	 * resolution. The first 8 bits are from register 11h and represent the
	 * integer portion while the last 2 bits are from the upper nibble at
	 * location 12h and represent the factional portion.
	 */
	dev_dbg(&client->dev, "%s: raw temp registers 0x%02x=%02x, 0x%02x=x%02x", __func__, DS3231_REG_TEMP, temp_buf[0], DS3231_REG_TEMP+1, temp_buf[1]);
	temp = (((temp_buf[0] << 8) | temp_buf[1]) >> 6);
	temp *= DS3231_TEMP_RESOLUTION * 10;
	dev_dbg(&client->dev, "%s: temp: %dmC", __func__, temp);

	return sprintf(buf, "%d\n", temp);
}

/* Define the attribute group for registering the hwmon device - temp sensor */
SENSOR_DEVICE_ATTR(temp1, 0444, ds3231_temp_show, NULL, 0);
static struct attribute *ds3231_hwmon_attrs[] = {
	&sensor_dev_attr_temp1.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ds3231_hwmon);

/**
 * ds3231_probe() - Probe function for the ds3231 driver
 */
static int ds3231_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct ds3231 *priv;
	struct device *dev;

	/* Allocate a private struct for the device */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto fail;
	}

	priv->dev = &client->dev;

	/* Register a rtc device */
	priv->rtc = rtc_device_register("ds3231", &client->dev, &ds3231_rtc_ops,
					THIS_MODULE);
	if (IS_ERR(priv->rtc)) {
		ret = PTR_ERR(priv->rtc);
		goto fail_rtc;
	}

	/* Register a hwmon device for the temperature sensor */
	priv->hwmon = hwmon_device_register_with_groups(&client->dev, "ds3231", priv,
						ds3231_hwmon_groups);
	if (IS_ERR(priv->hwmon)) {
		ret = PTR_ERR(priv->hwmon);
		goto fail_hwmon;
	}

	/* Set client private data */
	i2c_set_clientdata(client, priv);

	return 0;

fail_hwmon:
	rtc_device_unregister(priv->rtc);
fail_rtc:
fail:
	return ret;
}

/**
 * ds3231_remove() - Remove function for the ds3231 driver
 */
static int ds3231_remove(struct i2c_client *client) {
	struct ds3231 *priv = i2c_get_clientdata(client);

	hwmon_device_unregister(priv->hwmon);
	rtc_device_unregister(priv->rtc);
	kfree(priv);

	return 0;
}

static struct i2c_driver ds3231_i2c_driver = {
	.driver = {
		.name = "ds3231",
		.of_match_table = ds3231_of_match,
	},
	.probe 		= ds3231_probe,
	.remove		= ds3231_remove,
	.id_table 	= ds3231_id,
};

static int __init ds3231_init(void) {
	return i2c_add_driver(&ds3231_i2c_driver);
}
module_init(ds3231_init);

static void __exit ds3231_exit(void) {
	i2c_del_driver(&ds3231_i2c_driver);
}
module_exit(ds3231_exit);

MODULE_AUTHOR("Andrei Gherzan <andrei@gherzan.ro>");
MODULE_DESCRIPTION("Maxim/Dallas DS3231 RTC Driver");
MODULE_LICENSE("GPL");
