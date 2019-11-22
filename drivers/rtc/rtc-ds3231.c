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

/**
 * struct ds3231 - private data for the ds3231 device
 * @rtc: Associated RTC device structure
 */
struct ds3231 {
	struct rtc_device *rtc;
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
 * ds3231_rtc_ops: RTC operations for DS3231
 */
static const struct rtc_class_ops ds3231_rtc_ops = {
	.read_time = ds3231_rtc_read_time,
	.set_time = ds3231_rtc_set_time,
};

/**
 * ds3231_probe() - Probe function for the ds3231 driver
 */
static int ds3231_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct ds3231 *priv;

	/* Allocate a private struct for the device */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}

	/* Register a rtc device */
	priv->rtc = rtc_device_register("ds3231", &client->dev, &ds3231_rtc_ops,
					THIS_MODULE);
	if (IS_ERR(priv->rtc)) {
		ret = PTR_ERR(priv->rtc);
		return ret;
	}

	/* Set client private data */
	i2c_set_clientdata(client, priv);

	return 0;
}

/**
 * ds3231_remove() - Remove function for the ds3231 driver
 */
static int ds3231_remove(struct i2c_client *client) {
	struct ds3231 *priv = i2c_get_clientdata(client);

	/* Unregister rtc device and free client private data */
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
