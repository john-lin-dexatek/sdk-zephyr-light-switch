/* cm3218.c - driver for CM3218 light sensor */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stts75

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "stts75.h"

LOG_MODULE_REGISTER(STTS75, CONFIG_SENSOR_LOG_LEVEL);

static int stts75_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct stts75_driver_data *drv_data = dev->data;
	const struct stts75_config *config = dev->config;
	uint8_t buf[2];

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	if (i2c_burst_read_dt(&config->i2c, 0, buf, 2) < 0) {
		LOG_DBG("stts75 read NG \n");
		return -EIO;
	}
	// LOG_DBG("stts75 read reg 0 bytes: %02x %02x\n", buf[0], buf[1]);
	

	// if (i2c_reg_read_byte_dt(&config->i2c,
	// 			 ISL29035_DATA_MSB_REG, &msb) < 0) {
	// 	return -EIO;
	// }

	// if (i2c_reg_read_byte_dt(&config->i2c, ISL29035_DATA_LSB_REG,
	// 			 &lsb) < 0) {
	// 	return -EIO;
	// }

	drv_data->data_sample = (buf[0] << 8) + buf[1];

	return 0;
}

static int stts75_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct stts75_driver_data *drv_data = dev->data;
	uint64_t tmp;

	val->val1 = (drv_data->data_sample >> 8);
	val->val2 = (uint8_t)drv_data->data_sample;

	/* val = sample_val * lux_range / (2 ^ adc_data_bits) */
	// tmp = (uint64_t)drv_data->data_sample * ISL29035_LUX_RANGE;
	// val->val1 = tmp >> ISL29035_ADC_DATA_BITS;
	// tmp = (tmp & ISL29035_ADC_DATA_MASK) * 1000000U;
	// val->val2 = tmp >> ISL29035_ADC_DATA_BITS;

	return 0;
}

static const struct sensor_driver_api stts75_api = {
#if CONFIG_STTS75_TRIGGER
	.attr_set = &stts75_attr_set,
	.trigger_set = &stts75_trigger_set,
#endif
	.sample_fetch = &stts75_sample_fetch,
	.channel_get = &stts75_channel_get,
};

static int stts75_init(const struct device *dev)
{
	struct stts75_driver_data *drv_data = dev->data;
	const struct stts75_config *config = dev->config;
	uint8_t buf[2];

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	drv_data->data_sample = 0U;

	if (i2c_burst_read_dt(&config->i2c, 0, buf, 2) < 0) {
		LOG_DBG("stts75 read NG \n");
		return -EIO;
	}
	LOG_DBG("stts75 read reg 0 bytes: %02x %02x\n", buf[0], buf[1]);

	// /* clear blownout status bit */
	// if (i2c_reg_update_byte_dt(&config->i2c, ISL29035_ID_REG,
	// 			   ISL29035_BOUT_MASK, 0) < 0) {
	// 	LOG_DBG("Failed to clear blownout status bit.");
	// 	return -EIO;
	// }

	// /* set command registers to set default attributes */
	// if (i2c_reg_write_byte_dt(&config->i2c,
	// 			  ISL29035_COMMAND_I_REG, 0) < 0) {
	// 	LOG_DBG("Failed to clear COMMAND-I.");
	// 	return -EIO;
	// }

	// if (i2c_reg_write_byte_dt(&config->i2c,
	// 			  ISL29035_COMMAND_II_REG, 0) < 0) {
	// 	LOG_DBG("Failed to clear COMMAND-II.");
	// 	return -EIO;
	// }

	// /* set operation mode */
	// if (i2c_reg_update_byte_dt(&config->i2c,
	// 			   ISL29035_COMMAND_I_REG,
	// 			   ISL29035_OPMODE_MASK,
	// 			   ISL29035_ACTIVE_OPMODE_BITS) < 0) {
	// 	LOG_DBG("Failed to set opmode.");
	// 	return -EIO;
	// }

	// /* set lux range */
	// if (i2c_reg_update_byte_dt(&config->i2c,
	// 			   ISL29035_COMMAND_II_REG,
	// 			   ISL29035_LUX_RANGE_MASK,
	// 			   ISL29035_LUX_RANGE_BITS) < 0) {
	// 	LOG_DBG("Failed to set lux range.");
	// 	return -EIO;
	// }

	// /* set ADC resolution */
	// if (i2c_reg_update_byte_dt(&config->i2c,
	// 			   ISL29035_COMMAND_II_REG,
	// 			   ISL29035_ADC_RES_MASK,
	// 			   ISL29035_ADC_RES_BITS) < 0) {
	// 	LOG_DBG("Failed to set ADC resolution.");
	// 	return -EIO;
	// }

#ifdef CONFIG_STTS75_TRIGGER
	// if (config->int_gpio.port) {
	// 	if (isl29035_init_interrupt(dev) < 0) {
	// 		LOG_DBG("Failed to initialize interrupt.");
	// 		return -EIO;
	// 	}
	// }
#endif

	return 0;
}

#define STTS75_DEFINE(inst)									\
	static struct stts75_driver_data stts75_data_##inst;				\
												\
	static const struct stts75_config stts75_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
		IF_ENABLED(CONFIG_STTS75_TRIGGER,						\
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),))	\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, &stts75_init, NULL,				\
			      &stts75_data_##inst, &stts75_config_##inst, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &stts75_api);			\

DT_INST_FOREACH_STATUS_OKAY(STTS75_DEFINE)
