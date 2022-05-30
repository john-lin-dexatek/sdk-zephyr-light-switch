/*
 * Copyright (c) 2020 NXP
 * Copyright (c) 2020 Mark Olsson <mark@markolsson.se>
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT cypress_cy8cmbr3108

#include <drivers/kscan.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(cy8cmbr3108, CONFIG_KSCAN_LOG_LEVEL);

#include "cypress_vendor_code/CY8CMBR3xxx_APIs.h"
#include "cypress_vendor_code/CY8CMBR3xxx_Registers.h"

/** GPIO DT information. */
struct gpio_dt_info {
	/** Port. */
	const char *port;
	/** Pin. */
	gpio_pin_t pin;
	/** Flags. */
	gpio_dt_flags_t flags;
};

/** CY8CMBR3108 configuration (DT). */
struct cy8cmbr3108_config {
	/** I2C controller device name. */
	char *i2c_name;
	/** I2C chip address. */
	uint8_t i2c_address;
#ifdef CONFIG_KSCAN_CY8CMBR3108_INTERRUPT
	/** Interrupt GPIO information. */
	struct gpio_dt_info int_gpio;
#endif
};

/** CY8CMBR3108 data. */
struct cy8cmbr3108_data {
	/** Device pointer. */
	const struct device *dev;
	/** I2C controller device. */
	const struct device *i2c;
	/** KSCAN Callback. */
	kscan_callback_t callback;
	/** Work queue (for deferred read). */
	struct k_work work;
#ifdef CONFIG_KSCAN_CY8CMBR3108_INTERRUPT
	/** Interrupt GPIO controller. */
	const struct device *int_gpio;
	/** Interrupt GPIO callback. */
	struct gpio_callback int_gpio_cb;
#else
	/** Timer (polling mode). */
	struct k_timer timer;
#endif
};

static int cy8cmbr3108_process(const struct device *dev)
{
	const struct cy8cmbr3108_config *config = dev->config;
	struct cy8cmbr3108_data *data = dev->data;
	int r;
	uint8_t event;
	uint8_t row=0xFF, col=0xFF;
	bool pressed;
	uint8_t localReadBuffer[CY8CMBR3xxx_SENSOR_STATUS_LENGTH];
	uint8_t retry_count=3;
	uint8_t index;
	typedef struct {
		int8_t x;
		int8_t y;
	} coordinate_type;
	coordinate_type layout_map[] = {
		{0, 1},
		{0, 0},
		{1, 1},
		{-1, -1},
		{2, 0},
		{2, 1},
		{1, 0},
		{-1, -1},		
	};
	CY8CMBR3xxx_SENSORSTATUS sensor_status;

	/* obtain number of touch points (NOTE: multi-touch ignored) */
	while (retry_count) {
		r = i2c_burst_read(data->i2c, config->i2c_address, CY8CMBR3xxx_BUTTON_STAT, localReadBuffer, CY8CMBR3xxx_SENSOR_STATUS_LENGTH);
		if (r < 0) {
			k_busy_wait(10000);//10ms
			retry_count--;
		}
		else {
			break;
		}
	}

	parse_sensor_status(localReadBuffer, &sensor_status);
	LOG_DBG("buttonStatus 0x%04x", sensor_status.buttonStatus);

	for (index = 0; index < sizeof(layout_map)/sizeof(coordinate_type); index++) {
		if (sensor_status.buttonStatus & (0x01 << index)) {
			row = layout_map[index].x;
			col = layout_map[index].y;
			break;
		}
	}
	
	if (row != 0xFF) {
		event = (uint8_t)sensor_status.buttonStatus;
		pressed = true;
		LOG_DBG("event: %d, row: %d, col: %d", event, row, col);
		data->callback(dev, row, col, pressed);
	}
	return 0;
}

static void cy8cmbr3108_work_handler(struct k_work *work)
{
	struct cy8cmbr3108_data *data = CONTAINER_OF(work, struct cy8cmbr3108_data, work);

	cy8cmbr3108_process(data->dev);
}

#ifdef CONFIG_KSCAN_CY8CMBR3108_INTERRUPT
static void cy8cmbr3108_isr_handler(const struct device *dev,
			       struct gpio_callback *cb, uint32_t pins)
{
	struct cy8cmbr3108_data *data = CONTAINER_OF(cb, struct cy8cmbr3108_data, int_gpio_cb);

	k_work_submit(&data->work);
}
#else
static void cy8cmbr3108_timer_handler(struct k_timer *timer)
{
	struct cy8cmbr3108_data *data = CONTAINER_OF(timer, struct cy8cmbr3108_data, timer);

	k_work_submit(&data->work);
}
#endif

static int cy8cmbr3108_configure(const struct device *dev,
			    kscan_callback_t callback)
{
	struct cy8cmbr3108_data *data = dev->data;

	if (!callback) {
		LOG_ERR("Invalid callback (NULL)");
		return -EINVAL;
	}

	data->callback = callback;

	return 0;
}

static int cy8cmbr3108_enable_callback(const struct device *dev)
{
	struct cy8cmbr3108_data *data = dev->data;

#ifdef CONFIG_KSCAN_CY8CMBR3108_INTERRUPT
	gpio_add_callback(data->int_gpio, &data->int_gpio_cb);
#else
	k_timer_start(&data->timer, K_MSEC(CONFIG_KSCAN_CY8CMBR3108_PERIOD),
		      K_MSEC(CONFIG_KSCAN_CY8CMBR3108_PERIOD));
#endif

	return 0;
}

static int cy8cmbr3108_disable_callback(const struct device *dev)
{
	struct cy8cmbr3108_data *data = dev->data;

#ifdef CONFIG_KSCAN_CY8CMBR3108_INTERRUPT
	gpio_remove_callback(data->int_gpio, &data->int_gpio_cb);
#else
	k_timer_stop(&data->timer);
#endif

	return 0;
}

const unsigned char CY8CMBR3108_configuration[128] = {
    0x77u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x80u, 0x80u, 0x80u, 0x7Fu,
    0x80u, 0x80u, 0x80u, 0x7Fu, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x01u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x80u,
    0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x02u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x1Eu, 0x1Eu, 0x00u,
    0x00u, 0x1Eu, 0x1Eu, 0x00u, 0x00u, 0x00u, 0x01u, 0x01u,
    0x00u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x24u, 0x03u, 0x01u, 0x69u,
    0x00u, 0x37u, 0x06u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x49u, 0x25u
};

static int cy8cmbr3108_init(const struct device *dev)
{
	const struct cy8cmbr3108_config *config = dev->config;
	struct cy8cmbr3108_data *data = dev->data;
	int r = -1;

	data->i2c = device_get_binding(config->i2c_name);
	if (!data->i2c) {
		LOG_ERR("Could not find I2C controller");
		return -ENODEV;
	}

	data->dev = dev;

	k_work_init(&data->work, cy8cmbr3108_work_handler);

	/* init registers */ 
	while (r < 0) {
		r = i2c_burst_write(data->i2c, config->i2c_address, 0x00, 
			CY8CMBR3108_configuration, sizeof(CY8CMBR3108_configuration));
		k_busy_wait(10000);//10ms
	}

#ifdef CONFIG_KSCAN_CY8CMBR3108_INTERRUPT
	int r;

	data->int_gpio = device_get_binding(config->int_gpio.port);
	if (!data->int_gpio) {
		LOG_ERR("Could not find interrupt GPIO controller");
		return -ENODEV;
	}

	r = gpio_pin_configure(data->int_gpio, config->int_gpio.pin,
			       config->int_gpio.flags | GPIO_INPUT);
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO pin");
		return r;
	}

	r = gpio_pin_interrupt_configure(data->int_gpio, config->int_gpio.pin,
					 GPIO_INT_EDGE_TO_ACTIVE);
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO interrupt.");
		return r;
	}

	gpio_init_callback(&data->int_gpio_cb, cy8cmbr3108_isr_handler,
			   BIT(config->int_gpio.pin));
#else
	k_timer_init(&data->timer, cy8cmbr3108_timer_handler, NULL);
#endif

	return 0;
}

static const struct kscan_driver_api cy8cmbr3108_driver_api = {
	.config = cy8cmbr3108_configure,
	.enable_callback = cy8cmbr3108_enable_callback,
	.disable_callback = cy8cmbr3108_disable_callback,
};

#define DT_INST_GPIO(index, gpio_pha)					       \
	{								       \
		DT_INST_GPIO_LABEL(index, gpio_pha),			       \
		DT_INST_GPIO_PIN(index, gpio_pha),			       \
		DT_INST_GPIO_FLAGS(index, gpio_pha),			       \
	}

#ifdef CONFIG_KSCAN_CY8CMBR3108_INTERRUPT
#define CY8CMBR3108_DEFINE_CONFIG(index)					       \
	static const struct cy8cmbr3108_config cy8cmbr3108_config_##index = {	       \
		.i2c_name = DT_INST_BUS_LABEL(index),			       \
		.i2c_address = DT_INST_REG_ADDR(index),			       \
		.int_gpio = DT_INST_GPIO(index, int_gpios)		       \
	}
#else
#define CY8CMBR3108_DEFINE_CONFIG(index)					       \
	static const struct cy8cmbr3108_config cy8cmbr3108_config_##index = {	       \
		.i2c_name = DT_INST_BUS_LABEL(index),			       \
		.i2c_address = DT_INST_REG_ADDR(index)			       \
	}
#endif

#define CY8CMBR3108_INIT(index)                                                     \
	CY8CMBR3108_DEFINE_CONFIG(index);					       \
	static struct cy8cmbr3108_data cy8cmbr3108_data_##index;			       \
	DEVICE_DT_INST_DEFINE(index, cy8cmbr3108_init, NULL,			       \
			    &cy8cmbr3108_data_##index, &cy8cmbr3108_config_##index,      \
			    POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,	       \
			    &cy8cmbr3108_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CY8CMBR3108_INIT)
