#define DT_DRV_COMPAT st_l3g4200d

#include <drivers/sensor.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "l3g4200d.h"

LOG_MODULE_REGISTER(L3G4200D, CONFIG_SENSOR_LOG_LEVEL);

static inline int l3g4200d_reboot(const struct device *dev)
{
	struct l3g4200d_data *data = dev->data;

	if (data->hw_tf->update_reg(dev, L3G4200D_CTRL_REG5,
				    L3G4200D_MASK_CTRL5_C_BOOT,
				    1 << L3G4200D_SHIFT_CTRL5_C_BOOT) < 0) {
		return -EIO;
	}

	/* Wait sensor turn-on time as per datasheet */
	k_busy_wait(USEC_PER_MSEC * 35U);

	return 0;
}

static inline int l3g4200d_enable(const struct device *dev)
{
	struct l3g4200d_data *data = dev->data;

	if (data->hw_tf->update_reg(dev, L3G4200D_CTRL_REG1,
				L3G4200D_MASK_CTRL1_C_PD,
				L3G4200D_CTRL1_PD_NORMAL) < 0) {
		return -EIO;
	}

	return 0;
}

static int l3g4200d_sample_fetch_gyro(const struct device *dev)
{
	struct l3g4200d_data *data = dev->data;

	uint8_t buf[6];

	if (data->hw_tf->read_data(dev, L3G4200D_OUT_X_L,
				buf, sizeof(buf)) < 0) {
		LOG_ERR("failed to read gyroscope data");
		return -EIO;
	}

	data->gyro_sample_x = (buf[1] << 8) | buf[0];
	data->gyro_sample_y = (buf[3] << 8) | buf[2];
	data->gyro_sample_z = (buf[5] << 8) | buf[4];

	LOG_INF("%d %d %d", data->gyro_sample_x, data->gyro_sample_y, data->gyro_sample_z);
	return 0;
}

static int l3g4200d_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		//return lsm6dsl_accel_config(dev, chan, attr, val);
	case SENSOR_CHAN_GYRO_XYZ:
		//return lsm6dsl_gyro_config(dev, chan, attr, val);
	default:
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int l3g4200d_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_GYRO_XYZ);

	switch (chan) {
	case SENSOR_CHAN_GYRO_XYZ:
		l3g4200d_sample_fetch_gyro(dev);
		break;
#if defined(CONFIG_l3g4200d_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		l3g4200d_sample_fetch_temp(dev);
		break;
#endif
	case SENSOR_CHAN_ALL:
		l3g4200d_sample_fetch_gyro(dev);
#if defined(CONFIG_l3g4200d_ENABLE_TEMP)
		l3g4200d_sample_fetch_temp(dev);
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline void l3g4200d_gyro_convert(struct sensor_value *val, int raw_val,
					float sensitivity)
{
	double dval;

	/* Sensitivity is exposed in mdps/LSB */
	/* Convert to rad/s */
	dval = (double)(raw_val * sensitivity * SENSOR_DEG2RAD_DOUBLE / 1000);
	val->val1 = (int32_t)dval;
	val->val2 = (((int32_t)(dval * 1000)) % 1000) * 1000;
}

static inline int l3g4200d_gyro_get_channel(enum sensor_channel chan,
					   struct sensor_value *val,
					   struct l3g4200d_data *data,
					   float sensitivity)
{
	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
		l3g4200d_gyro_convert(val, data->gyro_sample_x, sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Y:
		l3g4200d_gyro_convert(val, data->gyro_sample_y, sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Z:
		l3g4200d_gyro_convert(val, data->gyro_sample_z, sensitivity);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		l3g4200d_gyro_convert(val, data->gyro_sample_x, sensitivity);
		l3g4200d_gyro_convert(val + 1, data->gyro_sample_y, sensitivity);
		l3g4200d_gyro_convert(val + 2, data->gyro_sample_z, sensitivity);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int l3g4200d_gyro_channel_get(enum sensor_channel chan,
				    struct sensor_value *val,
				    struct l3g4200d_data *data)
{
	return l3g4200d_gyro_get_channel(chan, val, data,
					L3G4200D_DEFAULT_GYRO_SENSITIVITY);
}

static int l3g4200d_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct l3g4200d_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		l3g4200d_gyro_channel_get(chan, val, data);
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api l3g4200d_driver_api = {
	.attr_set = l3g4200d_attr_set,
#if CONFIG_L3G4200D_TRIGGER
	.trigger_set = l3g4200d_trigger_set,
#endif
	.sample_fetch = l3g4200d_sample_fetch,
	.channel_get = l3g4200d_channel_get,
};

static int l3g4200d_init_chip(const struct device *dev)
{
	struct l3g4200d_data *data = dev->data;
	// const struct l3g4200d_config *config = dev->config;
	uint8_t chip_id;

	if (l3g4200d_reboot(dev) < 0) {
		LOG_DBG("failed to reboot device");
		return -EIO;
	}

	if (data->hw_tf->read_reg(dev, L3G4200D_WHO_AM_I, &chip_id) < 0) {
		LOG_DBG("failed reading chip id");
		return -EIO;
	}
	if (chip_id != L3G4200D_VAL_WHO_AM_I) {
		LOG_DBG("invalid chip id 0x%x", chip_id);
		return -EIO;
	}

	LOG_INF("chip id 0x%x", chip_id);

	if (l3g4200d_enable(dev) < 0) {
		LOG_ERR("failed enable normal mode");
		return -EIO;
	};
	return 0;
}

static int l3g4200d_init(const struct device *dev)
{
	const struct l3g4200d_config *config = dev->config;
	struct l3g4200d_data *data = dev->data;

	data->bus = device_get_binding(config->bus_name);
	if (!data->bus) {
		LOG_ERR("bus not found %s", config->bus_name);
		return -EINVAL;
	}

	config->bus_init(dev);

	if (l3g4200d_init_chip(dev) < 0) {
		LOG_ERR("failed to initialize chip");
		return -EIO;
	}

#ifdef CONFIG_L3G4200D_TRIGGER
	if (l3g4200d_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt.");
		return -EIO;
	}
#endif

	return 0;
}


#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "L3G4200D driver enabled without any devices"
#endif

/*
 * Device creation macro, shared by L3G4200D_DEFINE_SPI() and
 * L3G4200D_DEFINE_I2C().
 */

#define L3G4200D_DEVICE_INIT(inst)					\
	DEVICE_DT_INST_DEFINE(inst,					\
			    l3g4200d_init,				\
			    device_pm_control_nop,			\
			    &l3g4200d_data_##inst,			\
			    &l3g4200d_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &l3g4200d_driver_api);

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#define L3G4200D_HAS_CS(inst) DT_INST_SPI_DEV_HAS_CS_GPIOS(inst)

#define L3G4200D_DATA_SPI_CS(inst)					\
	{ .cs_ctrl = {							\
		.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(inst),		\
		.gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(inst),	\
		},							\
	}

#define L3G4200D_DATA_SPI(inst)						\
	COND_CODE_1(L3G4200D_HAS_CS(inst),				\
		    (L3G4200D_DATA_SPI_CS(inst)),			\
		    ({}))

#define L3G4200D_SPI_CS_PTR(inst)					\
	COND_CODE_1(L3G4200D_HAS_CS(inst),				\
		    (&(l3g4200d_data_##inst.cs_ctrl)),			\
		    (NULL))

#define L3G4200D_SPI_CS_LABEL(inst)					\
	COND_CODE_1(L3G4200D_HAS_CS(inst),				\
		    (DT_INST_SPI_DEV_CS_GPIOS_LABEL(inst)), (NULL))

#define L3G4200D_SPI_CFG(inst)						\
	(&(struct l3g4200d_spi_cfg) {					\
		.spi_conf = {						\
			.frequency = DT_INST_PROP(inst, spi_max_frequency),	\
			.operation = (SPI_WORD_SET(8) |			\
				      SPI_OP_MODE_MASTER |		\
				      SPI_MODE_CPOL |			\
				      SPI_MODE_CPHA),			\
			.slave = DT_INST_REG_ADDR(inst),		\
			.cs = L3G4200D_SPI_CS_PTR(inst),			\
		},							\
		.cs_gpios_label = L3G4200D_SPI_CS_LABEL(inst),		\
	})

#ifdef CONFIG_L3G4200D_TRIGGER
#define L3G4200D_CFG_IRQ(inst) \
		.irq_dev_name = DT_INST_GPIO_LABEL(inst, irq_gpios),	\
		.irq_pin = DT_INST_GPIO_PIN(inst, irq_gpios),		\
		.irq_flags = DT_INST_GPIO_FLAGS(inst, irq_gpios),
#else
#define L3G4200D_CFG_IRQ(inst)
#endif /* CONFIG_L3G4200D_TRIGGER */

#define L3G4200D_CONFIG_SPI(inst)					\
	{								\
		.bus_name = DT_INST_BUS_LABEL(inst),			\
		.bus_init = l3g4200d_spi_init,				\
		.bus_cfg = { .spi_cfg = L3G4200D_SPI_CFG(inst)	},	\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios),	\
		(L3G4200D_CFG_IRQ(inst)), ())				\
	}

#define L3G4200D_DEFINE_SPI(inst)					\
	static struct l3g4200d_data l3g4200d_data_##inst =		\
		L3G4200D_DATA_SPI(inst);					\
	static const struct l3g4200d_config l3g4200d_config_##inst =	\
		L3G4200D_CONFIG_SPI(inst);				\
	L3G4200D_DEVICE_INIT(inst)

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define L3G4200D_CONFIG_I2C(inst)					\
	{								\
		.bus_name = DT_INST_BUS_LABEL(inst),			\
		.bus_init = l3g4200d_i2c_init,				\
		.bus_cfg = { .i2c_slv_addr = DT_INST_REG_ADDR(inst), },	\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios),	\
		(L3G4200D_CFG_IRQ(inst)), ())				\
	}

#define L3G4200D_DEFINE_I2C(inst)					\
	static struct l3g4200d_data l3g4200d_data_##inst;			\
	static const struct l3g4200d_config l3g4200d_config_##inst =	\
		L3G4200D_CONFIG_I2C(inst);				\
	L3G4200D_DEVICE_INIT(inst)
/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define L3G4200D_DEFINE(inst)						\
	COND_CODE_1(DT_INST_ON_BUS(inst, spi),				\
		    (L3G4200D_DEFINE_SPI(inst)),				\
		    (L3G4200D_DEFINE_I2C(inst)))

DT_INST_FOREACH_STATUS_OKAY(L3G4200D_DEFINE)
