
#define DT_DRV_COMPAT st_lsm303dlh_accel

#include <drivers/i2c.h>
#include <init.h>
#include <drivers/sensor.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(lsm303dlh_accel, CONFIG_SENSOR_LOG_LEVEL);

#include "lsm303dlh_accel.h"

static int lsm303dlhc_sample_fetch(const struct device *dev,
				   enum sensor_channel chan)
{
	const struct lsm303dlhc_accel_config *config = dev->config;
	struct lsm303dlhc_accel_data *drv_data = dev->data;
	uint8_t accel_buf[6];
	uint8_t status;

	i2c_reg_read_byte(drv_data->i2c, config->i2c_address, LSM303DLH_OUT_X_L_A, &accel_buf[0]);
	i2c_reg_read_byte(drv_data->i2c, config->i2c_address, LSM303DLH_OUT_X_H_A, &accel_buf[1]);
	i2c_reg_read_byte(drv_data->i2c, config->i2c_address, LSM303DLH_OUT_Y_L_A, &accel_buf[2]);
	i2c_reg_read_byte(drv_data->i2c, config->i2c_address, LSM303DLH_OUT_Y_H_A, &accel_buf[3]);
	i2c_reg_read_byte(drv_data->i2c, config->i2c_address, LSM303DLH_OUT_Z_L_A, &accel_buf[4]);
	i2c_reg_read_byte(drv_data->i2c, config->i2c_address, LSM303DLH_OUT_Z_H_A, &accel_buf[5]);

	drv_data->accel_x = (accel_buf[1] << 8) | accel_buf[0];
	drv_data->accel_y = (accel_buf[3] << 8) | accel_buf[2];
	drv_data->accel_z = (accel_buf[5] << 8) | accel_buf[4];

	LOG_INF("%d %d %d", drv_data->accel_x, drv_data->accel_y, drv_data->accel_z);
	return 0;
}

static void lsm303dlhc_convert(struct sensor_value *val,
			       int64_t raw_val)
{
//	double dval;

//	dval = (double)(raw_val) * 2.0 * SENSOR_G / 32768.0;
//val->val1 = (int32_t)dval;
//	val->val2 = ((int32_t)(dval * 1000000)) % 1000000;
	raw_val = (raw_val * SENSOR_G) / (0x8000 / 2);
//	raw_val = raw_val * 2  * SENSOR_G / 0x8000;
	val->val1 = raw_val / 1000000;
	val->val2 = raw_val % 1000000;
}

static int lsm303dlhc_channel_get(const struct device *dev,
				  enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct lsm303dlhc_accel_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		lsm303dlhc_convert(val, drv_data->accel_x);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		lsm303dlhc_convert(val, drv_data->accel_y);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		lsm303dlhc_convert(val, drv_data->accel_z);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		lsm303dlhc_convert(val, drv_data->accel_x);
		lsm303dlhc_convert(val + 1, drv_data->accel_y);
		lsm303dlhc_convert(val + 2, drv_data->accel_z);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static const struct sensor_driver_api lsm303dlhc_accel_driver_api = {
	.sample_fetch = lsm303dlhc_sample_fetch,
	.channel_get = lsm303dlhc_channel_get,
};

static int lsm303dlhc_accel_init(const struct device *dev)
{
	const struct lsm303dlhc_accel_config *config = dev->config;
	struct lsm303dlhc_accel_data *drv_data = dev->data;

	drv_data->i2c = device_get_binding(config->i2c_name);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Could not get pointer to %s device",
			    config->i2c_name);
		return -ENODEV;
	}
#if 0
	/* Set accelerometer output data rate */
	if (i2c_reg_update_byte(drv_data->i2c,
			       config->i2c_address,
			       LSM303DLH_CTRL_REG1_A,
			       LSM303DLH_ACCEL_ODR_MASK,
			       LSM303DLH_ACCEL_ODR_BITS) < 0) {
		LOG_ERR("Failed to configure chip.");
		return -EIO;
	}

	/* Set accelerometer full scale range */
	if (i2c_reg_update_byte(drv_data->i2c,
			       config->i2c_address,
			       LSM303DLH_CTRL_REG4_A,
			       LSM303DLH_ACCEL_RANGE_MASK,
			       LSM303DLH_ACCEL_RANGE_BITS) < 0) {
		LOG_ERR("Failed to set magnetometer full scale range.");
		return -EIO;
	}
#endif
	if (i2c_reg_update_byte(drv_data->i2c,
			       config->i2c_address,
			       LSM303DLH_CTRL_REG1_A,
			       LSM303DLH_ACCEL_PM_MASK,
			       LSM303DLH_ACCEL_PM_BITS) < 0) {
		LOG_ERR("Failed to set lsm3030dlh full scale range.");
		return -EIO;
	}
	return 0;
}

static const struct lsm303dlhc_accel_config lsm303dlhc_accel_config = {
	.i2c_name = DT_INST_BUS_LABEL(0),
	.i2c_address = DT_INST_REG_ADDR(0),
};

static struct lsm303dlhc_accel_data lsm303dlhc_accel_driver;

DEVICE_DT_INST_DEFINE(0, lsm303dlhc_accel_init, device_pm_control_nop,
		    &lsm303dlhc_accel_driver,
		    &lsm303dlhc_accel_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &lsm303dlhc_accel_driver_api);
