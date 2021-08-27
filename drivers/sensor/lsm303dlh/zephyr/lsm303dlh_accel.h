#ifndef _LSM303DLH_ACCEL_H
#define _LSM303DLH_ACCEL_H

#define LSM303DLH_CTRL_REG1_A       0x20
#define LSM303DLH_CTRL_REG2_A       0x21
#define LSM303DLH_CTRL_REG3_A       0x22
#define LSM303DLH_CTRL_REG4_A       0x23
#define LSM303DLH_CTRL_REG5_A       0x24
#define LSM303DLH_HP_FILTER_RESET_A 0x25
#define LSM303DLH_REFERENCE_A       0x26
#define LSM303DLH_STATUS_REG_A      0x27

#define LSM303DLH_OUT_X_L_A         0x28
#define LSM303DLH_OUT_X_H_A         0x29
#define LSM303DLH_OUT_Y_L_A         0x2A
#define LSM303DLH_OUT_Y_H_A         0x2B
#define LSM303DLH_OUT_Z_L_A         0x2C
#define LSM303DLH_OUT_Z_H_A         0x2D

#define LSM303DLH_INT1_CFG_A        0x30
#define LSM303DLH_INT1_SRC_A        0x31
#define LSM303DLH_INT1_THS_A        0x32
#define LSM303DLH_INT1_DURATION_A   0x33
#define LSM303DLH_INT2_CFG_A        0x34
#define LSM303DLH_INT2_SRC_A        0x35
#define LSM303DLH_INT2_THS_A        0x36
#define LSM303DLH_INT2_DURATION_A   0x37

struct lsm303dlhc_accel_data {
	const struct device *i2c;

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
};

struct lsm303dlhc_accel_config {
	char *i2c_name;
	uint8_t i2c_address;
};
#endif /* _LSM303DLH_ACCEL_H */