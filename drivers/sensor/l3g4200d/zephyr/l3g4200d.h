#ifndef L3G4200D_H
#define L3G4200D_H

#include <drivers/sensor.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>
#include <sys/util.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
// register addresses

#define L3G4200D_WHO_AM_I      0x0F

#define L3G4200D_CTRL_REG1     0x20
#define L3G4200D_CTRL_REG2     0x21
#define L3G4200D_CTRL_REG3     0x22
#define L3G4200D_CTRL_REG4     0x23
#define L3G4200D_CTRL_REG5     0x24
#define L3G4200D_REFERENCE     0x25
#define L3G4200D_OUT_TEMP      0x26
#define L3G4200D_STATUS_REG    0x27

#define L3G4200D_OUT_X_L       0x28
#define L3G4200D_OUT_X_H       0x29
#define L3G4200D_OUT_Y_L       0x2A
#define L3G4200D_OUT_Y_H       0x2B
#define L3G4200D_OUT_Z_L       0x2C
#define L3G4200D_OUT_Z_H       0x2D

#define L3G4200D_FIFO_CTRL_REG 0x2E
#define L3G4200D_FIFO_SRC_REG  0x2F

#define L3G4200D_INT1_CFG      0x30
#define L3G4200D_INT1_SRC      0x31
#define L3G4200D_INT1_THS_XH   0x32
#define L3G4200D_INT1_THS_XL   0x33
#define L3G4200D_INT1_THS_YH   0x34
#define L3G4200D_INT1_THS_YL   0x35
#define L3G4200D_INT1_THS_ZH   0x36
#define L3G4200D_INT1_THS_ZL   0x37
#define L3G4200D_INT1_DURATION 0x38

#define L3G4200D_VAL_WHO_AM_I       0xD3

#define L3G4200D_MASK_CTRL5_C_BOOT  BIT(7)
#define L3G4200D_SHIFT_CTRL5_C_BOOT 7

#define L3G4200D_MASK_CTRL1_C_PD    BIT(3)
#define L3G4200D_CTRL1_PD_NORMAL    (1 < 3)

/* Gyro sensor sensitivity grain is 8.75 mdps/LSB */
#define SENSI_GRAIN_G				(8750LL / 1000.0)
#define SENSOR_PI_DOUBLE			(SENSOR_PI / 1000000.0)
#define SENSOR_DEG2RAD_DOUBLE		(SENSOR_PI_DOUBLE / 180)

#if CONFIG_L3G4200D_GYRO_FS == 250
	#define L3G4200D_DEFAULT_GYRO_FULLSCALE     0
	#define L3G4200D_DEFAULT_GYRO_SENSITIVITY   SENSI_GRAIN_G
#elif CONFIG_LSM6DSL_GYRO_FS == 500
	#define L3G4200D_DEFAULT_GYRO_FULLSCALE     1
	#define L3G4200D_DEFAULT_GYRO_SENSITIVITY   (2.0 * SENSI_GRAIN_G)
#elif CONFIG_LSM6DSL_GYRO_FS == 2000
	#define L3G4200D_DEFAULT_GYRO_FULLSCALE     2
	#define L3G4200D_DEFAULT_GYRO_SENSITIVITY   (8.0 * SENSI_GRAIN_G)
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
struct l3g4200d_spi_cfg {
	struct spi_config spi_conf;
	const char *cs_gpios_label;
};
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

union l3g4200d_bus_cfg {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	uint16_t i2c_slv_addr;
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	const struct l3g4200d_spi_cfg *spi_cfg;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
};

struct l3g4200d_config {
	char *bus_name;
	int (*bus_init)(const struct device *dev);
	const union l3g4200d_bus_cfg bus_cfg;
#ifdef CONFIG_L3G4200D_TRIGGER
	char *irq_dev_name;
	uint32_t irq_pin;
	int irq_flags;
#endif
};

struct l3g4200d_data;

struct l3g4200d_transfer_function {
	int (*read_data)(const struct device *dev, uint8_t reg_addr,
			 uint8_t *value, uint8_t len);
	int (*write_data)(const struct device *dev, uint8_t reg_addr,
			  uint8_t *value, uint8_t len);
	int (*read_reg)(const struct device *dev, uint8_t reg_addr,
			uint8_t *value);
	int (*update_reg)(const struct device *dev, uint8_t reg_addr,
			  uint8_t mask, uint8_t value);
};

struct l3g4200d_data {
	const struct device *bus;
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_cs_control cs_ctrl;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
	int gyro_sample_x;
	int gyro_sample_y;
	int gyro_sample_z;
	float gyro_sensitivity;
	const struct l3g4200d_transfer_function *hw_tf;
#ifdef CONFIG_L3G4200D_TRIGGER
	const struct device *dev;
	const struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_L3G4200D_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_L3G4200D_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_L3G4200D_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_L3G4200D_TRIGGER */
};

int l3g4200d_spi_init(const struct device *dev);
int l3g4200d_i2c_init(const struct device *dev);

#ifdef CONFIG_L3G4200D_TRIGGER
int l3g4200d_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int l3g4200d_init_interrupt(const struct device *dev);
#endif

#endif

