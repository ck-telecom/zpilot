Overview
********
zephyr based AHRS

Hardware
********
MCU: stm32f103cbt6

GYROSCOPE: l3g4200d

ACCEL + MAGN : LSM303DLH

BAROMETER: bmp085

Building and flash
********************
```
west build -p auto -b kite .
west flash
```
