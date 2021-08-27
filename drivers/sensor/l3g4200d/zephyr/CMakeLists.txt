# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(.)

zephyr_library_sources_ifdef(CONFIG_L3G4200D l3g4200d.c)
zephyr_library_sources_ifdef(CONFIG_L3G4200D l3g4200d_spi.c)
zephyr_library_sources_ifdef(CONFIG_L3G4200D_TRIGGER cst816s_trigger.c)