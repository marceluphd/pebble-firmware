#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

cmake_minimum_required(VERSION 3.8.2)

include(../../cmake/boilerplate.cmake)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(asset_tracker)
zephyr_compile_definitions(PROJECT_NAME=${PROJECT_NAME})

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)

# BME680
target_sources(app PRIVATE src/bme/bme680.c)
target_sources(app PRIVATE src/bme/bme680_helper.c)

# HAL
target_sources(app PRIVATE src/hal/hal_adc.c)
target_sources(app PRIVATE src/hal/hal_gpio.c)

# Modem
target_sources(app PRIVATE src/modem/modem_helper.c)

# MQTT
target_sources(app PRIVATE src/mqtt/mqtt.c)

# Protbuf
target_sources(app PRIVATE src/nanopb/pb_encode.c)
target_sources(app PRIVATE src/nanopb/pb_decode.c)
target_sources(app PRIVATE src/nanopb/pb_common.c)

# Local NVS
target_sources(app PRIVATE src/nvs/local_storage.c)

# Unittest
target_sources(app PRIVATE src/unittest/unittest.c)
target_sources(app PRIVATE src/unittest/nvs_unittest.c)
target_sources(app PRIVATE src/unittest/nanopb_unittest.c)
target_sources(app PRIVATE src/unittest/nanopb_unittest.pb.c)

target_sources(app PRIVATE src/icm/icm42605_helper.c)
target_sources(app PRIVATE src/icm/Icm426xxDriver_HL_apex.c)
target_sources(app PRIVATE src/icm/Icm426xxDriver_HL.c)
target_sources(app PRIVATE src/icm/Icm426xxTransport.c)
# NORDIC SDK APP END
zephyr_include_directories(src)

# Include application events and configuration headers
zephyr_library_include_directories(
  src/orientation_detector
  src/cloud_codec
  src/unittest
  src/nanopb
  src/mqtt
  src/nvs
  src/ui
  src
  )

# Application sources
add_subdirectory(src/orientation_detector)
add_subdirectory(src/ui)
add_subdirectory(src/cloud_codec)
add_subdirectory(src/gps_controller)
add_subdirectory(src/env_sensors)

