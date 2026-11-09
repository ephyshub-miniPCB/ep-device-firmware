cmake_minimum_required(VERSION 3.20.0)

# Default `west flash`/`west debug` to the nrfutil runner rather than nrfjprog
# (the legacy nRF Command Line Tools, superseded by nrfutil). Pass `-r <runner>`
# to west to use a different one.
set(BOARD_FLASH_RUNNER nrfutil)

# the RHS2116 driver is an out-of-tree Zephyr module (firmware/modules/rhs2116)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../modules/rhs2116)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ep_device)

target_include_directories(app PRIVATE src)
target_sources(app PRIVATE
  src/recording.c
  src/stim.c
  src/ble_handler.c
  src/cmd_handler.c
  src/device_info.c
  main/main.c
)

# Zephyr's APP_BUILD_VERSION is only refreshed when the Zephyr repo's index
# changes and never marks uncommitted changes, so generate our own build
# version on every build (see cmake/ep_version.cmake).
set(BUILD_VERSION_H ${CMAKE_CURRENT_BINARY_DIR}/generated/build_version.h)
add_custom_target(ep_build_version
  COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR} -DOUT_FILE=${BUILD_VERSION_H}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ep_version.cmake
  BYPRODUCTS ${BUILD_VERSION_H}
)
add_dependencies(app ep_build_version)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
