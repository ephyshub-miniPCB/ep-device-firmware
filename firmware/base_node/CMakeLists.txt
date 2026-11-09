cmake_minimum_required(VERSION 3.20.0)

# Default `west flash`/`west debug` to the nrfutil runner rather than nrfjprog
# (the legacy nRF Command Line Tools, superseded by nrfutil). Pass `-r <runner>`
# to west to use a different one.
set(BOARD_FLASH_RUNNER nrfutil)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(base_node)

target_include_directories(app PRIVATE src)
target_sources(app PRIVATE src/ble_central.c src/ep_info.c src/ep_request.c src/link_test.c src/shell_cmds.c src/usb_stream.c main/main.c)
