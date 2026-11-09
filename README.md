# Wireless RHS2116 stimulation/recording system

Firmware and PC software for a wireless neural recording and stimulation system built
around the Intan RHS2116 and the nRF52840.

| Folder | Contents |
| --- | --- |
| `firmware/ep_device/` | Implant-side ("ep device") firmware: drives the RHS2116, records, stimulates and streams over BLE |
| `firmware/base_node/` | USB-connected BLE central: shell for control on the first CDC ACM port, binary data stream on the second |
| `firmware/modules/rhs2116/` | Out-of-tree Zephyr module for the RHS2116 (see its README) |
| `firmware/pc_side/` | Python GUI that controls the system through the base node and plots the recording |

## Requirements

- Zephyr 3.7.0 (with `west`) and Zephyr SDK 0.16.8 (0.17.x fails to build Zephyr 3.7 with picolibc)
- `nrfutil` for flashing nRF52840 DK targets (the CMake files select the nrfutil runner)
- Python 3 for the GUI

## Building

Run from the `firmware/` directory with the Zephyr environment set up.

ep device, on the rev1 PCB (Seeed XIAO nRF52840) or the nRF52840 DK:

```bash
west build -b xiao_ble ep_device -d build/ep_device
```

```bash
west build -b nrf52840dk/nrf52840 ep_device -d build/ep_device_dk
```

Optional overlays for `ep_device`, passed as `-- -DEXTRA_CONF_FILE=...`:

- `emul.conf` builds an RHS2116 emulator in place of the chip, for bench testing without hardware
- `debug.conf` enables asserts and thread names for crash debugging
- `selftest.conf` runs the SPI transport self-test at boot (about a minute)

Base node, on a nice!nano (built as the pin- and bootloader-compatible Adafruit ItsyBitsy nRF52840):

```bash
west build -b adafruit_itsybitsy_nrf52840 base_node -d build/base_node
```

The XIAO and nice!nano builds produce `zephyr/zephyr.uf2`, which is copied onto the board's
UF2 bootloader drive.

## Host-side unit tests

```bash
make -C firmware/ep_device/tests/host run
```

## PC GUI

```bash
pip install -r firmware/pc_side/requirements.txt
```

```bash
python3 firmware/pc_side/ep_gui.py
```

Plug in the base node first. The GUI sends commands to the base node's shell port and reads
the recording from its stream port.
