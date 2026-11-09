# RHS2116 Zephyr module

Out-of-tree Zephyr module (tested with Zephyr 3.7) for the Intan RHS2116 16-channel
stimulation/recording amplifier on nRF52 SoCs.

## Using it

Add the module before `find_package(Zephyr)` in the application's `CMakeLists.txt`:

```cmake
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/../modules/rhs2116)
```

Then add a devicetree node. The SPIM it names must be disabled, because the driver
programs the SPIM itself instead of going through Zephyr's SPI driver:

```dts
/ {
    ep_dev: rhs2116 {
        compatible = "intan,rhs2116";
        spim = <&spi2>;                          /* not SPIM3 (anomaly 198) */
        cs-gpios = <&gpio0 5 GPIO_ACTIVE_LOW>;
    };
};

&spi2 {
    compatible = "nordic,nrf-spim";
    status = "disabled";                         /* only its pinctrl is used */
};
```

`CONFIG_RHS2116` turns on by itself when that node exists. Set `CONFIG_NRFX_SPIMn=y`
for the SPIM you chose. Other options:

- `CONFIG_RHS2116_EMUL`: replaces the chip with a software model for bench testing.
  SPI still runs on the pins.
- `CONFIG_RHS2116_INIT_PRIORITY`: the device's init priority.
- `CONFIG_RHS2116_LOG_LEVEL`: the driver's log level.

## API

Include `<rhs2116/...>`:

| Header | What it gives you |
| --- | --- |
| `rhs2116.h` | Task-level API, in physical units, for: chip ID and init, ADC rate, bandwidth, amplifier power, stimulation step size, charge recovery, trims, amplitudes, arm/disarm, immediate output, compliance and fault monitoring. Also the register-level `rhs2116_write_reg()` and `rhs2116_read_reg()`. |
| `rhs2116_regs.h` | The register map and field encoders. These are pure functions, so they can be tested on a PC. |
| `rhs2116_stream.h` | Continuous recording stream, paced by a timer through PPI, with a per-frame stimulation provider for frame-timed pulse trains. |
| `rhs2116_transport.h` | The SPIM/TIMER/PPI transport underneath the stream. Its stats are part of the stream stats. |

`firmware/ep_device` shows a complete application. Its `src/stim.c` builds pulse
trains on top of `rhs2116.h` and the stream's stimulation provider.

## Limitations

- **nRF52 only.** The transport uses SPIM, TIMER2/TIMER3, GPIOTE and PPI directly.
  TIMER2 and TIMER3 must be left free for the driver.
- **One chip.** The transport and stream keep their state in single static
  instances, and the build asserts there is exactly one `intan,rhs2116` node. The
  public API already takes a `const struct device *`, so supporting more chips
  would only change the inside of the driver.
- **No power management hooks.**

## Tests

The host test in `firmware/ep_device/tests/host` builds `rhs2116_stream.c` and
`rhs2116_ctrl.c` against Zephyr stubs, together with a model of the chip and the
SPIM ring:

```sh
make -C firmware/ep_device/tests/host run
```
