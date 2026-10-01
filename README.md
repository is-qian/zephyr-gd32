# Zephyr firmware: Power Board MCU (GD32C231F8P6TR)

The GD32 on the power board runs as a device on the Dynamixel single-wire
half-duplex bus (protocol 2.0, ID 200, 1 Mbps), sharing the bus with XL330
servos without interference: it only answers its own ID and supports
PING / READ / WRITE / REBOOT / SYNC READ / FAST SYNC READ (0x8A, taking its
slot in the ID list, either answering first or appending after earlier
devices).

The register map matches the `imu_to_dxl` v2 board contract used by robotd
(the rustypot host stack); with `bus.fast_sync_read = true` this board can
sit on the bus directly.

## Control table (little-endian)

| Address | Length | Content | Access |
|---|---|---|---|
| 0 | 2 | model_number = 0x4D44 | R |
| 2 | 4 | model_information = 0 | R |
| 6 | 1 | firmware_version | R |
| 7 | 1 | ID (default 200, writable, resets on power cycle) | RW |
| 8 | 1 | baud_rate = 3 (1 Mbps) | R |
| 9 | 1 | return_delay_time = 0 | RW (ignored) |
| 124 | 6 | gyro x/y/z, i16, +/-500 dps (17.5 mdps/LSB) | R |
| 130 | 6 | SFLP quaternion x/y/z, IEEE half-float (w = sqrt(1-x^2-y^2-z^2)) | R |
| 136 | 6 | accel x/y/z, i16, +/-2 g (0.061 mg/LSB) | R |
| 142 | 1 | sample_counter (+1 per new quaternion) | R |
| 143 | 1 | status: bit0 IMU online, bit1 quaternion valid, bit2 FIFO overflow | R |
| 144 | 2 | present_input_voltage, u16, 0.1 V/count (Vbat) | R |
| 146 | 1 | present_temperature, u8, deg C (IMU die) | R |

- Read everything at once: `READ 124, len 23` (124..146 in one go).
- Read a single value: `READ <address>, len <register length>`.
- The 12-byte block for the 50 Hz tick (gyro + quaternion) is
  `READ 124, len 12`, same address for SYNC READ / FAST SYNC READ.

## Sensor configuration

- `PA0/ADC_CTL` is driven high at boot; `PA1/ADC_IN1` samples the Vbat
  divider (Vbat = Vadc x 2.8), 32-sample average.
- `PB6/PB7` I2C0 reads the `LSM6DSV16X` (address `0x6A`, +/-2 g /
  +/-500 dps / 120 Hz); SFLP game rotation runs at 120 Hz and the
  quaternion comes out of the FIFO (tag 0x13).
- The bus engine owns the main thread; sensor I2C/ADC work runs in its own
  thread and only while the bus has been idle for >= 2 ms, so a stuck
  sensor can never block bus answers.

## Single-wire half-duplex

The on-board SN74LVC1G125/126 pair derives the bus direction from TXD
(drives the bus while TXD is low, releases to the pull-up while high), so
the firmware needs no direction pin and never hears its own echo. TXD/RXD
use pins 16/17; `boards/gd32c231f8_dk/board.c` sets the SYSCFG
`PA11_RMP`/`PA12_RMP` bits to remap them to PA9/PA10.

Note: USART0 TX/RX only exist on PA9/PA10, which have no physical pins in
the TSSOP20 package; pins 16/17 are PA11/PA12 by default.

Also note: PA14 shares pin 19 with BOOT0, so with a J-Link attached a
reset may sample BOOT0=1 and enter the ROM bootloader; unplug the J-Link
and power-cycle for standalone runs.

## Zephyr tree

Upstream Zephyr has no GD32C2X1 support yet; this project uses the
GigaDevice official fork. The workspace lives in `zephyrproject/` under
this directory (gitignored, not committed):

- Source: `GD32-MCU-IOT/gd32_zephyr` (with `soc/gd/gd32/gd32c2x1`)
- HAL: `GD32-MCU-IOT/hal_gigadevice_zephyr`

The forks need local patches to build/run this project. Patches live in
`patches/` (directory name = repo path inside the workspace); see each
patch header for details:

- `patches/gd32_zephyr/` -> apply to `<zephyrproject>/zephyr`: ADC driver
  support for C2X1 (sample-time table, no calibration sequence, CK_SYS/2
  clock, software trigger, CTL0_DRES resolution); the build fails without it;
- `patches/hal_gigadevice_zephyr/` -> apply to
  `<zephyrproject>/modules/hal/gigadevice`: fix the I2C0 PB6/PB7 AF numbers
  (I2C0_SCL_PB6/I2C0_SDA_PB7 = AF6, I2C0_SCL_PB7 = AF15,
  I2C0_SMBA_PB6 = AF7); the original AF4 values disagree with datasheet
  Table 2-13 and hang the I2C0 master FSM after START (no ACK/NACK).

Bootstrap on a new machine (only cmsis / cmsis_6 / hal_gigadevice modules
are needed; run inside this directory):

```bash
west init -m https://github.com/GD32-MCU-IOT/gd32_zephyr --mr main zephyrproject
cd zephyrproject
west update cmsis cmsis_6 hal_gigadevice
git -C zephyr apply ../patches/gd32_zephyr/*.patch
git -C modules/hal/gigadevice apply ../patches/hal_gigadevice_zephyr/*.patch
```

(If the patches are already applied, `git apply` reports an error; that is
expected.)

## Target board

Custom board `gd32c231f8_dk` (under `boards/`, pulled in via `BOARD_ROOT`):

- `dts/arm/gd/gd32c2x1/gd32c231f8.dtsi`: 64K flash, adds the `i2c0`
  (gd,gd32-i2c-v2) and `adc0` nodes missing from the fork;
- `gd32c231f8_dk.dts`: peripheral enables and pinmux (pinctrl macros come
  from the HAL header `gd32c231f(6)xx-pinctrl.h`).

## Build and flash

The fork wants Zephyr SDK 1.x; if your SDK is older, build with the system
`arm-none-eabi-gcc` (gnuarmemb) instead:

```bash
cd zephyr-gd32/zephyrproject
ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb GNUARMEMB_TOOLCHAIN_PATH=/usr \
  west build -b gd32c231f8_dk .. -d ../build -p always
ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb GNUARMEMB_TOOLCHAIN_PATH=/usr \
  west flash -d ../build --runner jlink
```

Artifacts: `build/zephyr/zephyr.{elf,bin,hex}` (about FLASH 22K / RAM 8.7K).
The raw `zephyr.bin` carries no address; flash it at `0x08000000`
(`loadbin zephyr.bin 0x08000000`). The hex/elf embed the address.

## Bus self-test

Without a Dynamixel host, any 3.3 V USB-UART can drive the single DATA
wire: `UART_TX -- 1K resistor -- DATA`, `UART_RX -- DATA`, common GND, then:

```bash
python3 tools/dxl_probe.py /dev/ttyUSB0
```

The script PINGs, reads each register, then reads 124..146 as one block.

## CI

`.github/workflows/firmware.yml` builds the firmware and publishes a
release named after the build date. Trigger it manually from the Actions
tab ("Run workflow").
