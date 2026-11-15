# SPI -> BLE HIDS Keyboard Bridge

This sample turns an nRF52840 board into a BLE HID keyboard.
It receives keyboard reports from an external MCU over SPI slave (SPIS), then forwards them as BLE HID input reports.

## Expected SPI packet

Every SPI transaction is exactly 48 bytes. Input reports use this versioned
frame:

`B4 4B <type> 01 <sequence> <payload_len> 00 00 <payload/padding> <crc8> 96`

- `type 0x01`: 8-byte USB boot keyboard payload
- `type 0x02`: 31-byte NKRO payload (`modifiers + 240 key bits`)
- `type 0x03`: 2-byte little-endian Consumer usage
- `type 0x04`: 2-byte little-endian System Control usage

The payload starts at byte 8, CRC-8 is byte 46, and the tail is byte 47. CRC-8
uses polynomial `0x07`, initial value `0x00`, over bytes 0 through 45. The nRF
only acknowledges frames whose header, version, type-specific payload length,
CRC, and tail all validate.

The master retransmits an unchanged frame and sequence until the next SPI
transaction returns an ACK with that sequence and report token `0x00`. While a
BLE host is connected, the nRF does not issue that ACK until the report has
been accepted by the BLE transmit queue. Queue backpressure therefore makes
the QMK side retry instead of silently dropping an already acknowledged key
state. Duplicates are acknowledged but forwarded to BLE only once.

Bridge control commands use a validated 8-byte prefix followed by 40 zero bytes:

`C3 3C <cmd> <sequence> <~cmd> <~sequence> <crc8> 69`

The CRC-8 uses polynomial `0x07`, initial value `0x00`, over bytes 0 through 5.
The master retransmits the same command and sequence until the ACK echoes both;
the nRF executes a repeated command/sequence pair only once.

Supported commands are:

- `0x01`..`0x03`: switch active profile slot 1..3
- `0xF1`: record an explicit user action for the hostless power-off timer
- `0xF2`: disconnect current BLE link
- `0xF3`: clear bond in active profile slot
- `0xF4`: clear all bonds
- `0xF5`: report that no USB host is attached
- `0xF6`: report that a USB host is attached
- `0xF7`: wired mode; immediately disconnect BLE and stop advertising
- `0xF8`: wireless mode; resume BLE advertising and keyboard reports

A frame is treated as control only when its header, command, complements, CRC,
and tail all validate. ACK data occupies bytes 0 through 7 of the 48-byte MISO
frame; the remaining bytes are zero.

Each profile uses a different BLE identity (different MAC address), and appends
profile index to the device name.

- Profile 1/2/3 use different Bluetooth addresses.
- Device name is `HKB_1` / `HKB_2` / `HKB_3`.
- This prevents one host from taking over another profile through auto-reconnect.
- This firmware exposes NKRO keyboard, Consumer, and System reports. After
  upgrading, delete the old bond on each host and re-pair so the new HID report
  map is discovered.

## SPI mode

- 8-bit words
- MSB first
- mode 0 (`CPOL=0`, `CPHA=0`)
- nRF side is SPI slave

## Board overlay (current)

File: `boards/promicro_nrf52840_nrf52840.overlay`

Current SPIS pins:

- SCK: `P0.06`
- MISO: `P0.08`
- MOSI: `P0.17`
- CSN: `P0.20`

External VCC control:

- `P0.13` controls the ProMicro external VCC output.
- `P0.02` drives the TPS22919 `ON` input for the keyboard 5 V rail.
- Neither pin is GPIO-hogged. `power_manager_init()` configures both outputs
  and enables the rails on a cold start; automatic shutdown drives both low
  before entering System OFF.
- The nRF core remains powered when the keyboard rail is off, so it can wake
  the keyboard without a SPI command from the unpowered QMK MCU.

Power button and battery measurement:

- Connect a normally-open momentary button between `P0.29` and GND. The
  firmware enables the nRF52840 internal pull-up, so no external pull-up is
  required.
- While powered on, hold the button for about 1 second to release all BLE
  keys, disconnect, stop advertising, turn off `P0.02`/`P0.13`, and enter
  System OFF.
- The firmware waits for button release before enabling the active-low wake
  source. While off, hold the button for about 1 second to wake and reset the
  nRF52840; a short press is intentionally ignored as debounce/false-wake
  protection. Bonds remain stored in flash and the keyboard rails are enabled
  during startup.
- Battery sense: `VBAT -> 1 MOhm -> P0.31/AIN7 -> 1 MOhm -> GND`. The ADC
  reports an estimated percentage through the existing BLE Battery Service.
  Change `output-ohms` and `full-ohms` in the overlay if your resistor values
  differ; the divider ratio must match the fitted parts.

Automatic low-power policy:

- During wireless use, QMK sends an 8 ms SPI heartbeat while keys are active.
  After 30 seconds without a keyboard report change, the heartbeat becomes
  250 ms and the nRF requests a 30-50 ms BLE connection interval with slave
  latency 4. The next key report immediately requests the active 7.5 ms
  connection parameters again.
- If the nRF receives no SPI frame for 1 second while connected, it sends an
  all-keys-released report. This prevents a stuck BLE key if the QMK MCU loses
  power or the SPI link is interrupted; the first frame after recovery is
  always forwarded again.
- When no BLE host is connected, advertising stays fast at 100-150 ms for 2
  minutes, then changes to slow advertising at 1-1.2 seconds. A real key
  report change switches it back to fast advertising and restarts the timers.
- QMK reports USB presence separately from transport choice and refreshes a
  wired USB presence heartbeat every 5 seconds. With neither a USB host nor a
  BLE connection, 10 minutes without input turns off P0.02/P0.13 and enters
  System OFF. Automatic shutdown keeps the P0.29 wake source armed; hold the
  momentary button for about 1 second to wake it. A board without that button
  must still be power-cycled.
- Full System OFF remains available through the P0.29 long-press power action.
- HID routing is exclusive. QMK starts in wired standby with the radio silent;
  it selects Bluetooth only after an explicit `Fn+F8`, and `Fn+F7` selects wired
  mode. A newly configured USB host gets input ownership, while `Fn+F8` can
  still force Bluetooth while USB supplies power. Removing USB preserves the
  selected route and does not start Bluetooth by itself. The nRF waits for
  QMK's first validated transport command after reset and applies all radio,
  profile, disconnect, bond and settings work outside the SPI receive loop, so
  a control command cannot block the next ACK transaction. If the first
  advertising start is rejected while the radio is disconnected, the nRF
  retries through deferred work for at most 1.5 seconds instead of requiring a
  second `Fn+F8` press.
- QMK reasserts the selected transport after each 5-second recovery deadline,
  but waits for an 80 ms input-quiet gap and lets a new key preempt that
  recovery frame. This restores the current Fn+F7/Fn+F8 choice if the physical
  switch power-cycles only the nRF while the QMK MCU stays powered from USB.
- The P0.29 behavior above assumes a separate normally-open momentary button.
  If your board only has the latching switch that cuts nRF power, leave P0.29
  inactive and use the latching switch for a cold restart.
- BLE connection parameters are negotiated with the host. Check the RTT
  `Connection parameters` log to confirm the values accepted by each host.

Adjust these pins to match your nice!nano wiring.

## Build

From NCS root:

```sh
source zephyr/zephyr-env.sh
cmake -S nrf/samples/bluetooth/spi_hids_keyboard_bridge \
      -B build_spi_bridge \
      -GNinja \
      -DBOARD=promicro_nrf52840/nrf52840
cmake --build build_spi_bridge
```

For a ProMicro nRF52840 with an Adafruit-compatible UF2 bootloader, run this
from `/home/aier/ncs/v3.2.3` so the source and output paths match this project:

```sh
west build --no-sysbuild -p always \
  -d nrf/samples/bluetooth/spi_hids_keyboard_bridge/build_uf2 \
  -b promicro_nrf52840/nrf52840/uf2 \
  nrf/samples/bluetooth/spi_hids_keyboard_bridge \
  -- \
  -DDTC_OVERLAY_FILE=boards/promicro_nrf52840_nrf52840.overlay
```

The drag-and-drop image is
`nrf/samples/bluetooth/spi_hids_keyboard_bridge/build_uf2/zephyr/zephyr.uf2`.

On the first boot, RTT output should show the P0.29 button setup and then one
`Battery: sample=<mV>, filtered=<mV>, <percent>%` line immediately and every
30 seconds. Compare `sample` with a multimeter before relying on the percentage;
the percentage curve must be calibrated against the actual cell under the
keyboard's normal LED/radio load.

If board parsing fails with `No module named pykwalify`, install it in your active Python environment:

```sh
pip install pykwalify
```

## QMK side recommendation

On the QMK master MCU, keep CS low for all 48 bytes. The paired GMMK TKL
firmware sends at most one transaction per scan pass, retains a pending report
until the ACK sequence matches, and retries on later 1 ms task ticks. It keeps
USB host presence separate from the selected HID route: power-up is wired
standby, a newly configured host selects wired mode, and `Fn+F8`/`Fn+F7`
explicitly select Bluetooth/wired mode. Removing USB does not select Bluetooth.

The SN32F268F QMK build must retain `USE_PROCESS_STACKSIZE = 0x190` (400
bytes) together with the current static-RAM budget. A larger stack does not fit
the 2 KB SRAM image without reclaiming other buffers; check the final ELF map
after every bridge or matrix change. The paired QMK tree keeps the transport
queues bounded instead of taking RAM from this stack.

Report Protocol carries the full 240-bit QMK NKRO bitmap. BLE Boot Protocol is
kept for BIOS-style compatibility and converts the current state to a standard
6KRO boot report. Consumer/media and System Control reports use separate BLE
HID report IDs. The nRF initiates ATT MTU exchange because the NKRO input value
is 31 bytes.

Synthetic QMK taps, including VIA text macros, hold each key for 15 ms. The
first SPI frame is sent as soon as each key state is generated, and the tap
delay services the pending ACK once per millisecond. After release, QMK waits
only while the press or release report is still awaiting its matching ACK,
with a 32 ms fault timeout. The nRF also serializes HID notifications using the
single report thread and waits for each ATT transmit completion before dequeuing
the next absolute key state. A missing completion is never retried as the same
key press: after one second the link is disconnected so the host releases its
HID state and the bridge can reconnect cleanly. Transient ATT resource errors
are retried only before a notification is accepted and use the same bounded
disconnect recovery after one second. Together these prevent a blocked VIA
macro from collapsing a release and the next key press into one state.

## VIA remapping for BLE controls

If you want BLE control keys to remain editable in VIA, map them to VIA's fixed
`USERxx` keycodes on the QMK side instead of only relying on keymap-local custom
codes. In the paired GMMK TKL bridge keymap, the mapping is:

- `USER00`: switch to BLE profile 1
- `USER01`: switch to BLE profile 2
- `USER02`: switch to BLE profile 3
- `USER03`: select wired USB (`Fn+F7` in the default keymap)
- `USER04`: select Bluetooth (`Fn+F8` in the default keymap)
- `USER05`: disconnect current BLE host
- `USER06`: hold for 1.5 seconds to clear the active profile bond
- `USER07`: hold for 1.5 seconds to clear all bonds
- `USER09`: latched local LED display test. In the default keymap, press
  `Fn+Up` repeatedly to select Num, Caps, Scroll, all three, then normal host
  control again. The selected test pattern is not overwritten by SPI ACKs.

`USER08` is intentionally ignored for compatibility with old VIA EEPROM
mappings. Transport selection never toggles SPI buffer `/OE`; that level
shifter remains enabled for status and control traffic in both modes.

Selecting an empty profile opens pairing for 120 seconds. If it remains empty,
advertising stops when the window expires; selecting that profile again reopens
the pairing window. Bonded profiles advertise through their own BLE identity.

## SPI test from Orange Pi (master)

If you want to verify SPI wiring before integrating the keyboard MCU, you can use a Linux SBC
like Orange Pi 3B as the SPI master to send test HID reports.

This repo includes a small script:

```sh
python3 tools/spi_send.py --bus 3 --cs 0 --key a --mods lshift --repeat 5
```

This sends validated 48-byte bridge frames over SPI (`mode 0`, `8-bit`, `MSB
first`) and confirms both the press and release ACKs.

Transport mode commands can be tested directly with:

```sh
python3 tools/spi_send.py --ctrl 0xF7 --repeat 1
python3 tools/spi_send.py --ctrl 0xF8 --repeat 1
```

Notes:
- Ensure the Linux device node exists (e.g. `/dev/spidev3.0` or `/dev/spidev3.1`).
- Match SPI pins and CS to your wiring. The nRF side must be `nordic,nrf-spis`.
- Install Python dependency if needed: `pip3 install spidev`.

## Linux lock LED helper

If your Linux desktop updates Caps/Num/Scroll state internally but does not
forward the HID output report back to the BLE keyboard, you can mirror that
state manually through `hidraw`:

```sh
python3 tools/linux_led_sync.py --list
sudo python3 tools/linux_led_sync.py --hidraw /dev/hidraw2 --bits caps
sudo python3 tools/linux_led_sync.py --hidraw /dev/hidraw2 --watch --verbose
```

Notes:
- The keyboard output report uses report ID `1`. The helper therefore writes
  `01 <led_bits>` by default.
- `--bits caps` is the quickest way to verify that the Linux-to-keyboard write
  path works at all before testing automatic sync.
- `--watch` polls the related `inputX::capslock` / `numlock` / `scrolllock`
  sysfs nodes and pushes changes back into the BLE keyboard's `hidraw` device.
- If auto-detection picks the wrong input node, run `--list` and then pass
  `--input inputX` explicitly.
- Use `--report-id 0 --no-prefix-zero-report-id` only when testing an older
  unnumbered descriptor.

## STM32 master quick debug

When testing with STM32 (Blue Pill) before integrating QMK:

- Keep mode `0`, `8-bit`, `MSB first`, and send exactly 48 bytes per CS-low transaction.
- Read back each `SPI.transfer()` return byte on STM32.
- nRF SPIS now returns an ACK frame on MISO:
- `A5 5A <count_lo> <count_hi> <kbd_leds> <flags> <ctrl_seq> <ctrl_cmd>`
  - `count_*` increments every received frame.
- `kbd_leds` is the 3-bit indicator value that QMK should apply to
  `LED_NUM_LOCK_PIN` / `LED_CAPS_LOCK_PIN` / `LED_SCROLL_LOCK_PIN`.
  When the BLE host has already sent a keyboard output report, it carries the
  real Num Lock / Caps Lock / Scroll Lock state. Otherwise it uses a minimal
  production-style bridge indication:
  - active profile LED blinks quickly while pairing
  - after switching to a bonded profile, the active profile LED stays solid for 2 seconds
  - after a BLE link comes up, the active profile LED stays solid for 3 seconds
  - after the 3-second connected hint window, only the real host lock LED state is shown
  - if there is no host output report yet, the indicators stay off instead of guessing
- `flags bit0` is `1` whenever `kbd_leds` is valid.
- `flags bit1` is `1` when there is an active BLE connection.
- `flags bit2` is `1` while pairing mode is active.
- `flags bit3` is `1` when the selected profile slot already has stored bond data.
- `flags bit4` is `1` in wireless mode and `0` in wired mode.
- Bits 7..5 of `flags` stay `0b101`, allowing the QMK master to reject invalid ACKs.
- `ctrl_seq` and `ctrl_cmd` echo the last validated control frame. For a valid
  input report, `ctrl_seq` echoes the frame sequence and `ctrl_cmd` is `0x00`.
  QMK clears a pending command only after both fields match; mode
  changes also require bit 4 to report the requested state.
- If you always read `FF`/`00`, usually wiring is wrong (`MOSI/MISO/CS/GND`) or CS is not on the configured pin.

Also note: this sample uses RTT console in `prj.conf` (`CONFIG_RTT_CONSOLE=y`, `CONFIG_UART_CONSOLE=n`), so normal UART serial monitor will not show logs.
