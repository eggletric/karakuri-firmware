# Karakuri Firmware

Firmware that turns a Raspberry Pi Pico W / Pico 2 W into a controller for the
Nintendo Switch, letting you stream button input and macros to it over
Bluetooth (BLE) or Wi-Fi. It also has a **dongle mode** that converts a
Pro Controller 2 into a USB gamepad for PC / Mac.

The companion app is [KarakuriPad](https://github.com/eggletric/karakuri-pad),
which sends the input and macros this firmware plays back.

- **Bluetooth** — no router required. KarakuriPad connects directly over BLE.
  The board advertises as `Karakuri-XXXX` unless `btname` is set
  (XXXX is a per-device fixed value)
- **Wi-Fi (TCP)** — the traditional path, through a router. Switch between them
  with `mode` in `link.cfg`
- **Dongle** (`mode=dongle`) — the Pico connects directly to a Pro Controller 2
  (Switch 2 Pro Controller) over BLE and relays its input over USB under a
  selectable identity (`usbmode`): an SInput (SDL3) or DualShock 4 gamepad for a
  PC / Mac, a HORI-compatible licensed pad or a Pro Controller emulation (gyro +
  rumble) for a Switch / Switch 2. Gyro, rumble and C/GL/GR all pass through
  where the identity supports them (the original Pro Controller is not supported
  as the BLE source). See [`docs/commands.md`](docs/commands.md) for details

The three modes are mutually exclusive — only the one configured in `link.cfg`
starts up, because they share the same radio chip.

**First boot:** a board with no `link.cfg` writes a factory default one
(`mode=dongle`, `usbmode=procon`, `macro=on`, GL/GR unassigned) and reboots into
it, so a freshly flashed Pico works as a Pro Controller dongle with no setup.
Change the mode over USB serial (`CFG BEGIN` … `CFG END`, see
[`docs/commands.md`](docs/commands.md)) to use BLE or Wi-Fi instead.

## Supported boards

| Board | Chip | UF2 |
|---|---|---|
| Raspberry Pi Pico W | RP2040 | `karakuri-firmware-picow.uf2` |
| Raspberry Pi Pico 2 W | RP2350 | `karakuri-firmware-pico2w.uf2` |

A UF2 header carries a family ID for the chip generation, so **a single file
cannot cover both boards.** Flash the one that matches your board (the app
detects the connected board and picks it automatically).

See [`docs/boards.md`](docs/boards.md) for details.

## How releases work

- **Just push to the `main` branch.** CI builds for both boards and publishes a release.

## URLs that always give you the latest build

For this repository:

```
# Pico W (RP2040)
https://github.com/eggletric/karakuri-firmware/releases/latest/download/karakuri-firmware-picow.uf2

# Pico 2 W (RP2350)
https://github.com/eggletric/karakuri-firmware/releases/latest/download/karakuri-firmware-pico2w.uf2
```

These URLs always return the UF2 attached to the latest official release.

## Repository layout

- `firmware/pico_switch_pad/`
  - The firmware itself (shared by Pico W and Pico 2 W)
  - `pico_switch_pad.ino` — entry point (mode startup, command handling, macros)
  - `ble_link.h` — BLE link to the app (NUS-compatible GATT)
  - `procon2_dongle.h` — Pro Controller 2 (BLE) connection for dongle mode
  - `sinput_usb.h` / `ds4_usb.h` / `procon_usb.h` — USB identities for dongle mode
    (SInput / DualShock 4 / Pro Controller; the fourth identity, the HORI-compatible
    Switch pad, is the shared gamepad the bt/wifi modes use)
- `.github/workflows/build-auto-release.yml`
  - GitHub Actions definition that builds, tags and releases automatically on a push to `main`
- `docs/commands.md`
  - Command reference (gamepad input, macros, configuration)
- `docs/boards.md`
  - Supported boards and how to identify them in BOOTSEL mode
- `docs/switch-limitations.md`
  - Notes on Switch-side limitations (waking the console from sleep and other things a firmware fix cannot solve)

## Acknowledgements

This firmware builds on the work of the controller reverse-engineering community:

- [dekuNukem/Nintendo_Switch_Reverse_Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering) (MIT) —
  the Switch controller protocol documentation the Pro Controller emulation follows
  (USB handshake, subcommands, SPI layout, HD rumble encoding)
- [TheFrano/joycon2cpp](https://github.com/TheFrano/joycon2cpp) (MIT) —
  the Pro Controller 2 BLE initialization sequence
- [ndeadly/switch2_controller_research](https://github.com/ndeadly/switch2_controller_research) —
  Switch 2 controller research; the Pro Controller 2 rumble sample format was
  derived from its console captures
- [touchgadget/switch_tinyusb](https://github.com/touchgadget/switch_tinyusb) (MIT, bundled in `libraries/`) —
  the HORI-compatible Switch gamepad identity
- [Adafruit TinyUSB](https://github.com/adafruit/Adafruit_TinyUSB_Arduino) (MIT) and
  [arduino-pico](https://github.com/earlephilhower/arduino-pico) (LGPL-2.1) —
  the USB stack and the RP2040 Arduino core this firmware is built with

## License

MIT

## Author

Developed by [Eggletric](https://github.com/eggletric) — the team behind
[DiffyPick](https://diffy-pick.com/), a visual diff & pick tool for database schemas.
