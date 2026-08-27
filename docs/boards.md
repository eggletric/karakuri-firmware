# Supported boards and BOOTSEL identification

## Supported boards

| Board | Chip | SRAM | Flash | Radio |
|---|---|---|---|---|
| Raspberry Pi Pico W | RP2040 (Cortex-M0+) | 264KB | 2MB | CYW43439 |
| Raspberry Pi Pico 2 W | RP2350 (Cortex-M33) | 520KB | 4MB | CYW43439 |

**Both boards use the same CYW43439 radio chip**, so there is no difference in the
Wi-Fi code. `macroLines` (a static 1800 × 25 = 45KB array) also fits within the
RP2040's 264KB, which leaves the RP2350 with even more headroom.

**The firmware source is shared.** Only the build configuration differs.

## Build configuration

`.github/workflows/build-auto-release.yml` builds both.

| | Pico W | Pico 2 W |
|---|---|---|
| FQBN | `rp2040:rp2040:rpipicow` | `rp2040:rp2040:rpipico2w` |
| flash | `2097152_65536` (2MB / FS 64KB) | `4194304_65536` (4MB / FS 64KB) |
| usbstack | `tinyusb` | `tinyusb` |
| arch | (unspecified) | `arm` |

- The platform ID stays `rp2040:rp2040` even for the RP2350 (a historical quirk of arduino-pico)
- The Pico 2 W lets you pick ARM or RISC-V via `menu.arch`. We specify ARM explicitly
- The matching UF2 family is `rp2350-arm-s`

## A single UF2 cannot cover both boards

A UF2 header carries a family ID, and **the bootloader rejects any UF2 that does not match its own.**

| Chip | family ID |
|---|---|
| RP2040 | `0xe48bff56` |
| RP2350 (ARM Secure) | `0xe48bff59` |
| RP2350 (RISC-V) | `0xe48bff5a` |
| RP2350 (ARM Non-secure) | `0xe48bff5b` |

One file therefore cannot serve both, and each release ships two of them.

Flashing the wrong one does no damage, but it fails in a confusing way: **the copy to
the drive succeeds while the firmware is never replaced.** KarakuriPad
validates the UF2 header's family ID before flashing and rejects a mismatch.

## Identification in BOOTSEL mode (measured)

Values observed over a BOOTSEL connection on macOS.

| Item | Pico W (RP2040) | Pico 2 W (RP2350) |
|---|---|---|
| Volume label | `RPI-RP2` | `RP2350` |
| USB VID | `0x2e8a` | `0x2e8a` (same) |
| USB PID | `0x0003` | **`0x000f`** |
| USB Product | — | `RP2350 Boot` |
| `Model` in `INFO_UF2.TXT` | — | `Raspberry Pi RP2350` |
| `Board-ID` in `INFO_UF2.TXT` | `RPI-RP2` | `RP2350` |
| Partition | FAT | FAT16 / 134.2MB |

The app finds the BOOTSEL drive by volume label and USB PID, then determines the
chip generation from `Board-ID` in `INFO_UF2.TXT`.

### Caveat: BOOTSEL mode cannot tell you whether the board is a "W"

`Board-ID` only indicates the chip generation.

- Pico and Pico W → both report `RPI-RP2`
- Pico 2 and Pico 2 W → both report `RP2350`

So auto-detection can only tell you **RP2040 vs RP2350**. Flashing a radio-less
Pico / Pico 2 still results in the UF2 being accepted (in that case Wi-Fi simply
does not work, while USB HID still does).
