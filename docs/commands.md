# Command reference

The firmware accepts the same commands over three transports.

- **Bluetooth (BLE)** — the default. A Nordic UART Service (NUS)-compatible GATT
  service: writes to the RX characteristic (`6E400002-...`) are commands, and
  notifications on the TX characteristic (`6E400003-...`) are responses. No pairing required
- **Wi-Fi (TCP)** — on the port configured in `link.cfg`. Only one connection at a time;
  a new connection drops the old one
- **USB serial** — 115200bps. Any line that is not a configuration command is handled
  as a gamepad / macro command

BLE and Wi-Fi are mutually exclusive: only the one selected by `mode` in `link.cfg`
starts up (BLE by default).

There is also a **dongle mode** (`mode=dongle`). The Pico acts as a BLE central and
connects directly to a Pro Controller 2 (Switch 2 Pro Controller), streaming its input
to a PC / Mac as a USB gamepad. It sends the official initialization sequence on
connect to enable full reports, so in addition to buttons and sticks you also get
**gyro, accelerometer, the C/GL/GR buttons and rumble**.

The USB identity is selected with `usbmode`:

- **sinput** (default) — an open standard supported natively by SDL3 (VID:PID 2E8A:10C6).
  C passes through as a misc button and GL/GR as paddles
- **ds4** — DualShock 4 (054C:05C4) emulation. The identity to use if you want gyro plus
  rumble working in Steam today. C/GL/GR do not exist on a DS4, so `ds4map` chooses what
  they map to (default: C = touchpad click). CAPTURE maps to touchpad click, HOME to PS,
  MINUS to SHARE and PLUS to OPTIONS. For the IMU calibration SDL requests
  (feature 0x02) it returns stock-DS4 scales (16.384 LSB/dps, 8192 LSB/g) and converts
  the raw values into those units
- **switch** — the same HORI-compatible Switch pad identity the bt/wifi modes use.
  Plug the dongle into a **Switch / Switch 2** and the Pro Controller 2 relays into it
  as a licensed wired pad. The identity has no IMU and no rumble; C/GL/GR do not exist
  on it, so `switchmap` chooses what they map to (default none for all three). The
  macro recorder's feedback vibration still works — it goes to the controller over
  BLE, not to the USB host
- **procon** — Pro Controller emulation (057E:2009, the console's own proprietary USB
  protocol). The identity to use on a **Switch / Switch 2** when you also want
  **gyro and rumble**: IMU passes through (relay only — never recorded or replayed),
  and the console's HD rumble is decoded and relayed to the Pro Controller 2 with
  its amplitude preserved proportionally. C/GL/GR follow `switchmap`, same as usbmode=switch. On a Switch 1
  the console setting **"Pro Controller Wired Communication" must be ON**

**Only the Pro Controller 2 is supported** (the original Pro Controller from the
Switch 1 generation and Joy-Cons are not).
Dongle mode has no link to the app; configuration is over USB serial only. Put the
Pro Controller 2 into pairing mode with its sync button and it connects automatically;
on disconnect the firmware sends a neutral state and rescans.
The **USB controller stays present on the host across a Pro Controller 2 disconnect**:
the report stream keeps running (as a neutral controller) through the rescan and the
reconnect handshake, so a console does not drop the pad and the controller is live
again the moment the Pro Controller 2 comes back. Unplug the USB cable to make it go
away.
A change to `usbmode` takes effect via the automatic reboot on `RESET`.
Logs appear on serial under the `[DONGLE]` tag.

One command per line, newline-separated. Commands are case-insensitive.

---

## Gamepad input

### Buttons

```
BTN <NAME> DOWN     press
BTN <NAME> UP       release
BTN ALL UP          release all buttons + D-Pad neutral + sticks centered
TAP <NAME> <ms>     press, then release after <ms> (1-60000)
```

Valid values for `<NAME>`:

```
A  B  X  Y  L  R  ZL  ZR  PLUS  MINUS  HOME  CAPTURE  LSTICK  RSTICK
```

`TAP` releases asynchronously, so it does not block. Inside a macro it waits for the
press duration on top of the step interval before moving on. Because it replaces the
three steps `DOWN` / `SLEEP` / `UP` with one, it also saves room in the 1800-step budget.

### D-Pad

```
DPAD CENTER
DPAD UP / DOWN / LEFT / RIGHT
DPAD UPLEFT / UPRIGHT / DOWNLEFT / DOWNRIGHT
DTAP <DIR> <ms>     hold for the given ms, then return to neutral automatically (1-60000)
```

The D-Pad is a **state set** on a hat switch, so `DPAD UP` stays held until you send
`DPAD CENTER`. Use `DTAP` for a quick tap.
If the hold is too short the game will not sample it (typically 60fps, i.e. a ~16.7ms
period), so specify at least two or three frames (50ms and up).

Sending a `DPAD` command by hand while a `DTAP` release is pending discards that
pending release. Inside a macro, `DTAP` consumes "step interval + hold time" just like `TAP`.

### Sticks

```
LSTICK CENTER / UP / DOWN / LEFT / RIGHT
LSTICK <x> <y>          0-255 (128 is center)
RSTICK ...              same as above
```

### Other

```
VERSION                 returns the firmware version
```

---

## Macros

### Loading and playback

```
MACRO LOAD <interval_ms>    begin loading. Sets the step interval (minimum 10ms)
<step>                      one step per line
<step>
...
MACRO END                   finish loading
MACRO START                 start playback
MACRO STOP                  stop and discard the macro
```

- `MACRO LOAD` is **accepted even during playback** (it discards and replaces the current macro)
- All other commands are ignored during playback (except `MACRO STOP`)
- Playback **loops** until `MACRO STOP`

### What a step can contain

Every gamepad command above, plus one that is only valid inside a macro:

```
SLEEP <ms>              wait before the next step (0-600000)
```

`SLEEP` does not consume the step interval; it waits exactly the given number of ms.
Every other step consumes the interval given to `MACRO LOAD`
(`TAP` alone consumes "interval + press duration").

### Limits

| | Value |
|---|---|
| Maximum steps | 1800 |
| Maximum length of one step | 24 characters |
| Minimum step interval | 10ms |

Steps beyond the limit are **discarded rather than truncated**, and the counts are
returned in the response to `MACRO END`.

String macros are not available in dongle mode (their buffer is reused by the dongle
macro recorder below, and playback needs the Switch HID identity anyway).

---

## Dongle macro recorder (mode=dongle, macro=on)

Records the Pro Controller 2's own input (buttons and sticks, sampled at 32Hz for up
to 90 seconds) and replays it into the active USB identity — any of the three,
including `usbmode=switch`, so recordings can be replayed into a Switch / Switch 2.
Up to 8 recordings are kept on flash. Everything is driven from the controller itself:

| Gesture | Effect |
|---|---|
| C + slot button (short press) | loop-play that slot |
| C + GL or C + GR held for 1s | vibration = armed; recording starts (with a vibration) 0.5s after everything is released |
| while recording: C, or 90s elapsed | vibration = recording ends, host output goes neutral |
| then: slot button | save to that slot (double vibration), resume relaying |
| then: C | discard (one long vibration), resume relaying |
| while playing: C | stop playback |
| C + GL + GR held for 1s | a separate, longer vibration = GL/GR assignment mode (see below) |

Slot buttons: **A, B, X, Y, L, R, ZL, ZR** (8 slots). GL/GR are not slots — they are
the record trigger — and keep passing through as normal buttons.

Details:

- **C never reaches the host while macro=on** (it is the control button), and neither
  do **GL/GR while C is held** (every C+paddle combination is a control gesture, so a
  held paddle must not fire in-game during the 1s hold). The C assignment in
  `ds4map` / `switchmap` is effectively inert while macro=on. On ds4 the touchpad
  click is still available through CAPTURE
- For **slot playback** C must be pressed first: only a slot button pressed while C is
  already held counts, so pressing C while holding e.g. ZR in-game cannot hijack into
  playback. The **record trigger is order-independent** (the back paddles are naturally
  gripped before C): the 1s hold starts once C and GL/GR are both down, in any order
- During recording the controller keeps relaying to the host as usual, with two
  exceptions: the **IMU is muted** (gyro is relay-only and never recorded, so a take
  must not rely on input that playback cannot reproduce) and **game rumble is
  discarded** (the recorder's vibration is the only feedback channel). Recording is
  RAM-only; flash is written only after recording ends (while the host sees neutral),
  so relaying is never stalled by a flash write
- While playing, live input is **not** forwarded (C = stop is the only control),
  and game rumble is discarded (nobody is holding the controller the way the game
  assumes). Replayed input reports zero gyro/accel, i.e. a controller at rest
- Saving overwrites the slot silently. Playing an empty slot answers with a triple
  vibration. Controller or USB disconnect discards a recording in progress and stops
  playback
- The `DMACRO` serial command prints the recorder state and slot occupancy

---

## GL/GR button assignment

The two back paddles can be assigned any combination of buttons, so a single paddle
can fire several at once (for example GL = A + UP). The assignment is one
**mode-independent** value per paddle: the same setting applies to every `usbmode`,
because it is stored as a controller-level button mask and converted by each USB
identity with its own existing logic.

Set it two ways — both write the same value, so the last write wins:

- **From the app / serial**: `glmap=` / `grmap=` (see Configuration below)
- **From the controller**, with `macro=on` (the gesture needs C as a control button):

| Step | Action | Feedback |
|---|---|---|
| 1 | Hold **C + GL + GR** for 1s | one **long** vibration (clearly longer than any recorder buzz) |
| 2 | Release everything | selection mode starts 0.5s later; the host sees neutral from here on |
| 3 | Hold the button(s) to assign for **3s** without changing them | double vibration = captured |
| 4 | Release everything, then press **GL** or **GR** | vibration = saved to that paddle |

- Assignable buttons, by gesture and from the app alike: **dpad up/down/left/right,
  A, B, X, Y, L, R, ZL, ZR, Lstick-click, Rstick-click** (14). C, GL and GR are control
  buttons, and the system buttons (`+`, `-`, HOME, CAPTURE) are deliberately left out so
  a paddle can never fire something that leaves the game. They are ignored during
  selection and rejected by `glmap=` / `grmap=`
- Releasing a button or adding another **restarts the 3s count**. Releasing everything
  just stops the count — selection mode stays active
- **C cancels** at any point during the gesture (one long vibration, nothing saved).
  This is why C itself can never be assigned
- **To clear an assignment**: in selection mode, with nothing held, press GL or GR
  directly. That paddle goes back to its default behavior. Selection only begins
  0.5s after a clean full release (pressing anything restarts that wait), so a
  paddle still bouncing from the trigger grip cannot clear an assignment by accident
- 15s without any button activity aborts the gesture (triple vibration)
- The result is saved to flash immediately — no `RESET` or reboot needed

Precedence, per paddle independently:

- **assignment set** → it replaces the paddle entirely: the `ds4map`/`switchmap` token
  for that paddle and the native sinput paddle bit are both overridden
- **assignment not set** (the default) → unchanged legacy behavior: sinput forwards the
  native paddle bit, ds4/switch/procon apply their `ds4map`/`switchmap` token

Assignments apply **regardless of `macro`** — only the on-controller gesture for
*setting* them needs `macro=on`. Macro playback goes through the same conversion, so a
recording that contains paddle presses follows whatever assignment is current at
playback time. `DMACRO` prints the current assignments.

---

## Configuration (USB serial only)

```
CFG GET                 print the current configuration
CFG BEGIN               begin receiving configuration
mode=<bt|wifi|dongle>   operating mode (omitted or unknown values mean bt)
btname=<NAME>           BLE device name (empty auto-generates Karakuri-XXXX)
usbmode=<sinput|ds4|switch|procon>  USB identity in dongle mode (default sinput)
ds4map=<c>,<gl>,<gr>    C/GL/GR assignments in ds4 mode. Tokens:
                        none/touchpad/ps/share/options/l1/r1/l2/r2/l3/r3/
                        cross/circle/square/triangle (default touchpad,none,none)
                        The GL/GR halves are overridden when glmap/grmap is set
switchmap=<c>,<gl>,<gr> C/GL/GR assignments in switch and procon modes. Tokens:
                        none/a/b/x/y/l/r/zl/zr/plus/minus/home/capture/
                        lstick/rstick (default none,none,none)
                        The GL/GR halves are overridden when glmap/grmap is set
glmap=<tokens>          GL assignment, applied in every usbmode. Combine with '+'
grmap=<tokens>          GR assignment, same syntax. Tokens:
                        none/up/down/left/right/a/b/x/y/l/r/zl/zr/
                        lstick/rstick (default none)
                        e.g. glmap=a+up   grmap=none   (none clears it)
                        An unknown token rejects the whole line, keeping the
                        previous value (a typo cannot save a half-built combo)
macro=<on|off>          dongle macro recorder (default off; mode=dongle only)
ssid=<SSID>             the Wi-Fi keys below are required only when mode=wifi
pass=<PASSWORD>         leading and trailing spaces are NOT trimmed
ip=<IPv4>
port=<1-65535>
gw=<IPv4>
sn=<IPv4>
CFG END                 validate and save to /link.cfg

RESET                   reload link.cfg and apply it
DMACRO                  dongle macro recorder status (state, slot occupancy,
                        and the current glmap/grmap assignments)
FS INFO                 LittleFS status and whether link.cfg exists
FS TEST                 read/write test
FS FORMAT               format LittleFS (this erases the configuration too)
```

- `CFG GET` prints **only the keys relevant to the active mode** (`btname` for bt,
  the Wi-Fi keys for wifi; settings for the unused side are not shown)
- `CFG BEGIN` starts from the current configuration, so sending only some keys
  preserves the rest (for example, the Wi-Fi settings survive when you only change the BT name)
- `RESET` **reboots automatically** when the mode (`mode`) or the BLE name differs from
  what is currently running, because the BLE stack cannot be rebuilt on the fly.
  Changing only the Wi-Fi settings while staying on mode=wifi reconnects without a reboot, as before

Saved settings take effect on reboot or on `RESET`.

### Implementation note (radio)

Always read radio state through the official `WiFi.status()`.
Calling `cyw43_tcpip_link_status()` directly without the lock has been observed on real
hardware to leave the driver's connect (join) sequence unable to complete.
Avoid direct calls into the `cyw43_arch_*` APIs for the same reason.

---

## Responses from the firmware

Sent back to the connected app (a TCP client, or a BLE notification):

```
MACRO LOADED <accepted> <discarded>   response to MACRO END
ERR unknown command: <line>           a line that could not be parsed
<version>                             response to VERSION
```

During macro loop playback, `ERR` is only returned **on the first pass**, to keep the
link from being flooded.

Tagged logs are printed to serial.

| Tag | Meaning |
|---|---|
| `[BLE]` | BLE advertising and app connect / disconnect |
| `[WLAN]` | Wireless connection to the Wi-Fi router (connect, disconnect, reconnect) |
| `[TCP]` | TCP connection to the app (listen, app connected/disconnected) |
| `[CFG]` | Reading and writing configuration |
| `[MACRO]` | Macro loading and playback |
| `[FS]` | LittleFS |
| `[DONGLE]` | Dongle mode (Pro Controller 2 connect / disconnect, USB identity) |
| `[DMACRO]` | Dongle macro recorder (arm, record, save, playback) |
| `[DMAP]` | The on-controller GL/GR assignment gesture |

While a macro is loading, a log line is printed only every 200 steps
(printing one per line would occupy the serial port for a long time).
