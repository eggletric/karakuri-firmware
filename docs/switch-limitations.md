# Switch-side limitations

This firmware makes a Pico W / Pico 2 W look like a **USB HID gamepad for the
Nintendo Switch** (see `USBDevice.setID(0x0f0d, 0x0092)` = HORI Pokken Tournament
Pro Pad inside `NSGamepad::begin()` in
`libraries/switch_tinyusb/src/switch_tinyusb.h`).

That means it **only works within what the Switch itself allows a generic
third-party HID pad to do.** This file records the things that "no firmware fix
will solve" or that "would require a structural change to solve".

---

## The HOME button does not work

### The firmware sends it correctly

- `enum NSButtons` in `switch_tinyusb.h` defines `NSButton_Home = 12`
- The HID report descriptor declares `Report Count (14)` = button bits 0..13, so bit 12 is in range
- `BTN HOME DOWN` → `Gamepad.press(NSButton_Home)` → `_report.buttons |= 1 << 12` is set and sent

So this is **not a defect in the code**. When it does not work, the cause is one of the following.

### Cause (a) the press is too short

The Switch is known to require a longer press for HOME than for other buttons.
Macros default to a 100ms interval, so if the line after `BTN HOME DOWN` is
immediately `BTN HOME UP`, the button is only held for 100ms.

**Try this first. No firmware change needed.**

```
BTN HOME DOWN
SLEEP 500
BTN HOME UP
```

If pressing it with your finger from KarakuriPad's ControllerPad does not work either,
press duration is not the cause — go to (b).

### Cause (b) the Switch drops HOME based on VID:PID

The HORI Pokken Tournament Pro Pad at `0x0f0d:0x0092` **has no physical HOME button.**

The Switch treats third-party wired controllers as known devices by VID:PID, so it
does not necessarily trust the "14 buttons" descriptor we declare. Once it has
recognized us as a Pokken Pad, it most likely discards bit 12 on the grounds that
"this controller has no HOME".

The fix to try is replacing `setID` in `switch_tinyusb.h` with the VID:PID of a
Switch-compatible wired controller that does have HOME. **This needs no hardware
work — only a firmware rebuild.** Expect to try several IDs on real hardware to
find one that gets through.

### Diagnostic steps

1. Try the `BTN HOME DOWN` / `SLEEP 500` / `BTN HOME UP` macro
2. If that fails, try a long press with your finger from the ControllerPad
3. If neither works, (b) is confirmed. Try swapping `setID`

---

## Reference: what this firmware does and does not do

| | Supported | Notes |
|---|---|---|
| Buttons, sticks, D-Pad | Yes | See the command list in `handleCommand()` |
| Macro loop playback | Yes | Loops forever until `MACRO STOP`. Designed for unattended grinding |
| HOME button | Partial | It is sent. Whether the console accepts it is covered above |
| CAPTURE button | Partial | May have the same problem for the same reason as HOME |
| Rumble (HD rumble) | No | Output reports are not handled |
| Gyro / IMU | No | The descriptor declares no axes |
