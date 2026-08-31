# Switch-side limitations

This firmware makes a Pico W / Pico 2 W look like a **USB HID gamepad for the
Nintendo Switch** (see `USBDevice.setID(0x0f0d, 0x0092)` = HORI Pokken Tournament
Pro Pad inside `NSGamepad::begin()` in
`libraries/switch_tinyusb/src/switch_tinyusb.h`).

That means it **only works within what the Switch itself allows a generic
third-party HID pad to do.** This file records the things that "no firmware fix
will solve" or that "would require a structural change to solve".

---

## The HOME button works, but it cannot wake a sleeping Switch

The HOME button itself is fine. Once the console is awake and the pad is
recognized, `BTN HOME DOWN` / `BTN HOME UP` opens the HOME menu as expected,
whether it comes from a macro or from KarakuriPad's ControllerPad.

What does **not** work is **waking the console from sleep.** A sleeping Switch
only wakes for controllers it treats as first-party wireless pads; it does not
wake because a third-party USB HID device was plugged in or because that device
started reporting a HOME press. This is a console-side rule, not something the
firmware can send its way around.

### Workaround: wake it with the real controller first

Wake the Switch the normal way, then hand control over:

1. Wake the Switch with a Pro Controller connected to it directly (its own HOME button)
2. Connect the dongle, or connect KarakuriPad and send HOME from the app
3. From here on HOME behaves normally

There is no firmware change to make here — the split is deliberate: **use the
normal connection to wake, use this firmware for everything after that.**

### If HOME does not respond while the console is awake

The likely cause is press duration. The Switch wants a longer hold for HOME
than for other buttons, and macros default to a 100ms interval, so
`BTN HOME DOWN` immediately followed by `BTN HOME UP` only holds it for 100ms.

```
BTN HOME DOWN
SLEEP 500
BTN HOME UP
```

---

## Reference: what this firmware does and does not do

| | Supported | Notes |
|---|---|---|
| Buttons, sticks, D-Pad | Yes | See the command list in `handleCommand()` |
| Macro loop playback | Yes | Loops forever until `MACRO STOP`. Designed for unattended grinding |
| HOME button | Yes | Works while the console is awake. Hold it long enough — see above |
| CAPTURE button | Yes | Sent the same way as HOME |
| Waking the Switch from sleep | No | Console-side rule. Wake it with a directly connected controller first |
| Rumble (HD rumble) | No | Output reports are not handled |
| Gyro / IMU | No | The descriptor declares no axes |
