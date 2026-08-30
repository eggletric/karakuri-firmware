# Spec: unified GL/GR button assignment (app + on-controller gesture)

Status: **firmware implemented** (see `pico_switch_pad.ino`; user-facing behavior is
documented in `commands.md`). **The app side of section 9 is still open**, and the
hardware checks in section 11 have not been run yet.
This document is kept as the design rationale and the verification checklist.

---

## 1. Goal

Replace the per-usbmode GL/GR token assignments with **one mode-independent
assignment per paddle**, stored as a `P2_BTN_*` bitmask, that can be set two ways:

1. **From the app / serial** via new `glmap=` / `grmap=` CFG keys (full vocabulary,
   multi-button combos allowed).
2. **From the controller itself** via a new C+GL+GR gesture (subset vocabulary,
   multi-button combos allowed), available while `mode=dongle` and `macro=on`.

Both write the same stored value, so there is no synchronization problem — last
write wins. A mask assignment supports **simultaneous multi-button output**
(e.g. GL = A+UP), which the old token system could not express.

The C button assignment (`ds4MapC` / `swMapC`) is **out of scope and unchanged**.

### Why a P2_BTN mask covers everything the old tokens did

The four USB identity converters (`buildDS4FromProcon2`, `buildSInputFromProcon2`,
`dongleApplySwitchReport`, `buildProconFromProcon2`) all derive their output from
the raw `Procon2Report::buttons` mask. Injecting bits into that mask *before*
conversion therefore reuses all existing per-identity logic for free: DS4 gets its
hat from `ds4HatFromButtons`, `P2_BTN_ZL` sets `l2=255` analog, procon gets its
dpad bits, etc. Every old token has a P2_BTN equivalent through the converters:

| old ds4map token | P2_BTN equivalent | old switchmap token | P2_BTN equivalent |
|---|---|---|---|
| touchpad | *(dropped, see below)* | capture | *(dropped)* |
| ps | *(dropped)* | home | *(dropped)* |
| share | *(dropped)* | minus | *(dropped)* |
| options | *(dropped)* | plus | *(dropped)* |
| l1 / r1 | `P2_BTN_L` / `P2_BTN_R` | l / r | `P2_BTN_L` / `P2_BTN_R` |
| l2 / r2 | `P2_BTN_ZL` / `P2_BTN_ZR` | zl / zr | `P2_BTN_ZL` / `P2_BTN_ZR` |
| l3 / r3 | `P2_BTN_LSTICK` / `P2_BTN_RSTICK` | lstick / rstick | `P2_BTN_LSTICK` / `P2_BTN_RSTICK` |
| cross / circle / square / triangle | `P2_BTN_B` / `P2_BTN_A` / `P2_BTN_Y` / `P2_BTN_X` | a / b / x / y | `P2_BTN_A` etc. |

---

## 2. Data model

Add to `LinkConfig`:

```cpp
uint32_t glMask;   // P2_BTN_* bitmask GL is remapped to. 0 = no remap (legacy behavior)
uint32_t grMask;   // same for GR
```

Defaults: `0` (in the initializer of `gLinkConfig`, in `loadLinkConfig()`'s
reset-to-defaults block, and implicitly via `incomingCfg = gLinkConfig`).

Two allowed-bit constants (place near the DM_* constants):

**Amended after the first hardware test:** the system buttons (`+`, `-`, HOME,
CAPTURE) were dropped from the vocabulary entirely, so a paddle can never fire
something that leaves the game or opens a console overlay. The gesture set and the
app set are therefore identical, and `DM_ASSIGN_CANDIDATE_MASK` is just an alias of
`GLGR_ASSIGNABLE_MASK`. The cost is that a paddle can no longer be mapped to HOME or
CAPTURE the way the old `ds4map`/`switchmap` tokens allowed.

```cpp
// Buttons assignable from the app (glmap=/grmap=)
const uint32_t GLGR_ASSIGNABLE_MASK =
    P2_BTN_UP | P2_BTN_DOWN | P2_BTN_LEFT | P2_BTN_RIGHT |
    P2_BTN_A | P2_BTN_B | P2_BTN_X | P2_BTN_Y |
    P2_BTN_L | P2_BTN_R | P2_BTN_ZL | P2_BTN_ZR |
    P2_BTN_LSTICK | P2_BTN_RSTICK |
    P2_BTN_MINUS | P2_BTN_PLUS | P2_BTN_HOME | P2_BTN_CAPTURE;

// Buttons selectable in the on-controller gesture (subset: no system buttons)
const uint32_t DM_ASSIGN_CANDIDATE_MASK =
    P2_BTN_UP | P2_BTN_DOWN | P2_BTN_LEFT | P2_BTN_RIGHT |
    P2_BTN_A | P2_BTN_B | P2_BTN_X | P2_BTN_Y |
    P2_BTN_L | P2_BTN_R | P2_BTN_ZL | P2_BTN_ZR |
    P2_BTN_LSTICK | P2_BTN_RSTICK;
```

`P2_BTN_C`, `P2_BTN_GL`, `P2_BTN_GR` are **never** allowed in a stored mask.
Sanitize every value at every entry point (file load, CFG key, gesture) with
`mask &= GLGR_ASSIGNABLE_MASK;`.

### Precedence rule (the "兼ね合い")

Per paddle, independently:

- `glMask != 0` → GL is remapped to `glMask`. The legacy token (`ds4MapGL` /
  `swMapGL`) and the native sinput paddle bit are **overridden** (the GL bit is
  removed from the report, so neither can fire).
- `glMask == 0` → **legacy behavior, completely unchanged**: sinput forwards the
  native paddle bit; ds4/switch/procon apply their `ds4map`/`switchmap` token.

Same for GR with `grMask` / `ds4MapGR` / `swMapGR`.

The legacy token fields stay in the struct, the file format, and the CFG protocol
(old app versions keep working). They simply lose against a non-zero mask.

---

## 3. Where the mask is applied

Apply inside **`dongleApplyReport()`**, at the top, before dispatching to the
identity-specific builders. This is the single choke point through which both the
live relay path and dongle-macro playback flow, so **replayed recordings are also
remapped with the assignment current at playback time** (recordings store raw
buttons; this is intended — a recording survives assignment changes).

```cpp
void dongleApplyReport(const Procon2Report &r) {
  Procon2Report m = r;
  if (gLinkConfig.glMask && (m.buttons & P2_BTN_GL)) {
    m.buttons = (m.buttons & ~P2_BTN_GL) | gLinkConfig.glMask;
  }
  if (gLinkConfig.grMask && (m.buttons & P2_BTN_GR)) {
    m.buttons = (m.buttons & ~P2_BTN_GR) | gLinkConfig.grMask;
  }
  // ... existing dispatch, using m instead of r ...
}
```

Notes:

- The application is **unconditional on `macroOn`** — assignments work with the
  macro recorder off (the gesture UI is macro=on only; the *effect* is not).
- `dmHandleReport()` continues to see the **raw** report (gesture detection must
  see physical GL/GR, not the remapped output). No change to its call site.
- Copying the full `Procon2Report` (~40 bytes) per report is fine.

### New rule: C held strips GL/GR (macro=on only)

Today, while a C+GL / C+GR arm hold is running (1s), the paddle's token/paddle
output leaks to the host. With assignments this gets worse (e.g. GL=ZR would fire
ZR in-game for the full second). Fix it generally:

In `processDongleTask()`, in the existing `macroOn` masking block that already
does `masked.buttons &= ~P2_BTN_C;` — when C is held, also strip the paddles:

```cpp
masked.buttons &= ~P2_BTN_C;
if (report.buttons & P2_BTN_C) {
  masked.buttons &= ~(P2_BTN_GL | P2_BTN_GR);   // C+paddle is always a control gesture
}
```

Rationale: with macro=on, any C+GL/GR combination is a recorder/assignment
gesture; the paddles must not reach the host during one. When macro=off nothing
changes (C is a normal button there).

---

## 4. Persistence (`/link.cfg`)

Append **line 13**: `glmask,grmask` as two lowercase 8-digit hex numbers,
comma-separated. Example: `0000000c,00000000` (GL = A+B, GR = none).

- `saveLinkConfig()`: after the switchmap line, add
  ```cpp
  {
    char buf[20];
    snprintf(buf, sizeof(buf), "%08lx,%08lx",
             (unsigned long)cfg.glMask, (unsigned long)cfg.grMask);
    ok &= f.println(buf) > 0;
  }
  ```
- `loadLinkConfig()`: read a 13th line. Parse with `strtoul(..., 16)` on each
  side of the comma; on any parse failure or a missing/empty line, both masks
  stay `0`. Always sanitize: `glMask &= GLGR_ASSIGNABLE_MASK;` (same for gr).
- **Backward compatibility:** an old 12-line file simply yields masks of 0 →
  legacy token behavior. No migration of old tokens into masks is performed
  (the precedence rule makes migration unnecessary). A new 13-line file read by
  old firmware: `loadLinkConfig` reads exactly the lines it knows, so the extra
  line is ignored — also safe. Update the file-format comment block above
  `loadLinkConfig()` (the numbered line list) to document line 13.

---

## 5. CFG serial protocol

### New keys (accepted between `CFG BEGIN` / `CFG END`)

```
glmap=<token>[+<token>...]   GL assignment. "none" (or empty) = clear (mask 0)
grmap=<token>[+<token>...]   GR assignment. Same syntax
```

Token vocabulary (18 tokens + none), mapping 1:1 to `GLGR_ASSIGNABLE_MASK` bits:

```
up down left right a b x y l r zl zr lstick rstick minus plus home capture
```

Parsing rules:

- Case-insensitive, trimmed. Split on `+`.
- `none` alone → mask 0. An unknown token makes the **whole line invalid**: keep
  the previous value and print `[CFG] glmap parse error: <line>` (do not
  silently drop just the bad token — a typo must not save a half-combo).
- Duplicate tokens are harmless (OR is idempotent).
- Echo the parsed result the way other keys do, in canonical token form.

Implement two helpers usable from both parser and printer:

```cpp
uint32_t glgrTokensToMask(const String &v, bool &ok);  // "a+up" -> mask
String   glgrMaskToTokens(uint32_t mask);              // mask -> "a+up" ("none" for 0)
```

Canonical output order: `up,down,left,right,a,b,x,y,l,r,zl,zr,lstick,rstick,minus,plus,home,capture`
joined with `+`.

### `CFG GET`

In the `mode == "dongle"` block, after `switchmap=`, print:

```
glmap=<tokens>
grmap=<tokens>
```

### `DMACRO` status command

Extend `dmPrintStatus()` with one line before the slots:

```
[DMACRO] glmap=<tokens> grmap=<tokens>
```

### `docs/commands.md`

Update as part of implementation: document `glmap`/`grmap` under Configuration,
document the gesture in the dongle-macro-recorder section (new table rows), and
note the precedence rule in the `ds4map`/`switchmap` entries ("overridden for
GL/GR when glmap/grmap is set").

---

## 6. On-controller gesture — user-visible behavior

Preconditions: `mode=dongle`, `macro=on`, controller connected. (With macro=off
the gesture does not exist; assignments still *apply*.)

| Step | Action | Feedback |
|---|---|---|
| 1 | Hold **C + GL + GR** together for 1s | one **long** vibration (600ms) — distinct from the short arm buzz |
| 2 | Release everything | (none) — selection mode starts 0.5s later; host sees neutral throughout |
| 3 | Hold the button(s) to assign — any combination from the candidate set — for **3s** without the set changing | on success: double vibration |
| 4 | Release everything, then press **GL** or **GR** | vibration on press; assignment saved to flash; done |

- Candidate set (step 3): **dpad up/down/left/right, A, B, X, Y, L, R, ZL, ZR,
  LSTICK, RSTICK** (14 buttons) — the whole vocabulary, the app included. System
  buttons (+/−/HOME/CAPTURE) are not assignable at all and are completely ignored
  during selection.
- Any change to the pressed candidate set — a button released, another added —
  **restarts the 3s count**. All candidates released → count stops (no timer
  running) but selection mode stays active.
- **C at any point during selection or target-wait cancels** the whole gesture
  (one long 400ms vibration, nothing saved). This is why C itself can never be
  assigned.
- **Clear gesture:** in selection mode, with *no* candidate held and no completed
  count pending, pressing GL or GR directly **clears that paddle's mask**
  (confirm vibration, saved to flash). This is the standalone way to undo an
  assignment.
- **Settle window (`DM_ASSIGN_SETTLE_MS`, 500ms):** selection does not begin at
  the instant everything is released — the fingers are still on the paddles from
  the trigger grip, and a bounce there would land as the clear gesture above and
  silently wipe an assignment. The delay always runs from a clean full release
  (pressing anything restarts it), mirroring `DM_START_DELAY_MS` on the recording
  path. **Do not remove this without also removing the clear gesture.**
- **Timeout:** 15s with no button activity in any assignment state cancels with
  a triple error vibration.
- The gesture writes `glMask`/`grMask` and saves `link.cfg` immediately; no
  RESET/reboot needed — the mask takes effect on the next report.

---

## 7. Gesture implementation — state machine

### 7.1 New states

Extend `DmState`:

```cpp
DM_ASSIGN_CLEARWAIT,   // trigger fired; wait for all buttons up before selection
DM_ASSIGN_SELECT,      // candidate selection + 3s count (also hosts the clear gesture)
DM_ASSIGN_TARGET,      // count done; wait all-up, then GL/GR picks the side
```

**`STATE_NAMES[]` in `dmPrintStatus()` MUST be extended in the same order**
(`"assign-clearwait", "assign-select", "assign-target"`). This array is indexed
by the enum — a missed entry is an out-of-bounds read.

### 7.2 New globals

```cpp
uint32_t gDmAssignPending   = 0;   // candidate set captured when the 3s count completed
uint32_t gDmAssignHeld      = 0;   // candidate set currently held (DM_ASSIGN_SELECT)
uint32_t gDmAssignCountAt   = 0;   // millis() deadline for the 3s count (0 = not counting)
uint32_t gDmAssignIdleAt    = 0;   // millis() deadline for the 15s inactivity timeout
uint8_t  gDmComboId         = 0;   // DM_RELAY combo tracking: 0 none / 1 C+GL / 2 C+GR / 3 C+GL+GR
bool     gDmAssignLatch     = false; // suppress 1/2 after 3 was seen, until C/GL/GR all released
```

All of these must be reset in `dmReset()` (and `gDmComboId`/`gDmAssignLatch`
replace/absorb the current `gDmArmMask` handling — see 7.3).

### 7.3 Trigger detection (rework of the DM_RELAY combo block)

Replace the current `comboMask` logic in `dmHandleReport()`'s `DM_RELAY` case
with a combo-id version. The existing behavior for C+GL / C+GR must be
preserved exactly, with the three-button combo added *above* it:

```cpp
uint8_t combo = 0;
if (btns & P2_BTN_C) {
  bool gl = btns & P2_BTN_GL, gr = btns & P2_BTN_GR;
  if (gl && gr)                       combo = 3;
  else if (gl && !gDmAssignLatch)     combo = 1;
  else if (gr && !gDmAssignLatch)     combo = 2;
}
if (combo == 3) gDmAssignLatch = true;
if (!(btns & (P2_BTN_C | P2_BTN_GL | P2_BTN_GR))) gDmAssignLatch = false;
if (combo != gDmComboId) {
  gDmComboId = combo;
  if (combo) gDmArmDeadline = millis() + DM_ARM_HOLD_MS;
}
```

Semantics this encodes (all deliberate):

- Adding GR to an in-flight C+GL hold upgrades the candidate to the assign combo
  and **restarts** the 1s timer (and vice versa).
- `gDmAssignLatch`: once all three were seen together, dropping back to two
  buttons (finger slip / paddle chatter) must **not** start a macro-arm
  candidate; the latch holds until C, GL and GR are all released. Without this a
  momentary GL bounce during the 3-button hold would arm the recorder.
- Combo timing keeps living in `gDmArmDeadline` (reused).

In `dmService()`, replace the arm-deadline block:

```cpp
if (gDmState == DM_RELAY && gDmComboId && (int32_t)(now - gDmArmDeadline) >= 0) {
  if (gDmComboId == 3) {
    gDmState = DM_ASSIGN_CLEARWAIT;
    gDmAssignIdleAt = now + DM_ASSIGN_TIMEOUT_MS;
    dongleSendNeutral();
    dmVibrate(1, 600, 80);            // the long "assignment mode" buzz
    Serial.println("[DMAP] assignment mode: release all buttons");
  } else {
    // existing DM_ARMED transition, unchanged (1x120 vibration etc.)
  }
  gDmComboId = 0;
}
```

Also update the two side conditions that currently reference `gDmArmMask` —
whatever replaces it must keep the "candidate dies when a button is released"
behavior implied by combo-id recomputation every report (it does: `combo`
derives from `btns` each time).

### 7.4 `dmHandleReport()` — new cases

All three new states set `forward = false` unconditionally (host stays neutral;
`dongleSendNeutral()` was sent on entry to DM_ASSIGN_CLEARWAIT and the state
machine never forwards until back in DM_RELAY via DM_RELEASE_WAIT).

Any report whose `btns` differs from `gDmPrevButtons` refreshes the inactivity
deadline in all three states: `gDmAssignIdleAt = millis() + DM_ASSIGN_TIMEOUT_MS;`

```cpp
case DM_ASSIGN_CLEARWAIT:
  forward = false;
  if (btns == 0) {
    gDmState        = DM_ASSIGN_SELECT;
    gDmAssignHeld   = 0;
    gDmAssignCountAt = 0;
    Serial.println("[DMAP] select: hold buttons to assign for 3s (C = cancel)");
  }
  break;

case DM_ASSIGN_SELECT: {
  forward = false;
  if (newly & P2_BTN_C) {                       // cancel (C cannot be assigned)
    dmAssignCancel("C");                        // helper: vib 1x400, -> DM_RELEASE_WAIT
    break;
  }
  uint32_t held = btns & DM_ASSIGN_CANDIDATE_MASK;
  if (held == 0 && (newly & (P2_BTN_GL | P2_BTN_GR))) {
    // clear gesture: GL/GR pressed with nothing selected
    bool isGl = (newly & P2_BTN_GL) != 0;       // GL wins if somehow both are new
    if (isGl) gLinkConfig.glMask = 0; else gLinkConfig.grMask = 0;
    dmAssignCommit(isGl, 0);                    // helper: save cfg, vib, -> DM_RELEASE_WAIT
    break;
  }
  if (held != gDmAssignHeld) {
    gDmAssignHeld = held;
    gDmAssignCountAt = held ? millis() + DM_ASSIGN_HOLD_MS : 0;   // restart or stop
  }
  // GL/GR pressed while candidates are held: treat as "additional press" -> restart
  if ((newly & (P2_BTN_GL | P2_BTN_GR)) && held) {
    gDmAssignCountAt = millis() + DM_ASSIGN_HOLD_MS;
  }
  break;
}

case DM_ASSIGN_TARGET:
  forward = false;
  if (newly & P2_BTN_C) { dmAssignCancel("C"); break; }
  // Accept GL/GR only after everything from the count has been released once.
  // Track with gDmAssignHeld: it is set to 1 (sentinel "still dirty") on entry
  // and cleared to 0 the first time btns == 0.
  if (btns == 0) { gDmAssignHeld = 0; break; }
  if (gDmAssignHeld == 0 && (newly & (P2_BTN_GL | P2_BTN_GR))) {
    bool isGl = (newly & P2_BTN_GL) != 0;
    if (isGl) gLinkConfig.glMask = gDmAssignPending;
    else      gLinkConfig.grMask = gDmAssignPending;
    dmAssignCommit(isGl, gDmAssignPending);
  }
  // any other button while waiting: ignored
  break;
```

The 3s count completion lives in `dmService()` (it must fire without a fresh
report — the user is holding still, and notifications may dedupe):

```cpp
if (gDmState == DM_ASSIGN_SELECT && gDmAssignCountAt &&
    (int32_t)(now - gDmAssignCountAt) >= 0) {
  gDmAssignPending = gDmAssignHeld & DM_ASSIGN_CANDIDATE_MASK;  // belt and braces
  gDmAssignCountAt = 0;
  gDmAssignHeld    = 1;            // sentinel: TARGET requires a full release first
  gDmState         = DM_ASSIGN_TARGET;
  gDmAssignIdleAt  = now + DM_ASSIGN_TIMEOUT_MS;
  dmVibrate(2, 120, 100);
  Serial.printf("[DMAP] captured %s: press GL or GR to assign\n",
                glgrMaskToTokens(gDmAssignPending).c_str());
}
```

Inactivity timeout, also in `dmService()`:

```cpp
if ((gDmState == DM_ASSIGN_CLEARWAIT || gDmState == DM_ASSIGN_SELECT ||
     gDmState == DM_ASSIGN_TARGET) &&
    (int32_t)(now - gDmAssignIdleAt) >= 0) {
  dmVibrate(3, 80, 80);
  gDmState = DM_RELEASE_WAIT;
  Serial.println("[DMAP] timed out");
}
```

### 7.5 Helpers

```cpp
void dmAssignCancel(const char *why) {
  dmVibrate(1, 400, 80);
  gDmState = DM_RELEASE_WAIT;
  Serial.printf("[DMAP] cancelled (%s)\n", why);
}

void dmAssignCommit(bool isGl, uint32_t mask) {
  saveLinkConfig(gLinkConfig);       // host is neutral here; the flash stall is safe
  dmVibrate(1, 120, 80);
  gDmState = DM_RELEASE_WAIT;
  Serial.printf("[DMAP] %s = %s\n", isGl ? "GL" : "GR",
                glgrMaskToTokens(mask).c_str());
}
```

`dmAssignCommit` mutates `gLinkConfig` *before* being called (see call sites) and
persists the whole config. `saveLinkConfig` failure: still apply the in-RAM value
(the assignment works until reboot) but log `[DMAP] save FAILED` and use the
error vibration `dmVibrate(3, 80, 80)` instead of the confirm buzz.

Note on the flash write: it happens while the host output is neutral and no
recording is running — the same reasoning as `dmSaveSlot()`. Do not move the
save into DM_RELAY.

### 7.6 Vibration pattern map (must stay distinguishable)

| Pattern | Existing meaning | New meaning |
|---|---|---|
| 1×120ms | armed / rec start / rec stop | assignment saved / cleared |
| 2×120ms | slot saved / playback start | 3s count captured |
| 3×80ms | error (empty slot / save fail) | assignment timeout / save fail |
| 1×400ms | recording discarded | assignment cancelled (C) |
| **1×1000ms** | — | **assignment mode entered** (the "long" buzz) |

`dmVibService` **resends `setRumble` every 100ms while a pulse is on**. The HD
pattern decays on its own (the host rumble relay already resends for exactly this
reason), so without it a long pulse is felt as a short tap regardless of `onMs` —
the 600ms buzz this started at was indistinguishable from the 120ms one on
hardware. The resend is suppressed within 50ms of the pulse ending so it cannot
crowd the OFF write inside `setRumble`'s 30ms spacing, which leaves every pulse
of 120ms or shorter behaving exactly as before.
Do not add per-restart pulses during the 3s count (setRumble's 30ms write
spacing makes rapid restarts race the pattern engine).

---

## 8. Guard rails and interactions (every one of these is a required edit)

1. **`STATE_NAMES[]`** — extend, same order as the enum (see 7.1).
2. **`dmReset()`** — zero all new globals (7.2). Existing callers (controller
   disconnect, USB unmount, `macro=off` via RESET) then already handle the new
   states because they test `gDmState != DM_RELAY`, which remains true for the
   `DM_ASSIGN_*` states. Verify all three call sites compile-time reference
   nothing state-specific.
3. **Rumble relay** in `processDongleTask()`: add the three `DM_ASSIGN_*` states
   to the *discard* branch alongside `DM_RECORDING` / `DM_PLAYING` (host rumble
   must not fire mid-gesture — the recorder's vibration is the only feedback
   channel there, and a stale request must not buzz after the gesture ends).
   Suggest a helper `bool dmOwnsActuator()` returning
   `state ∈ {RECORDING, PLAYING, ASSIGN_CLEARWAIT, ASSIGN_SELECT, ASSIGN_TARGET}`
   to keep the two conditions (skip + discard) in sync.
4. **`dongleSendNeutral()` on entry** to DM_ASSIGN_CLEARWAIT (7.3) — sticks and
   IMU go neutral along with buttons; nothing leaks for the whole gesture.
5. **Slot playback / arm interplay**: the C-first slot check runs only in
   `DM_RELAY`; the new states never fall through to it. The assign trigger fires
   from DM_RELAY only. No change needed — but do not reorder the switch cases.
6. **Recording data**: recordings keep storing **raw** buttons (GL/GR bits
   included). Remap happens at playback inside `dongleApplyReport` (section 3).
   Do not remap before sampling.
7. **`macro=on` C-strip block**: extend with the C-held paddle strip (section 3).
8. **Sanitization**: `GLGR_ASSIGNABLE_MASK` AND at all three entry points (file,
   CFG key, gesture capture).
9. **Opposite dpad directions** (e.g. UP+DOWN assigned together): allowed; the
   identity converters resolve it (DS4 hat picks by its priority order). Not the
   firmware's problem to police.
10. **usbmode=switch note**: the feedback vibration goes to the controller over
    BLE, not the USB host, so the gesture works on a Switch console too —
    unchanged from the recorder.

---

## 9. KarakuriPad app changes

Dongle configuration is over **USB serial only** (dongle mode has no app link),
so this touches the app's serial CFG screen.

1. **New GL/GR assignment UI**: per paddle, a multi-select over the 18-token
   vocabulary (section 5) plus "none". Sends `glmap=` / `grmap=` inside the
   existing `CFG BEGIN`/`CFG END` block. Parse the echoes like other keys.
2. **Read**: parse the new `glmap=` / `grmap=` lines from `CFG GET` (dongle mode
   only). **Re-issue `CFG GET` every time the screen is shown** — the firmware
   gesture may have changed the masks since the app last looked.
3. **Old-firmware detection**: if `CFG GET` (with `mode=dongle`) contains no
   `glmap=` line, the firmware predates this feature → hide the new UI and show
   the legacy GL/GR token pickers instead.
4. **Legacy GL/GR pickers**: on firmware that *does* support `glmap`, remove the
   GL/GR fields from the ds4map/switchmap UI (keep the C picker). When the user
   saves from the new UI, also send `ds4map=<c>,none,none` and
   `switchmap=<c>,none,none` (preserving the current C tokens) so a later
   `glmap=none` cannot resurrect a forgotten token assignment.
5. **UX for the gesture**: document the on-controller gesture in the app's help
   (the table in section 6).

---

## 10. Out of scope

- C button assignment stays token-based and per-usbmode (unchanged).
- No per-usbmode masks: the assignment is deliberately mode-independent.
- No paddle access to +/−/HOME/CAPTURE at all (neither gesture nor app).
- No change to bt/wifi modes, the string-macro engine, or the recorder's
  slot/record/playback flows beyond what section 8 lists.

## 11. Acceptance checklist

Firmware (serial):
- [ ] `glmap=a+up` between CFG BEGIN/END, CFG END, then `CFG GET` shows `glmap=up+a` (canonical order); survives reboot (link.cfg line 13)
- [ ] `glmap=bogus` → parse error, previous value kept
- [ ] Old 12-line link.cfg loads with masks 0; legacy ds4map/switchmap GL/GR tokens still fire
- [ ] With `glMask` set, GL press emits the mask (ds4: dpad via hat, ZL sets L2 analog; switch: dPad; procon: dpad bits); token/paddle suppressed. `grMask=0` GR untouched
- [ ] Assignment applies with `macro=off`

Gesture (mode=dongle, macro=on, each usbmode at least once with usbmode=ds4 and usbmode=switch):
- [ ] C+GL+GR 1s → long buzz; C+GL 1s alone still arms recording (regression)
- [ ] GL+GR gripped first, then C → assign trigger (order independence)
- [ ] During 3-button hold, GL bounce does not arm the recorder (latch)
- [ ] Host sees neutral for the entire gesture; no assigned output leaks during the 1s trigger hold (C-strip rule)
- [ ] Hold A+UP 3s → double buzz; release; GL press → buzz; GL now = A+UP; persisted
- [ ] Changing held set mid-count restarts the 3s; releasing everything stops the count without cancelling
- [ ] C during select and during target-wait cancels (400ms buzz), nothing saved
- [ ] GL press in select with nothing held clears GL; 15s idle → triple buzz, cancelled
- [ ] Releasing the trigger grip sloppily (paddle bounce right after the release) does NOT clear an assignment — the 0.5s settle absorbs it
- [ ] Recorder still fully works after all of the above (record, save, play, C-stop)
- [ ] `DMACRO` prints the new state names and glmap/grmap
- [ ] Playback of a recording containing GL presses follows the *current* assignment

App:
- [ ] New UI round-trips glmap/grmap; hidden on old firmware; legacy GL/GR pickers gone on new firmware; ds4map/switchmap GL/GR forced to none on save
