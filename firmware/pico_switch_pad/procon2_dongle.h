#pragma once

#include <BLE.h>
#include <BluetoothLock.h>

// The core's BLERemoteCharacteristic::setValue() requires the Write (with response)
// property, so it always fails on the Pro Controller 2's WriteWithoutResponse-only
// command / rumble characteristics. The core has no WWR API, so call btstack directly
// (<btstack.h> collides with TinyUSB, hence these declarations).
extern "C" uint8_t gatt_client_write_value_of_characteristic_without_response(
    uint16_t con_handle, uint16_t value_handle, uint16_t value_length, uint8_t *value);
extern "C" void gap_set_connection_parameters(
    uint16_t conn_scan_interval, uint16_t conn_scan_window,
    uint16_t conn_interval_min, uint16_t conn_interval_max, uint16_t conn_latency,
    uint16_t supervision_timeout, uint16_t min_ce_length, uint16_t max_ce_length);

// con_handle / _valueHandle() are protected. Explicit instantiation skips access
// checking (a loophole in the C++ standard), so this extracts them legally.
template <typename Tag, typename Tag::type M>
struct Procon2Rob {
    friend typename Tag::type procon2Steal(Tag) { return M; }
};
struct Procon2ConHandleTag {
    typedef volatile uint16_t BLERemoteCharacteristic::*type;
    friend type procon2Steal(Procon2ConHandleTag);
};
template struct Procon2Rob<Procon2ConHandleTag, &BLERemoteCharacteristic::con_handle>;
struct Procon2ValueHandleTag {
    typedef uint16_t (BLERemoteCharacteristic::*type)();
    friend type procon2Steal(Procon2ValueHandleTag);
};
template struct Procon2Rob<Procon2ValueHandleTag, &BLERemoteCharacteristic::_valueHandle>;
// For GATT enumeration (debug): the characteristic list and UUID are protected too
struct Procon2SvcCharsTag {
    typedef BLERemoteCharacteristicList BLERemoteService::*type;
    friend type procon2Steal(Procon2SvcCharsTag);
};
template struct Procon2Rob<Procon2SvcCharsTag, &BLERemoteService::_characteristic>;
struct Procon2CharUuidTag {
    typedef BLEUUID BLERemoteCharacteristic::*type;
    friend type procon2Steal(Procon2CharUuidTag);
};
template struct Procon2Rob<Procon2CharUuidTag, &BLERemoteCharacteristic::_uuid>;

// ==============================
// Dongle mode: BLE connection to a Pro Controller 2 (Switch 2 Pro Controller)
//
// The Pico connects to the controller as a BLE central. No PC involved. Besides the
// handshake-free simple report (7492866c-...f9) the app uses for recording and
// passthrough, the controller has a full input report path (IMU and the C/GL/GR
// buttons included) that an initialization sequence unlocks; the dongle uses that.
// The init sequence comes from SendProCon2OfficialInit in TheFrano/joycon2cpp
// (MIT License); the rumble sample encoding was derived in-house from the console
// captures in ndeadly/switch2_controller_research (see setRumble).
// TommyWabg/Switch2Connect was a protocol reference only (no code or constants taken):
//   - Input: notifications on characteristic ab7de9be-89fe-49ad-828f-118f09df7fd2
//   - Init (15 in a row) and rumble: both written to 3dacbc7e-6955-40b5-8eaf-6f9809e8b379
//     (649d...f005 is LED/audio-only and does not accept the full init. Confirmed on hardware)
//   - Report: time u32 @0 / buttons u32 LE @4 / sticks 2 x 12bit @10, @13 /
//     valid marker 0x01 @0x29 / accel 3 x s16 @0x30 / gyro 3 x s16 @0x36
// ==============================

// Button masks in the full report (u32 little-endian @4)
enum : uint32_t {
    P2_BTN_Y       = 0x00000001,
    P2_BTN_X       = 0x00000002,
    P2_BTN_B       = 0x00000004,
    P2_BTN_A       = 0x00000008,
    P2_BTN_R       = 0x00000040,
    P2_BTN_ZR      = 0x00000080,
    P2_BTN_MINUS   = 0x00000100,
    P2_BTN_PLUS    = 0x00000200,
    P2_BTN_RSTICK  = 0x00000400,
    P2_BTN_LSTICK  = 0x00000800,
    P2_BTN_HOME    = 0x00001000,
    P2_BTN_CAPTURE = 0x00002000,
    P2_BTN_C       = 0x00004000,
    P2_BTN_DOWN    = 0x00010000,
    P2_BTN_UP      = 0x00020000,
    P2_BTN_RIGHT   = 0x00040000,
    P2_BTN_LEFT    = 0x00080000,
    P2_BTN_L       = 0x00400000,
    P2_BTN_ZL      = 0x00800000,
    P2_BTN_GR      = 0x01000000,
    P2_BTN_GL      = 0x02000000,
};

// A parsed full input report
struct Procon2Report {
    uint32_t buttons;          // bitmask of P2_BTN_*
    uint16_t lx, ly, rx, ry;   // raw 12-bit values (0-4095, center ~2048, up and right are larger)
    int16_t accel[3];          // raw values (4096 LSB/g, +/-8g)
    int16_t gyro[3];           // raw values (14.286 LSB/dps, approx. +/-2294dps)
};

class Procon2Link {
public:
    // ab7de9be-89fe-49ad-828f-118f09df7fd2
    static constexpr uint8_t INPUT_CHAR_UUID[16] = {
        0xab, 0x7d, 0xe9, 0xbe, 0x89, 0xfe, 0x49, 0xad,
        0x82, 0x8f, 0x11, 0x8f, 0x09, 0xdf, 0x7f, 0xd2,
    };
    // 3dacbc7e-6955-40b5-8eaf-6f9809e8b379
    static constexpr uint8_t RUMBLE_CHAR_UUID[16] = {
        0x3d, 0xac, 0xbc, 0x7e, 0x69, 0x55, 0x40, 0xb5,
        0x8e, 0xaf, 0x6f, 0x98, 0x09, 0xe8, 0xb3, 0x79,
    };
    // 649d4ac9-8eb7-4e6c-af44-1ea54fe5f005
    static constexpr uint8_t COMMAND_CHAR_UUID[16] = {
        0x64, 0x9d, 0x4a, 0xc9, 0x8e, 0xb7, 0x4e, 0x6c,
        0xaf, 0x44, 0x1e, 0xa5, 0x4f, 0xe5, 0xf0, 0x05,
    };

    static constexpr size_t MIN_REPORT_LEN   = 0x3C;
    static constexpr size_t MARKER_OFFSET    = 0x29;   // a valid report has 0x01 here
    static constexpr size_t ACCEL_OFFSET     = 0x30;
    static constexpr size_t GYRO_OFFSET      = 0x36;

    // Switch for the protocol-investigation logs ([GATT]/[P2RSP]/cmd props/subscribed).
    // Toggled by the DEBUG serial command; it suppresses output only, and the work
    // (subscribing, draining the ring) runs exactly as in normal operation
    inline static bool debugLog = false;

    bool connected() { return _connected && BLE.client()->connected(); }

    // The link is alive but notifications stopped. Callers watch this and reconnect
    // (the GAP link can survive the controller going to sleep).
    bool stalled() const {
        return _connected && (int32_t)(millis() - _lastValidReportAt) > 3000;
    }

    // If not connected, scan for 2 seconds and try a matching device. BLE.scan() and
    // the init sequence (about 1.2 seconds) both block, so callers must space out calls.
    bool scanAndConnect() {
        // btstack defaults to a 30ms max interval with latency 4, and the controller
        // notifies once per connection event, dropping the report rate to ~32Hz.
        // Request 7.5-15ms with latency 0 (scan and supervision timeouts stay default).
        gap_set_connection_parameters(0x0060, 0x0030, 6, 12, 0, 0x0048, 0, 0);

        auto report = BLE.scan(2 /*sec*/, true);
        for (auto &item : *report) {
            if (!isProcon2Advertisement(item)) continue;

            Serial.print("[DONGLE] found Pro Controller 2: ");
            Serial.println(item.toString());

            // Try only the first device found; trying the rest would add seconds per failure
            return tryConnect(item);
        }
        return false;
    }

    void markDisconnected() {
        if (_connected) {
            _connected = false;
            Serial.println("[DONGLE] Pro Controller 2 disconnected");
        }
        _input = nullptr;
        _cmd = nullptr;
        _rumble = nullptr;
        BLE.client()->disconnect();
    }

    // Parses and returns a new report if one has arrived (tear-free copy via a seqlock)
    bool fetchReport(Procon2Report &out) {
        uint32_t seq = _reportSeq;
        asm volatile("" ::: "memory");   // read the sequence number before copying the body
        if ((seq & 1) || seq == _consumedSeq) return false;

        uint8_t buf[RAW_LEN];
        memcpy(buf, (const void *)_raw, RAW_LEN);
        size_t len = _rawLen;
        asm volatile("" ::: "memory");   // re-check the sequence number after the copy finishes
        if (_reportSeq != seq) return false;   // updated while we were copying

        _consumedSeq = seq;
        if (len < MIN_REPORT_LEN) return false;
        if (buf[MARKER_OFFSET] != 0x01) return false;   // invalid report from before initialization

        parse(buf, out);
        _lastValidReportAt = millis();
        return true;
    }

    // Sends an arbitrary byte string to the command char (for the P2CMD serial command / protocol work)
    bool sendRaw(const uint8_t *d, size_t n) {
        return _cmd ? writeToChar(_cmd, d, (uint16_t)n) : false;
    }

    // Finds a characteristic by the last 2 bytes of its UUID and writes to it (for the P2W command)
    bool sendRawTo(uint16_t suffix, const uint8_t *d, size_t n) {
        BLERemoteCharacteristic *ch = findCharBySuffix(suffix);
        return ch ? writeToChar(ch, d, (uint16_t)n) : false;
    }

    // Takes one response (notification). tag is the last 2 bytes of the source char UUID
    bool takeCmdResponse(uint8_t *dst, size_t &outLen, uint16_t &tag) {
        if (_rspRead >= _rspWrite) return false;
        if (_rspWrite - _rspRead > RSP_SLOTS) _rspRead = _rspWrite - RSP_SLOTS;   // dropped entries
        uint32_t slot = _rspRead % RSP_SLOTS;
        size_t n = _rspLen[slot];
        memcpy(dst, (const void *)_rspBuf[slot], n);
        outLen = n;
        tag = _rspTag[slot];
        _rspRead++;
        return true;
    }

    // Copies the latest raw report as-is (debug; unlike fetchReport it does not mark it
    // consumed). Returns the length, or 0 if nothing could be read
    size_t copyRaw(uint8_t *dst) {
        uint32_t seq = _reportSeq;
        asm volatile("" ::: "memory");
        if (seq & 1 || seq == 0) return 0;
        memcpy(dst, (const void *)_raw, RAW_LEN);
        size_t len = _rawLen;
        asm volatile("" ::: "memory");
        if (_reportSeq != seq) return 0;
        return len;
    }

    // Rumble with a perceptual level 0-255 per side (0 stops; the Switch
    // amplitude-index scale where 255 = index 100 = maximum). Callers must space
    // writes at least 30ms apart.
    //
    // Sample format (decoded from ndeadly's switch2_controller_research capture of
    // a real console driving a Pro Controller 2): each side is a 16-byte block of
    // [0x50 | counter][5-byte sample][zeros], and the 5-byte sample is a 40-bit
    // little-endian value packing [amp2:10][freq2:10][amp1:10][freq1:10] from the
    // MSB. The amplitude fields use the classic Switch amplitude-index scale x4
    // (console idle streams amp 0 at freq 388/481, game envelopes stay in 0-275,
    // and the community-measured weak/strong patterns sit at 31/397 = index
    // ~8/~100). Frequencies are held near the console's idle values; only the
    // amplitudes are driven.
    void setRumble(uint8_t left, uint8_t right) {
        if (!_rumble || !connected()) return;

        uint8_t pkt[58] = {};
        if (left > 0 || right > 0) {
            pkt[1] = 0x50 | (_rumbleCounter & 0x0F);
            pkt[17] = 0x50 | (_rumbleCounter & 0x0F);
            encodeRumbleSample(&pkt[2], left);
            encodeRumbleSample(&pkt[18], right);
            _rumbleCounter++;
        }
        writeToChar(_rumble, pkt, sizeof(pkt));
    }

private:
    // Builds one 5-byte rumble sample for a perceptual level 0-255 (index scale).
    // The nominal mapping would be field = index x4 (the strong preset's level,
    // 400, at full scale), but that reads stronger on the relayed controller than
    // the same request feels on a directly connected one. A linear trim would
    // push the faintest effects below perception, so use a soft knee instead:
    // 1:1 at the bottom (a faint buzz stays a faint buzz), compressing toward a
    // full scale of 300. field = 1200*amp / (765 + amp).
    // The low band (freq 388) carries the amplitude, the high band rides along
    // quietly at roughly the ratio the community presets use (strong: 397/52).
    static void encodeRumbleSample(uint8_t *out, uint8_t amp) {
        if (amp == 0) return;   // all-zero sample = silence
        uint32_t a1 = (uint32_t)amp * 1200 / (765 + (uint32_t)amp);
        if (a1 == 0) a1 = 1;
        uint32_t a2 = a1 / 8;
        const uint32_t f1 = 388, f2 = 460;
        uint64_t v = (uint64_t)f1
                   | ((uint64_t)a1 << 10)
                   | ((uint64_t)f2 << 20)
                   | ((uint64_t)a2 << 30);
        for (int i = 0; i < 5; i++) out[i] = (uint8_t)(v >> (8 * i));
    }
    // A notification on the simple path was 112B, so the full path may exceed 64B too.
    // We only use offsets up to 0x3B, but leave room for DUMP investigation
    static constexpr size_t RAW_LEN = 96;

    // Writes to a characteristic, preferring WriteWithoutResponse (the only mode the
    // command / rumble chars support, and it does not block). Falls back to the core's
    // setValue() only for Write-only chars.
    static bool writeToChar(BLERemoteCharacteristic *ch, const uint8_t *data, uint16_t len) {
        if (!ch) return false;
        if (ch->canWriteWithoutResponse()) {
            uint16_t con = ch->*procon2Steal(Procon2ConHandleTag{});
            uint16_t vh = (ch->*procon2Steal(Procon2ValueHandleTag{}))();
            if (!con || !vh) return false;
            // A full ATT buffer returns BUSY, so wait a moment and retry
            for (int i = 0; i < 20; i++) {
                uint8_t rc;
                {
                    BluetoothLock lock;
                    rc = gatt_client_write_value_of_characteristic_without_response(
                        con, vh, len, (uint8_t *)data);
                }
                if (rc == 0) return true;
                delay(5);
            }
            return false;
        }
        if (ch->canWrite()) return ch->setValue((uint8_t *)data, len);
        return false;
    }

    bool tryConnect(BLEAdvertising &item) {
        // The core returns false on failed service discovery but leaves the GAP link up.
        // Without disconnecting here the controller never re-advertises and cannot come back.
        if (!BLE.client()->connect(item, 4)) {
            Serial.println("[DONGLE] connect failed");
            BLE.client()->disconnect();
            return false;
        }

        _input  = findCharacteristic(INPUT_CHAR_UUID);
        _cmd    = findCharacteristic(COMMAND_CHAR_UUID);
        _rumble = findCharacteristic(RUMBLE_CHAR_UUID);
        if (!_input || !_cmd) {
            Serial.println("[DONGLE] required characteristics not found -> disconnect");
            markDisconnected();
            return false;
        }
        if (debugLog) {
            Serial.printf("[DONGLE] cmd props: write=%d wwr=%d / rumble: write=%d wwr=%d\n",
                          _cmd->canWrite(), _cmd->canWriteWithoutResponse(),
                          _rumble ? _rumble->canWrite() : -1,
                          _rumble ? _rumble->canWriteWithoutResponse() : -1);
        }
        dumpGatt();

        // Subscribe to every command response channel: the command char (F005) has no
        // notify, and responses arrive on separate notify-only chars. Take everything
        // except the input streams (D2/F9) and log it with a tag
        _rspRead = _rspWrite;
        subscribeDebugNotifies();

    // Following the implementation we ported from (joycon2cpp): enable notifications before sending init
        _reportSeq = 0;
        _consumedSeq = 0;
        _input->onNotify(onNotifyStatic);
        if (!_input->enableNotifications()) {
            Serial.println("[DONGLE] enableNotifications failed -> disconnect");
            markDisconnected();
            return false;
        }

        // Official init sequence that enables the full report (IMU included). A dropped
        // write means reports never arrive, so redo it on every connection.
        if (!sendInitSequence()) {
            Serial.println("[DONGLE] init sequence failed -> disconnect");
            markDisconnected();
            return false;
        }

        _lastValidReportAt = millis();   // so stalled() does not fire immediately after
        _connected = true;
        Serial.println("[DONGLE] Pro Controller 2 connected (full report mode)");
        return true;
    }

    static bool isProcon2Advertisement(BLEAdvertising &adv) {
        const uint8_t *md = nullptr;
        int len = adv.getManufacturerData(&md);
        if (md && len >= 4) {
            uint16_t companyId = (uint16_t)md[0] | ((uint16_t)md[1] << 8);
            if (companyId == 0x0553 || companyId == 0x057e) {
                for (int i = 0; i + 1 < len; i++) {
                    if (md[i] == 0x69 && md[i + 1] == 0x20) return true;   // PID 0x2069 (LE)
                }
            }
        }
        // In some states the manufacturer data is absent and only the name shows up
        const char *name = adv.getName();
        return name && strstr(name, "Pro Controller");
    }

    BLERemoteCharacteristic *findCharacteristic(const uint8_t uuid[16]) {
        auto svcs = BLE.client()->services();
        if (!svcs) return nullptr;
        for (auto s : *svcs) {
            BLERemoteCharacteristic *ch = s->characteristic(BLEUUID(uuid));
            if (ch) return ch;
        }
        return nullptr;
    }

    // Init command sequence that enables the full report and the IMU (ported from
    // joycon2cpp's SendProCon2OfficialInit). Each command carries a 33-byte zero prefix.
    // They go to the rumble char (b379), not the command char (f005): joycon2cpp sends
    // both init and rumble to rumbleChar, and f005 is LED/audio-only (sending there was
    // confirmed on hardware to leave the IMU off with no ACK).
    bool sendInitSequence() {
        static const uint8_t CMDS[][28] = {
            { 0x07,0x91,0x01,0x01,0x00,0x00,0x00,0x00 },
            { 0x02,0x91,0x01,0x04,0x00,0x08,0x00,0x00,0x40,0x7E,0x00,0x00,0x00,0x30,0x01,0x00 },
            { 0x16,0x91,0x01,0x01,0x00,0x00,0x00,0x00 },
            { 0x0A,0x91,0x01,0x02,0x00,0x04,0x00,0x00,0x03,0x00,0x00,0x00 },
            { 0x09,0x91,0x01,0x07,0x00,0x08,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
            { 0x0C,0x91,0x01,0x02,0x00,0x04,0x00,0x00,0x2F,0x00,0x00,0x00 },
            { 0x02,0x91,0x01,0x04,0x00,0x08,0x00,0x00,0x40,0x7E,0x00,0x00,0x80,0x30,0x01,0x00 },
            { 0x02,0x91,0x01,0x04,0x00,0x08,0x00,0x00,0x40,0x7E,0x00,0x00,0xC0,0x30,0x01,0x00 },
            { 0x02,0x91,0x01,0x04,0x00,0x08,0x00,0x00,0x40,0x7E,0x00,0x00,0x40,0xC0,0x1F,0x00 },
            { 0x02,0x91,0x01,0x04,0x00,0x08,0x00,0x00,0x10,0x7E,0x00,0x00,0x40,0x30,0x01,0x00 },
            { 0x02,0x91,0x01,0x04,0x00,0x08,0x00,0x00,0x18,0x7E,0x00,0x00,0x00,0x31,0x01,0x00 },
            { 0x11,0x91,0x01,0x03,0x00,0x00,0x00,0x00 },
            { 0x02,0x91,0x01,0x04,0x00,0x08,0x00,0x00,0x20,0x7E,0x00,0x00,0x60,0x30,0x01,0x00 },
            { 0x0A,0x91,0x01,0x08,0x00,0x14,0x00,0x00,
              0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x35,0x00,0x46,
              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },
            { 0x0C,0x91,0x01,0x04,0x00,0x04,0x00,0x00,0x2F,0x00,0x00,0x00 },
        };
        static const uint8_t CMD_LENS[] = { 8, 16, 8, 12, 16, 12, 16, 16, 16, 16, 16, 8, 16, 28, 12 };

        uint8_t pkt[33 + 28];
        for (size_t i = 0; i < sizeof(CMD_LENS); i++) {
            memset(pkt, 0, 33);
            memcpy(&pkt[33], CMDS[i], CMD_LENS[i]);
            if (!writeToChar(_rumble, pkt, 33 + CMD_LENS[i])) return false;
            delay(80);
            drainResponsesToSerial((int)i);
        }
        return true;
    }

    // Drains queued command responses (idx says which init command they followed).
    // Printing only happens under debugLog; the draining itself always runs
    void drainResponsesToSerial(int idx) {
        uint8_t rsp[64];
        size_t n;
        uint16_t tag;
        while (takeCmdResponse(rsp, n, tag)) {
            if (!debugLog) continue;
            Serial.printf("[P2RSP] after#%d from=%04X len=%u ", idx, tag, (unsigned)n);
            for (size_t i = 0; i < n; i++) Serial.printf("%02X", rsp[i]);
            Serial.println();
        }
    }

    // Finds a char by the last 2 bytes of its UUID (bytes [14][15] of a 128-bit UUID)
    BLERemoteCharacteristic *findCharBySuffix(uint16_t suffix) {
        auto svcs = BLE.client()->services();
        if (!svcs) return nullptr;
        for (auto s : *svcs) {
            auto &chars = s->*procon2Steal(Procon2SvcCharsTag{});
            for (auto ch : chars) {
                if (charSuffix(ch) == suffix) return ch;
            }
        }
        return nullptr;
    }

    static uint16_t charSuffix(BLERemoteCharacteristic *ch) {
        BLEUUID &u = ch->*procon2Steal(Procon2CharUuidTag{});
        if (u.is16) return u.uuid16;
        return (uint16_t)((u.uuid128[14] << 8) | u.uuid128[15]);
    }

    // Subscribes to notifications on the response channels (836A/57E0/2A80).
    // **Init commands are only accepted with this subscription in place; removing it
    // stops the full report** (confirmed on hardware; it must run regardless of debugLog).
    // Input streams (C0F8/C0F9 simple, 7FD2 full = _input, 7FDE) are excluded: they
    // stream continuously, overflowing the response ring.
    void subscribeDebugNotifies() {
        static const uint16_t SKIP[] = { 0xC0F8, 0xC0F9, 0x7FD2, 0x7FDE };
        auto svcs = BLE.client()->services();
        if (!svcs) return;
        for (auto s : *svcs) {
            auto &chars = s->*procon2Steal(Procon2SvcCharsTag{});
            for (auto ch : chars) {
                if (!ch->canNotify() || ch == _input) continue;
                uint16_t sfx = charSuffix(ch);
                bool skip = false;
                for (uint16_t k : SKIP) if (sfx == k) skip = true;
                if (skip) continue;
                ch->onNotify(onCmdNotifyStatic);
                ch->enableNotifications();
                if (debugLog) Serial.printf("[DONGLE] subscribed notify %04X\n", sfx);
            }
        }
    }

    // Full GATT enumeration (debug output only; reads the local cache, no BLE traffic)
    void dumpGatt() {
        if (!debugLog) return;
        auto svcs = BLE.client()->services();
        if (!svcs) return;
        for (auto s : *svcs) {
            auto &chars = s->*procon2Steal(Procon2SvcCharsTag{});
            Serial.printf("[GATT] service (%d chars)\n", (int)chars.size());
            for (auto ch : chars) {
                BLEUUID &u = ch->*procon2Steal(Procon2CharUuidTag{});
                Serial.print("[GATT]   ");
                if (u.is16) {
                    Serial.printf("%04X", u.uuid16);
                } else {
                    for (int i = 0; i < 16; i++) Serial.printf("%02X", u.uuid128[i]);
                }
                Serial.printf(" R=%d W=%d WWR=%d N=%d\n",
                              ch->canRead(), ch->canWrite(),
                              ch->canWriteWithoutResponse(), ch->canNotify());
            }
        }
    }

    // Called from the BT context, so do not print here; just push onto the ring
    static void onCmdNotifyStatic(BLERemoteCharacteristic *c, const uint8_t *data, uint32_t len) {
        uint32_t slot = _rspWrite % RSP_SLOTS;
        size_t n = (len < 64) ? len : 64;
        memcpy((void *)_rspBuf[slot], data, n);
        _rspLen[slot] = (uint8_t)n;
        _rspTag[slot] = charSuffix(c);
        _rspWrite = _rspWrite + 1;
    }

    // Called from the BT context. A seqlock (odd = write in progress) keeps only the latest value
    static void onNotifyStatic(BLERemoteCharacteristic *c, const uint8_t *data, uint32_t len) {
        (void)c;
        if (!data || len < MIN_REPORT_LEN) return;
        _reportSeq++;
        size_t n = (len < RAW_LEN) ? len : RAW_LEN;
        memcpy((void *)_raw, data, n);
        _rawLen = n;
        _reportSeq++;
    }

    static uint16_t stick12(const uint8_t *p, bool high) {
        // 3 bytes pack 2 x 12-bit values (X, Y)
        return high ? (uint16_t)(((p[1] & 0xF0) >> 4) | (p[2] << 4))
                    : (uint16_t)(p[0] | ((p[1] & 0x0F) << 8));
    }

    static int16_t s16le(const uint8_t *p) {
        return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    }

    static void parse(const uint8_t *b, Procon2Report &out) {
        out.buttons = (uint32_t)b[4] | ((uint32_t)b[5] << 8) |
                      ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 24);
        out.lx = stick12(&b[10], false);
        out.ly = stick12(&b[10], true);
        out.rx = stick12(&b[13], false);
        out.ry = stick12(&b[13], true);
        for (int i = 0; i < 3; i++) {
            out.accel[i] = s16le(&b[ACCEL_OFFSET + i * 2]);
            out.gyro[i]  = s16le(&b[GYRO_OFFSET + i * 2]);
        }
    }

    bool _connected = false;
    BLERemoteCharacteristic *_input = nullptr;
    BLERemoteCharacteristic *_cmd = nullptr;
    BLERemoteCharacteristic *_rumble = nullptr;
    uint32_t _consumedSeq = 0;
    uint32_t _lastValidReportAt = 0;
    uint8_t _rumbleCounter = 0;

    // Define them here as C++17 inline variables (no .cpp needed)
    inline static volatile uint32_t _reportSeq = 0;
    inline static volatile size_t _rawLen = 0;
    inline static volatile uint8_t _raw[RAW_LEN] = {};

    // Ring of command responses (for debugging; keeps only the latest RSP_SLOTS entries)
    static constexpr size_t RSP_SLOTS = 8;
    inline static volatile uint8_t _rspBuf[RSP_SLOTS][64] = {};
    inline static volatile uint8_t _rspLen[RSP_SLOTS] = {};
    inline static volatile uint16_t _rspTag[RSP_SLOTS] = {};
    inline static volatile uint32_t _rspWrite = 0;
    uint32_t _rspRead = 0;
};
