#pragma once

#include <Adafruit_TinyUSB.h>

// ==============================
// Switch Pro Controller emulation (the fourth USB identity for dongle mode)
//
// The only identity a Switch / Switch 2 accepts gyro + rumble through: the console
// speaks the Pro Controller's proprietary USB protocol (VID:PID 057E:2009), so this
// implements that protocol rather than plain HID. Everything follows dekuNukem's
// Nintendo_Switch_Reverse_Engineering notes:
//   - 0x80-prefixed USB commands (handshake; replies are input report 0x81)
//   - output 0x01 = rumble + subcommand, replied with input report 0x21
//   - output 0x10 = rumble only
//   - input 0x30 = full state + 3 IMU frames, streamed every 8ms once enabled
//   - subcommand 0x10 reads "SPI flash": served from a canned table (factory
//     calibration, colors, no user calibration), chosen so the Pro Controller 2's
//     raw values pass through with minimal conversion (sticks and accel 1:1,
//     gyro x16.4/14.286)
// NOTE: on a Switch 1 the console setting "Pro Controller Wired Communication"
// must be ON, or USB controllers are ignored.
// ==============================

// Input state the dongle puts into a Pro Controller report
// (buttons already in ProCon bit layout, sticks raw 12-bit, IMU in ProCon scale)
struct ProconState {
    uint8_t buttons[3];        // [0] right (Y/X/B/A/R/ZR), [1] shared, [2] left (dpad/L/ZL)
    uint16_t lx, ly, rx, ry;   // 0-4095, up and right are larger (same as the source)
    int16_t accel[3];          // 4096 LSB/g (same as the Pro Controller 2 raw)
    int16_t gyro[3];           // 16.4 LSB/dps (converted from the Pro Controller 2 raw)
};

class ProconUsb {
public:
    static constexpr uint16_t USB_VID = 0x057E;
    static constexpr uint16_t USB_PID = 0x2009;

    // The Pro Controller's own report descriptor (public knowledge via dekuNukem's
    // reverse-engineering docs). Input 0x30/0x21/0x81, output 0x01/0x10/0x80/0x82.
    static constexpr uint8_t REPORT_DESCRIPTOR[] = {
        0x05, 0x01,                    // Usage Page (Generic Desktop)
        0x15, 0x00,                    // Logical Minimum (0)
        0x09, 0x04,                    // Usage (Joystick)
        0xA1, 0x01,                    // Collection (Application)
        0x85, 0x30,                    //   Report ID (0x30)
        0x05, 0x01,
        0x05, 0x09,                    //   Buttons 1-10
        0x19, 0x01, 0x29, 0x0A,
        0x15, 0x00, 0x25, 0x01,
        0x75, 0x01, 0x95, 0x0A,
        0x55, 0x00, 0x65, 0x00,
        0x81, 0x02,
        0x05, 0x09,                    //   Buttons 11-14
        0x19, 0x0B, 0x29, 0x0E,
        0x15, 0x00, 0x25, 0x01,
        0x75, 0x01, 0x95, 0x04,
        0x81, 0x02,
        0x75, 0x01, 0x95, 0x02,        //   2 bits padding
        0x81, 0x03,
        0x0B, 0x01, 0x00, 0x01, 0x00,  //   Usage (0x010001)
        0xA1, 0x00,                    //   Collection (Physical)
        0x0B, 0x30, 0x00, 0x01, 0x00,  //     X
        0x0B, 0x31, 0x00, 0x01, 0x00,  //     Y
        0x0B, 0x32, 0x00, 0x01, 0x00,  //     Z
        0x0B, 0x35, 0x00, 0x01, 0x00,  //     Rz
        0x15, 0x00,
        0x27, 0xFF, 0xFF, 0x00, 0x00,
        0x75, 0x10, 0x95, 0x04,
        0x81, 0x02,
        0xC0,                          //   End Collection
        0x0B, 0x39, 0x00, 0x01, 0x00,  //   Hat switch
        0x15, 0x00, 0x25, 0x07,
        0x35, 0x00, 0x46, 0x3B, 0x01,
        0x65, 0x14,
        0x75, 0x04, 0x95, 0x01,
        0x81, 0x02,
        0x05, 0x09,                    //   Buttons 15-18
        0x19, 0x0F, 0x29, 0x12,
        0x15, 0x00, 0x25, 0x01,
        0x75, 0x01, 0x95, 0x04,
        0x81, 0x02,
        0x75, 0x08, 0x95, 0x34,        //   52 bytes padding
        0x81, 0x03,
        0x06, 0x00, 0xFF,              //   Vendor page
        0x85, 0x21,                    //   Report ID (0x21) input
        0x09, 0x01, 0x75, 0x08, 0x95, 0x3F, 0x81, 0x03,
        0x85, 0x81,                    //   Report ID (0x81) input
        0x09, 0x02, 0x75, 0x08, 0x95, 0x3F, 0x81, 0x03,
        0x85, 0x01,                    //   Report ID (0x01) output
        0x09, 0x03, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
        0x85, 0x10,                    //   Report ID (0x10) output
        0x09, 0x04, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
        0x85, 0x80,                    //   Report ID (0x80) output
        0x09, 0x05, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
        0x85, 0x82,                    //   Report ID (0x82) output
        0x09, 0x06, 0x75, 0x08, 0x95, 0x3F, 0x91, 0x83,
        0xC0                           // End Collection
    };

    ProconUsb() : _hid(REPORT_DESCRIPTOR, sizeof(REPORT_DESCRIPTOR),
                       HID_ITF_PROTOCOL_NONE, 8 /*ms*/, true /*with OUT EP*/) {}

    bool begin() {
        _instance = this;

        USBDevice.setID(USB_VID, USB_PID);
        USBDevice.setDeviceVersion(0x0200);   // bcdDevice of the real Pro Controller
        // The console identifies a Pro Controller by VID/PID, so the manufacturer
        // string stays neutral (same policy as the DS4 identity)
        USBDevice.setManufacturerDescriptor("Karakuri");
        USBDevice.setProductDescriptor("Pro Controller");
        buildMac();

        setNeutralState(_state);

        _hid.setReportCallback(getReportStatic, setReportStatic);
        return _hid.begin();
    }

    bool ready() { return _hid.ready(); }

    void setState(const ProconState &s) { _state = s; }

    void setNeutral() { setNeutralState(_state); }

    // Handshake replies, subcommand replies and the 0x30 stream.
    // Call every iteration from loop.
    void task() {
        // USB link-state transitions, plus a 1s heartbeat with the 0x30 send count
        // while streaming: together they show how long the console kept the stream
        // alive before giving up (or whether it suspended the bus)
        {
            bool m = USBDevice.mounted(), su = USBDevice.suspended();
            if (m != _lastMounted || su != _lastSuspended) {
                _lastMounted = m;
                _lastSuspended = su;
                uint8_t d[2] = { (uint8_t)m, (uint8_t)su };
                trace(4, d, sizeof(d));
            }
            if (_streaming && (int32_t)(millis() - _lastHbMs) >= 5000) {
                _lastHbMs = millis();
                uint8_t d[4] = { (uint8_t)(_txCount & 0xFF), (uint8_t)(_txCount >> 8),
                                 (uint8_t)(_txCount >> 16), (uint8_t)(_txCount >> 24) };
                trace(5, d, sizeof(d));
            }
        }

        // Drain queued OUT packets first (a subcommand may enable streaming)
        while (_rxRead != _rxWrite) {
            uint8_t pkt[64];
            memcpy(pkt, (const void *)_rx[_rxRead % RX_SLOTS], 64);
            _rxRead++;
            handlePacket(pkt);
        }

        if (!_hid.ready()) return;

        if (_pending81) {
            _pending81 = false;
            trace(1, _reply81, 10);
            _hid.sendReport(0x81, _reply81, sizeof(_reply81));
            return;
        }
        if (_pending21) {
            _pending21 = false;
            uint8_t p[63];
            buildInputPrefix(p);
            p[12] = _ack;
            p[13] = _ackSubcmd;
            memcpy(&p[14], _ackData, sizeof(_ackData));
            memset(&p[14 + sizeof(_ackData)], 0, sizeof(p) - 14 - sizeof(_ackData));
            trace(2, &p[12], 12);   // ack, subcmd, first reply bytes
            _hid.sendReport(0x21, p, sizeof(p));
            return;
        }

        if (!_streaming) return;
        uint32_t nowUs = micros();
        if ((int32_t)(nowUs - _nextSendUs) < 0) return;
        _nextSendUs = nowUs + 8000;

        uint8_t p[63];
        buildInputPrefix(p);
        // 3 IMU frames (0/5/10ms). We sample slower than that, so all three carry
        // the latest values; the console just sees a flat 5ms window
        for (int f = 0; f < 3; f++) {
            int off = 12 + f * 12;
            for (int i = 0; i < 3; i++) writeS16(&p[off + i * 2], _state.accel[i]);
            for (int i = 0; i < 3; i++) writeS16(&p[off + 6 + i * 2], _state.gyro[i]);
        }
        memset(&p[48], 0, sizeof(p) - 48);
        if (_hid.sendReport(0x30, p, sizeof(p))) _txCount++;
    }

    // Rumble request from the console. Returns true and hands over the values if they changed
    bool takeRumble(uint8_t &left, uint8_t &right) {
        if (!_rumbleDirty) return false;
        _rumbleDirty = false;
        left = _rumbleLeft;
        right = _rumbleRight;
        return true;
    }

    bool rumbleActive() const { return _rumbleLeft > 0 || _rumbleRight > 0; }

    // ---- Diagnostic trace (the PTRACE command) ----
    // Everything the console sends plus every reply we make, so a failed init
    // against a real console can be read back later (the .ino persists these
    // entries to LittleFS; while plugged into a console nothing can read serial)
    struct TraceEntry {
        uint32_t ms;
        uint8_t dir;      // 0=rx packet, 1=tx 0x81, 2=tx 0x21, 3=control GET_REPORT
        uint8_t len;
        uint8_t data[16];
    };

    bool takeTrace(TraceEntry &e) {
        if (_trRead == _trWrite) return false;
        e = _trace[_trRead % TRACE_SLOTS];
        _trRead++;
        return true;
    }

private:
    static constexpr uint8_t RX_SLOTS = 4;
    static constexpr uint8_t TRACE_SLOTS = 64;

    void trace(uint8_t dir, const uint8_t *d, uint8_t n) {
        TraceEntry &e = _trace[_trWrite % TRACE_SLOTS];
        e.ms = millis();
        e.dir = dir;
        if (n > sizeof(e.data)) n = sizeof(e.data);
        e.len = n;
        memcpy(e.data, d, n);
        _trWrite++;
        if ((uint8_t)(_trWrite - _trRead) > TRACE_SLOTS) {
            _trRead = _trWrite - TRACE_SLOTS;   // overflow: drop oldest
        }
    }

    static void writeS16(uint8_t *p, int16_t v) {
        p[0] = (uint8_t)(v & 0xFF);
        p[1] = (uint8_t)(((uint16_t)v) >> 8);
    }

    static void setNeutralState(ProconState &s) {
        memset(&s, 0, sizeof(s));
        s.lx = s.ly = s.rx = s.ry = 2048;
    }

    void buildMac() {
        const char *id = rp2040.getChipID();
        size_t len = strlen(id);
        memset(_mac, 0, sizeof(_mac));
        int bi = 5;
        for (int i = (int)len - 1; i >= 0 && bi >= 0; i -= 2) {
            char hex[3] = { (i >= 1) ? id[i - 1] : '0', id[i], 0 };
            _mac[bi--] = (uint8_t)strtoul(hex, nullptr, 16);
        }
        // Locally administered, and never multicast — an I/G bit inherited from the
        // chip ID would make this an invalid device address
        _mac[0] = (uint8_t)((_mac[0] | 0x02) & ~0x01);
    }

    // [0]=timer [1]=battery/conn [2-4]=buttons [5-7]=left stick [8-10]=right stick [11]=vib
    void buildInputPrefix(uint8_t *p) {
        p[0] = _timer++;
        p[1] = 0x91;   // battery full + charging, connection: Pro Controller on USB
        p[2] = _state.buttons[0];
        p[3] = _state.buttons[1];
        p[4] = _state.buttons[2];
        pack12(&p[5], _state.lx, _state.ly);
        pack12(&p[8], _state.rx, _state.ry);
        p[11] = 0x00;
    }

    static void pack12(uint8_t *p, uint16_t x, uint16_t y) {
        p[0] = (uint8_t)(x & 0xFF);
        p[1] = (uint8_t)(((x >> 8) & 0x0F) | ((y & 0x0F) << 4));
        p[2] = (uint8_t)(y >> 4);
    }

    // ---- OUT packets (queued in the callback, handled in task) ----

    static void setReportStatic(uint8_t report_id, hid_report_type_t report_type,
                                uint8_t const *buffer, uint16_t bufsize) {
        (void)report_type;
        if (!_instance || !buffer || bufsize == 0) return;
        ProconUsb *self = _instance;

        uint8_t slot = self->_rxWrite % RX_SLOTS;
        uint8_t *dst = (uint8_t *)self->_rx[slot];
        memset(dst, 0, 64);
        // Over the OUT EP report_id=0 and the ID is the first buffer byte; via
        // SET_REPORT the ID arrives separately, so restore it to the front
        if (report_id == 0) {
            memcpy(dst, buffer, (bufsize < 64) ? bufsize : 64);
        } else {
            dst[0] = report_id;
            memcpy(dst + 1, buffer, (bufsize < 63) ? bufsize : 63);
        }
        self->trace(0, dst, 16);
        self->_rxWrite++;
        if ((uint8_t)(self->_rxWrite - self->_rxRead) > RX_SLOTS) {
            self->_rxRead = self->_rxWrite - RX_SLOTS;   // overflow: drop oldest
        }
    }

    // GET_REPORT on the control endpoint: same STALL as before (return 0), but
    // recorded — a console poking reports we do not serve is exactly the kind of
    // init-sequence difference the trace exists to catch
    static uint16_t getReportStatic(uint8_t report_id, hid_report_type_t report_type,
                                    uint8_t *buffer, uint16_t reqlen) {
        (void)buffer;
        if (_instance) {
            uint8_t d[4] = { report_id, (uint8_t)report_type,
                             (uint8_t)(reqlen & 0xFF), (uint8_t)(reqlen >> 8) };
            _instance->trace(3, d, sizeof(d));
        }
        return 0;
    }

    void handlePacket(const uint8_t *pkt) {
        switch (pkt[0]) {
            case 0x80:   // USB handshake command; reply as input report 0x81
                switch (pkt[1]) {
                    case 0x01:   // status: subtype (03 = Pro Controller) + MAC
                        memset(_reply81, 0, sizeof(_reply81));
                        _reply81[0] = 0x01;
                        _reply81[1] = 0x00;
                        _reply81[2] = 0x03;
                        memcpy(&_reply81[3], _mac, 6);
                        _pending81 = true;
                        break;
                    case 0x02:   // handshake
                    case 0x03:   // baudrate (meaningless on real USB, ACK anyway)
                        memset(_reply81, 0, sizeof(_reply81));
                        _reply81[0] = pkt[1];
                        _pending81 = true;
                        break;
                    case 0x04:   // force USB HID mode: start streaming, no reply
                        _streaming = true;
                        break;
                    case 0x05:   // allow timeout/BT again
                        _streaming = false;
                        break;
                    default:
                        break;
                }
                break;

            case 0x01:   // rumble + subcommand
                decodeRumble(&pkt[2]);
                handleSubcommand(pkt[10], &pkt[11]);
                break;

            case 0x10:   // rumble only
                decodeRumble(&pkt[2]);
                break;

            default:
                break;
        }
    }

    void handleSubcommand(uint8_t subcmd, const uint8_t *args) {
        _ackSubcmd = subcmd;
        memset(_ackData, 0, sizeof(_ackData));
        _ack = 0x80;   // generic ACK unless the subcommand carries data

        switch (subcmd) {
            case 0x02:   // device info
                _ack = 0x82;
                _ackData[0] = 0x03;   // firmware 3.72
                _ackData[1] = 0x48;
                _ackData[2] = 0x03;   // Pro Controller
                _ackData[3] = 0x02;
                memcpy(&_ackData[4], _mac, 6);
                _ackData[10] = 0x01;
                _ackData[11] = 0x01;  // use colors from "SPI"
                break;

            case 0x01: {  // Bluetooth manual pairing: the console runs this over USB
                          // too, and a contentless ACK makes a Switch 2 reset the
                          // bus and abandon the controller (error 2162-0002)
                _ack = 0x81;
                uint8_t step = args[0];
                _ackData[0] = step;
                if (step == 0x01) {
                    // reply: our BT MAC in little-endian (host MAC arrived in args[1..6])
                    for (int i = 0; i < 6; i++) _ackData[1 + i] = _mac[5 - i];
                } else if (step == 0x02) {
                    // Long Term Key, each byte XORed with 0xAA. Any consistent key
                    // works — the BLE side of this dongle never uses it
                    for (int i = 0; i < 16; i++) _ackData[1 + i] = 0xAA;
                }
                // step 0x03 = "save pairing info": the step echo alone is the reply
                break;
            }

            case 0x03:   // set input report mode
                if (args[0] == 0x30) _streaming = true;
                break;

            case 0x04:   // trigger buttons elapsed time (zeros)
                _ack = 0x83;
                break;

            case 0x10: {  // SPI flash read
                uint32_t addr = (uint32_t)args[0] | ((uint32_t)args[1] << 8) |
                                ((uint32_t)args[2] << 16) | ((uint32_t)args[3] << 24);
                uint8_t len = args[4];
                if (len > 0x1D) len = 0x1D;
                _ack = 0x90;
                memcpy(_ackData, args, 5);   // echo address + length
                for (uint8_t i = 0; i < len; i++) {
                    _ackData[5 + i] = spiRead(addr + i);
                }
                break;
            }

            case 0x21:   // NFC/IR MCU configuration: canned "MCU in standby" state
                _ack = 0xA0;
                _ackData[0] = 0x01;
                _ackData[1] = 0x00;
                _ackData[2] = 0xFF;
                _ackData[3] = 0x00;
                _ackData[4] = 0x08;
                _ackData[5] = 0x00;
                _ackData[6] = 0x1B;
                _ackData[7] = 0x01;
                break;

            // 0x00 nop / 0x08 low power / 0x22 MCU state /
            // 0x30 player lights / 0x38 HOME light / 0x40 IMU / 0x41 IMU config /
            // 0x48 vibration: all fine with the generic ACK
            default:
                break;
        }
        _pending21 = true;
    }

    // ---- The "SPI flash" the console reads calibration and colors from ----
    // Values chosen so raw Pro Controller 2 sticks (center 2048) and our converted
    // IMU pass through: stick center 2048 / range 1400, standard IMU coefficients
    // (accel 0x4000 = 4096 LSB/g, gyro 0x343B = 16.4 LSB/dps), no user calibration.
    uint8_t spiRead(uint32_t addr) {
        // Factory IMU calibration @6020: offsets 0, standard sensitivities
        static const uint8_t IMU_CAL[24] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // accel origin
            0x00, 0x40, 0x00, 0x40, 0x00, 0x40,   // accel sensitivity 0x4000
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // gyro origin
            0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34,   // gyro sensitivity 0x343B
        };
        // Left stick @603D: X/Y max above center, X/Y center, X/Y min below center
        // Right stick @6046: X/Y center, X/Y min below, X/Y max above (order differs)
        // All packed 12-bit pairs: above/below = 1400, center = 2048
        static const uint8_t STICK_CAL_L[9] = {
            0x78, 0x85, 0x57,   // 1400, 1400
            0x00, 0x08, 0x80,   // 2048, 2048
            0x78, 0x85, 0x57,   // 1400, 1400
        };
        static const uint8_t STICK_CAL_R[9] = {
            0x00, 0x08, 0x80,   // 2048, 2048
            0x78, 0x85, 0x57,   // 1400, 1400
            0x78, 0x85, 0x57,   // 1400, 1400
        };
        // Colors @6050: charcoal body, white buttons, charcoal grips
        static const uint8_t COLORS[12] = {
            0x32, 0x32, 0x32, 0xFF, 0xFF, 0xFF,
            0x32, 0x32, 0x32, 0x32, 0x32, 0x32,
        };
        // Stick device parameters @6080+ (typical Pro Controller values; the first
        // 6 bytes @6080 are the 6-axis horizontal offset)
        static const uint8_t STICK_PARAMS[18] = {
            0x0F, 0x30, 0x61, 0xAE, 0x90, 0xD9,
            0xD4, 0x14, 0x54, 0x41, 0x15, 0x54,
            0xC7, 0x79, 0x9C, 0x33, 0x36, 0x63,
        };
        static const uint8_t HORIZONTAL_OFFSET[6] = {
            0x50, 0xFD, 0x00, 0x00, 0xC6, 0x0F,
        };

        // Shipment flag @5000: 0x01 = factory-fresh, 0x00 = initialized by a console.
        // 0xFF (erased) is a state no real controller is in — serve "initialized"
        if (addr == 0x5000) return 0x00;
        if (addr >= 0x6020 && addr < 0x6020 + 24) return IMU_CAL[addr - 0x6020];
        if (addr >= 0x603D && addr < 0x603D + 9) return STICK_CAL_L[addr - 0x603D];
        if (addr >= 0x6046 && addr < 0x6046 + 9) return STICK_CAL_R[addr - 0x6046];
        if (addr >= 0x6050 && addr < 0x6050 + 12) return COLORS[addr - 0x6050];
        if (addr >= 0x6080 && addr < 0x6080 + 6) return HORIZONTAL_OFFSET[addr - 0x6080];
        if (addr >= 0x6086 && addr < 0x6086 + 18) return STICK_PARAMS[addr - 0x6086];
        if (addr >= 0x6098 && addr < 0x6098 + 18) return STICK_PARAMS[addr - 0x6098];
        // Serial number @6000 (0xFF = none), user calibration @8010/@8026
        // (0xFF = not present), and anything unknown: erased flash
        return 0xFF;
    }

    // HD rumble decode, amplitude only: [hf][hf amp][lf][lf amp] per side,
    // handed to the BLE relay as a perceptual level 0-255 (the Switch amplitude
    // index scale, 255 = index 100). No linear conversion: the Pro Controller 2
    // amplitude field turned out to be the same index scale x4, so relaying the
    // index preserves the strength the console asked for.
    void decodeRumble(const uint8_t *d) {
        uint8_t l = decodeSideAmp(&d[0]);
        uint8_t r = decodeSideAmp(&d[4]);
        if (l != _rumbleLeft || r != _rumbleRight) {
            _rumbleLeft = l;
            _rumbleRight = r;
            _rumbleDirty = true;
        }
    }

    static uint8_t decodeSideAmp(const uint8_t *d) {
        // Bit 0 of the HF amplitude byte is the frequency MSB, not amplitude:
        // the neutral pattern 00 01 40 40 must decode to "off"
        uint8_t hfIdx = (uint8_t)((d[1] & 0xFE) >> 1);   // 0-100
        uint8_t lfRaw = (uint8_t)(d[3] & 0x7F);          // 0x40 + idx/2
        uint8_t lfIdx = (lfRaw > 0x40) ? (uint8_t)((lfRaw - 0x40) * 2) : 0;
        uint8_t idx = (hfIdx > lfIdx) ? hfIdx : lfIdx;
        if (idx > 100) idx = 100;
        return (uint8_t)((idx * 255 + 50) / 100);
    }

    Adafruit_USBD_HID _hid;
    ProconState _state = {};
    uint8_t _mac[6] = {};
    uint8_t _timer = 0;
    uint32_t _nextSendUs = 0;
    bool _streaming = false;

    // OUT packet queue (filled in USB context, drained in task)
    volatile uint8_t _rx[RX_SLOTS][64] = {};
    volatile uint8_t _rxRead = 0;
    volatile uint8_t _rxWrite = 0;

    // Trace ring (written in USB and task context, drained in loop)
    TraceEntry _trace[TRACE_SLOTS] = {};
    volatile uint8_t _trRead = 0;
    volatile uint8_t _trWrite = 0;

    // Link-state / heartbeat bookkeeping for the trace
    bool _lastMounted = false;
    bool _lastSuspended = false;
    uint32_t _lastHbMs = 0;
    uint32_t _txCount = 0;

    // Pending replies (single slot each: the console waits between commands)
    bool _pending81 = false;
    uint8_t _reply81[63] = {};
    bool _pending21 = false;
    uint8_t _ack = 0x80;
    uint8_t _ackSubcmd = 0;
    uint8_t _ackData[35] = {};

    volatile bool _rumbleDirty = false;
    volatile uint8_t _rumbleLeft = 0;
    volatile uint8_t _rumbleRight = 0;

    inline static ProconUsb *_instance = nullptr;
};
