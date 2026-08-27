#pragma once

#include <Adafruit_TinyUSB.h>

// ==============================
// DualShock 4 emulation (the second USB identity for dongle mode)
//
// The identity that has reliably supported gyro + rumble in Steam / SDL for years.
// Built to satisfy SDL's hidapi PS4 driver (SDL_hidapi_ps4.c):
//   - VID:PID 054C:05C4 (DS4 v1) -> treated as official; USB always uses enhanced reports
//   - Feature 0x02: IMU calibration (35 bytes or more; all zeros makes SDL retry)
//   - Feature 0x12: serial (MAC)
//   - Input 0x01 (64B): axes / buttons / triggers / timestamp / gyro / accel / touch
//   - Output 0x05: rumble (right = weak [4], left = strong [5]) and LED
// A real DS4 streams continuously over USB, so we resend the last state every 4ms even
// while no BLE report has arrived (the timestamp advances in 5.33us units).
// ==============================

// Input state the dongle puts into a DS4 report (already converted from the Pro Controller 2)
struct DS4State {
    uint8_t lx, ly, rx, ry;    // 0-255 (Y is positive downward)
    uint8_t hat;               // 0=up ... 7=up-left, 8=neutral
    uint8_t face;              // [5] high nibble: Square 0x10 / Cross 0x20 / Circle 0x40 / Triangle 0x80
    uint8_t buttons2;          // [6]: L1/R1/L2/R2/Share/Options/L3/R3
    uint8_t buttons3;          // [7] low 2 bits: PS 0x01 / TouchpadClick 0x02
    uint8_t l2, r2;            // analog triggers (digital input here, so 0/255)
    int16_t gyro[3];           // DS4 units (16.384 LSB/dps)
    int16_t accel[3];          // DS4 units (8192 LSB/g)
};

class DS4Usb {
public:
    static constexpr uint16_t USB_VID = 0x054C;
    static constexpr uint16_t USB_PID = 0x05C4;

    // A HID report descriptor semantically equivalent to a real DS4. Input 0x01 = 63B
    // (4 axes + hat + 14 buttons + 6-bit counter + 2 triggers + 54B vendor),
    // output 0x05 = 31B, features 0x02/0xA3/0x12.
    static constexpr uint8_t REPORT_DESCRIPTOR[] = {
        0x05, 0x01,        // Usage Page (Generic Desktop)
        0x09, 0x05,        // Usage (Gamepad)
        0xA1, 0x01,        // Collection (Application)
        0x85, 0x01,        //   Report ID (1)
        0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35,   // X, Y, Z, Rz
        0x15, 0x00, 0x26, 0xFF, 0x00,
        0x75, 0x08, 0x95, 0x04,
        0x81, 0x02,
        0x09, 0x39,        //   Hat switch
        0x15, 0x00, 0x25, 0x07,
        0x35, 0x00, 0x46, 0x3B, 0x01,
        0x65, 0x14,
        0x75, 0x04, 0x95, 0x01,
        0x81, 0x42,
        0x65, 0x00,
        0x05, 0x09,        //   14 buttons
        0x19, 0x01, 0x29, 0x0E,
        0x15, 0x00, 0x25, 0x01,
        0x75, 0x01, 0x95, 0x0E,
        0x81, 0x02,
        0x06, 0x00, 0xFF,  //   Vendor: 6-bit counter
        0x09, 0x20,
        0x15, 0x00, 0x25, 0x3F,
        0x75, 0x06, 0x95, 0x01,
        0x81, 0x02,
        0x05, 0x01,        //   Rx, Ry (analog triggers)
        0x09, 0x33, 0x09, 0x34,
        0x15, 0x00, 0x26, 0xFF, 0x00,
        0x75, 0x08, 0x95, 0x02,
        0x81, 0x02,
        0x06, 0x00, 0xFF,  //   Vendor: remaining 54B (timestamp / IMU / touch / etc.)
        0x09, 0x21,
        0x15, 0x00, 0x26, 0xFF, 0x00,
        0x75, 0x08, 0x95, 0x36,
        0x81, 0x02,
        0x85, 0x05,        //   Output 0x05 (rumble/LED) 31B
        0x09, 0x22,
        0x95, 0x1F,
        0x91, 0x02,
        0x85, 0x02,        //   Feature 0x02 (calibration) 36B
        0x09, 0x24,
        0x95, 0x24,
        0xB1, 0x02,
        0x85, 0xA3,        //   Feature 0xA3 (manufacturing info) 48B
        0x09, 0x25,
        0x95, 0x30,
        0xB1, 0x02,
        0x85, 0x12,        //   Feature 0x12 (MAC) 15B
        0x09, 0x26,
        0x95, 0x0F,
        0xB1, 0x02,
        0xC0               // End Collection
    };

    DS4Usb() : _hid(REPORT_DESCRIPTOR, sizeof(REPORT_DESCRIPTOR),
                    HID_ITF_PROTOCOL_NONE, 1 /*ms*/, true /*with OUT EP*/) {}

    bool begin() {
        _instance = this;

        USBDevice.setID(USB_VID, USB_PID);
        // The host identifies a DS4 by VID/PID, so the manufacturer string stays neutral
        // (we do not claim Sony's trade name)
        USBDevice.setManufacturerDescriptor("Karakuri");
        USBDevice.setProductDescriptor("Wireless Controller");
        buildMac();

        // Neutral state
        _state.lx = _state.ly = _state.rx = _state.ry = 0x80;
        _state.hat = 8;

        _hid.setReportCallback(getReportStatic, setReportStatic);
        return _hid.begin();
    }

    bool ready() { return _hid.ready(); }

    void setState(const DS4State &s) { _state = s; }

    void setNeutral() {
        DS4State n = {};
        n.lx = n.ly = n.rx = n.ry = 0x80;
        n.hat = 8;
        _state = n;
    }

    // Stream continuously every 4ms like real hardware. Call every iteration from loop
    void task() {
        uint32_t nowUs = micros();
        if ((int32_t)(nowUs - _nextSendUs) < 0) return;
        _nextSendUs = nowUs + 4000;
        if (!_hid.ready()) return;

        uint8_t p[63] = {};
        p[0] = _state.lx;
        p[1] = _state.ly;
        p[2] = _state.rx;
        p[3] = _state.ry;
        p[4] = (uint8_t)((_state.hat & 0x0F) | _state.face);
        p[5] = _state.buttons2;
        p[6] = (uint8_t)((_state.buttons3 & 0x03) | (_counter << 2));
        _counter = (_counter + 1) & 0x3F;
        p[7] = _state.l2;
        p[8] = _state.r2;
        // Timestamp (units of 5.33us = 3/16 us)
        uint16_t ts = (uint16_t)((nowUs * 3) >> 4);
        p[9] = (uint8_t)(ts & 0xFF);
        p[10] = (uint8_t)(ts >> 8);
        p[11] = 0x0B;   // battery
        for (int i = 0; i < 3; i++) {
            writeS16(&p[12 + i * 2], _state.gyro[i]);
            writeS16(&p[18 + i * 2], _state.accel[i]);
        }
        p[29] = 0x1B;   // status (USB powered)
        // Touchpad: bit7 of a finger ID means "not touching". Set every candidate slot,
        // since interpretations differ
        p[32] = 0x80;
        p[34] = 0x80;
        p[36] = 0x80;
        p[38] = 0x80;
        _hid.sendReport(0x01, p, sizeof(p));
    }

    // Rumble request from the host. Returns true and hands over the values if they changed
    bool takeRumble(uint8_t &left, uint8_t &right) {
        if (!_rumbleDirty) return false;
        _rumbleDirty = false;
        left = _rumbleLeft;
        right = _rumbleRight;
        return true;
    }

    bool rumbleActive() const { return _rumbleLeft > 0 || _rumbleRight > 0; }

private:
    static void writeS16(uint8_t *p, int16_t v) {
        p[0] = (uint8_t)(v & 0xFF);
        p[1] = (uint8_t)(((uint16_t)v) >> 8);
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
        _mac[0] |= 0x02;   // locally administered address
    }

    // GET_REPORT (Feature). SDL disables gyro if it cannot read the calibration (0x02),
    // so always return valid values
    static uint16_t getReportStatic(uint8_t report_id, hid_report_type_t report_type,
                                    uint8_t *buffer, uint16_t reqlen) {
        (void)report_type;
        if (!_instance || !buffer) return 0;
        return _instance->getFeature(report_id, buffer, reqlen);
    }

    uint16_t getFeature(uint8_t id, uint8_t *b, uint16_t reqlen) {
        switch (id) {
            case 0x02: {
                // IMU calibration. Return values that put SDL's formulas
                //   gyro : dps/LSB = (speed+ + speed-) / (plus - minus)
                //   accel: LSB per 1g = (plus - minus) / 2
                // at stock-DS4 scales (16.384 LSB/dps, 8192 LSB/g). All biases are 0.
                if (reqlen < 36) return 0;
                memset(b, 0, 36);
                int off = 0;
                auto put = [&](int16_t v) {
                    b[off++] = (uint8_t)(v & 0xFF);
                    b[off++] = (uint8_t)(((uint16_t)v) >> 8);
                };
                put(0); put(0); put(0);              // pitch/yaw/roll bias
                put(8847); put(-8847);               // pitch plus/minus
                put(8847); put(-8847);               // yaw plus/minus
                put(8847); put(-8847);               // roll plus/minus
                put(540); put(540);                  // speed plus/minus
                put(8192); put(-8192);               // acc X plus/minus
                put(8192); put(-8192);               // acc Y plus/minus
                put(8192); put(-8192);               // acc Z plus/minus
                return 36;
            }
            case 0x12: {
                // MAC address
                if (reqlen < 15) return 0;
                memset(b, 0, 15);
                for (int i = 0; i < 6; i++) b[i] = _mac[5 - i];   // little-endian order
                b[6] = 0x08;
                b[7] = 0x25;
                return 15;
            }
            case 0xA3: {
                // Manufacturing info. Nothing reads it, so just make it well-formed
                if (reqlen < 48) return 0;
                memset(b, 0, 48);
                memcpy(b, "Sep 21 2015", 11);
                return 48;
            }
            default:
                return 0;
        }
    }

    // Output 0x05: [ID][flags][--][--][rumble weak(right)][rumble strong(left)][R][G][B]...
    // Over the OUT EP, report_id=0 and the ID lands at the head of the buffer
    static void setReportStatic(uint8_t report_id, hid_report_type_t report_type,
                                uint8_t const *buffer, uint16_t bufsize) {
        (void)report_type;
        if (!_instance || !buffer || bufsize < 1) return;

        const uint8_t *d = buffer;
        uint16_t n = bufsize;
        if (report_id == 0) {
            if (d[0] != 0x05) return;
        } else if (report_id == 0x05) {
            static uint8_t tmp[32];
            tmp[0] = 0x05;
            uint16_t c = (bufsize < 31) ? bufsize : 31;
            memcpy(&tmp[1], buffer, c);
            d = tmp;
            n = c + 1;
        } else {
            return;
        }
        if (n < 6) return;
        _instance->_rumbleRight = d[4];   // weak motor
        _instance->_rumbleLeft = d[5];    // strong motor
        _instance->_rumbleDirty = true;
    }

    Adafruit_USBD_HID _hid;
    DS4State _state = {};
    uint8_t _mac[6] = {};
    uint8_t _counter = 0;
    uint32_t _nextSendUs = 0;
    volatile bool _rumbleDirty = false;
    volatile uint8_t _rumbleLeft = 0;
    volatile uint8_t _rumbleRight = 0;

    inline static DS4Usb *_instance = nullptr;
};
