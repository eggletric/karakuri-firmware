#pragma once

#include <Adafruit_TinyUSB.h>

// ==============================
// SInput USB gamepad implementation (a USB identity for dongle mode)
//
// SInput is an open HID gamepad standard supported natively by SDL3.
// It carries gyro, accelerometer, rumble and 32 buttons over generic HID.
//   Spec / reference implementation: HandHeldLegend/SINPUT-LIB-HID (MIT-0)
//   SDL-side driver: libsdl-org/SDL src/joystick/hidapi/SDL_hidapi_sinput.c
//
// Report layout:
//   Input  0x01 (64B): power / 32 buttons / sticks / triggers / IMU
//   Input  0x02 (64B): command response (FEATURES = capability declaration)
//   Output 0x03 (48B): commands from the host (rumble / FEATURES request / LED)
// The VID/PID is the generic SInput fallback (0x2E8A:0x10C6).
// SDL recognizes that VID/PID as "SInput Generic".
// ==============================

// Input state the dongle puts into an SInput report (already converted from the Pro Controller 2)
struct SInputState {
    uint8_t buttons[4];    // SInput bit layout (see build below)
    int16_t lx, ly, rx, ry;
    uint32_t timestampUs;
    int16_t accel[3];
    int16_t gyro[3];
};

class SInputUsb {
public:
    // HID report descriptor (from HandHeldLegend/SINPUT-LIB-HID, MIT-0)
    static constexpr uint8_t REPORT_DESCRIPTOR[139] = {
        0x05, 0x01,        // Usage Page (Generic Desktop)
        0x09, 0x05,        // Usage (Gamepad)
        0xA1, 0x01,        // Collection (Application)
        // --- Input 0x01: plug/charge 2B ---
        0x85, 0x01,        //   Report ID (1)
        0x06, 0x00, 0xFF,  //   Usage Page (Vendor)
        0x09, 0x01,        //   Usage (Vendor 1)
        0x15, 0x00, 0x25, 0xFF,
        0x75, 0x08, 0x95, 0x02,
        0x81, 0x02,        //   Input
        // --- 32 buttons ---
        0x05, 0x09,        //   Usage Page (Button)
        0x19, 0x01, 0x29, 0x20,
        0x15, 0x00, 0x25, 0x01,
        0x75, 0x01, 0x95, 0x20,
        0x81, 0x02,        //   Input
        // --- sticks + triggers (6 x s16) ---
        // The axis usages are X,Y,Z,Rx + Slider/Dial (triggers), changed from the
        // reference implementation's Z,Rz + Rx,Ry. Generic-HID hosts order axes by
        // declaration order, by usage number, or by fixed usage slot (X,Y,Z,Rx,...);
        // this assignment puts lx,ly,rx,ry first under all three (a fixed-slot host was
        // seen on real hardware). SDL's SInput driver reads by VID/PID and offset anyway.
        0x05, 0x01,        //   Usage Page (Generic Desktop)
        0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x33, 0x09, 0x36, 0x09, 0x37,
        0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F,
        0x75, 0x10, 0x95, 0x06,
        0x81, 0x02,        //   Input
        // --- IMU timestamp (u32) ---
        0x06, 0x00, 0xFF,  //   Usage Page (Vendor)
        0x09, 0x20,
        0x15, 0x00, 0x26, 0xFF, 0xFF,
        0x75, 0x20, 0x95, 0x01,
        0x81, 0x02,        //   Input
        // --- accel XYZ + gyro XYZ (6 x s16) ---
        0x09, 0x21,
        0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F,
        0x75, 0x10, 0x95, 0x06,
        0x81, 0x02,        //   Input
        // --- reserved 29B ---
        0x09, 0x22,
        0x15, 0x00, 0x26, 0xFF, 0x00,
        0x75, 0x08, 0x95, 0x1D,
        0x81, 0x02,        //   Input
        // --- Input 0x02: command reply 63B ---
        0x85, 0x02,
        0x09, 0x23,
        0x15, 0x00, 0x26, 0xFF, 0x00,
        0x75, 0x08, 0x95, 0x3F,
        0x81, 0x02,        //   Input
        // --- Output 0x03: command 47B ---
        0x85, 0x03,
        0x09, 0x24,
        0x15, 0x00, 0x26, 0xFF, 0x00,
        0x75, 0x08, 0x95, 0x2F,
        0x91, 0x02,        //   Output
        0xC0               // End Collection
    };

    static constexpr uint16_t USB_VID = 0x2E8A;   // Raspberry Pi
    static constexpr uint16_t USB_PID = 0x10C6;   // SInput Generic

    // SInput protocol command IDs (byte [1] of Output 0x03)
    static constexpr uint8_t CMD_HAPTIC   = 0x01;
    static constexpr uint8_t CMD_FEATURES = 0x02;

    SInputUsb() : _hid(REPORT_DESCRIPTOR, sizeof(REPORT_DESCRIPTOR),
                       HID_ITF_PROTOCOL_NONE, 1 /*ms*/, true /*with OUT EP*/) {}

    bool begin() {
        _instance = this;

        USBDevice.setID(USB_VID, USB_PID);
        USBDevice.setManufacturerDescriptor("Karakuri");
        USBDevice.setProductDescriptor("Karakuri SInput Dongle");
        // The serial SDL uses for its GUID; the tail of the chip ID distinguishes devices
        USBDevice.setSerialDescriptor(rp2040.getChipID());
        buildSerialBytes();

        _hid.setReportCallback(nullptr, setReportStatic);
        return _hid.begin();
    }

    bool ready() { return _hid.ready(); }

    // Sends the response to a FEATURES request. Call every iteration from loop
    void task() {
        if (!_featureRequested || !_hid.ready()) return;

        uint8_t p[63] = {};
        p[0] = CMD_FEATURES;
        p[1] = 0x01; p[2] = 0x00;             // protocol version 1
        // ff1: rumble | accel | gyro | left and right sticks (no analog triggers)
        p[3] = 0x01 | 0x04 | 0x08 | 0x10 | 0x20;
        p[4] = 0x00;                          // ff2
        p[5] = 7;                             // gamepad_type: NINTENDO_PRO
        p[6] = (uint8_t)(3 << 5);             // face style: BAYX (Nintendo)
        p[7] = 0x40; p[8] = 0x1F;             // polling 8000us (125Hz)
        p[9] = 8; p[10] = 0;                  // accel range ±8g (4096 LSB/g)
        p[11] = (uint8_t)(2294 & 0xFF);       // gyro range (32768 / 14.286 LSB/dps)
        p[12] = (uint8_t)(2294 >> 8);
        p[13] = 0xFF;                         // mask0: 4 face buttons + D-Pad
        p[14] = 0xFF;                         // mask1: L3/R3 + L/R + ZL/ZR + GL/GR(paddle1)
        p[15] = 0x0F;                         // mask2: PLUS/MINUS + HOME + CAPTURE
        p[16] = 0x02;                         // mask3: misc_4 = C button
        p[17] = 0;                            // touchpads
        p[18] = 0;
        memcpy(&p[19], _serial, 6);
        // Only count the request as handled once it actually went out (retry next iteration)
        if (_hid.sendReport(0x02, p, sizeof(p))) _featureRequested = false;
    }

    // Sends an input report (0x01)
    bool sendInput(const SInputState &s) {
        if (!_hid.ready()) return false;

        uint8_t p[63] = {};
        p[0] = 1;      // plug_status: externally powered, no battery
        p[1] = 100;    // charge_percent
        memcpy(&p[2], s.buttons, 4);
        writeS16(&p[6], s.lx);
        writeS16(&p[8], s.ly);
        writeS16(&p[10], s.rx);
        writeS16(&p[12], s.ry);
        writeS16(&p[14], INT16_MIN);   // analog triggers not supported
        writeS16(&p[16], INT16_MIN);
        memcpy(&p[18], &s.timestampUs, 4);
        for (int i = 0; i < 3; i++) {
            writeS16(&p[22 + i * 2], s.accel[i]);
            writeS16(&p[28 + i * 2], s.gyro[i]);
        }
        return _hid.sendReport(0x01, p, sizeof(p));
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

    void buildSerialBytes() {
        // Turn the last 12 hex digits of the chip ID into 6 bytes
        const char *id = rp2040.getChipID();
        size_t len = strlen(id);
        memset(_serial, 0, sizeof(_serial));
        int bi = 5;
        for (int i = (int)len - 1; i >= 0 && bi >= 0; i -= 2) {
            char hex[3] = { (i >= 1) ? id[i - 1] : '0', id[i], 0 };
            _serial[bi--] = (uint8_t)strtoul(hex, nullptr, 16);
        }
    }

    // Both SET_REPORT (control transfer) and the OUT endpoint land here. Over the OUT EP,
    // report_id=0 and the report ID sits at the head of the buffer
    static void setReportStatic(uint8_t report_id, hid_report_type_t report_type,
                                uint8_t const *buffer, uint16_t bufsize) {
        (void)report_type;
        if (!_instance || !buffer || bufsize < 2) return;

        const uint8_t *d = buffer;
        uint16_t n = bufsize;
        if (report_id == 0) {
            if (d[0] != 0x03) return;
        } else if (report_id == 0x03) {
            // Control transfer: the buffer has no ID, so prepend one virtually
            static uint8_t tmp[48];
            tmp[0] = 0x03;
            uint16_t c = (bufsize < 47) ? bufsize : 47;
            memcpy(&tmp[1], buffer, c);
            d = tmp;
            n = c + 1;
        } else {
            return;
        }
        _instance->handleCommand(d, n);
    }

    void handleCommand(const uint8_t *d, uint16_t n) {
        if (n < 3) return;
        switch (d[1]) {
            case CMD_HAPTIC:
                // d[2] = type: 1 = frequency+amplitude (8 x u16), 2 = ERM (amplitude+brake)
                if (d[2] == 2 && n >= 7) {
                    _rumbleLeft = d[3];
                    _rumbleRight = d[5];
                    _rumbleDirty = true;
                } else if (d[2] == 1 && n >= 19) {
                    // For the frequency form, reduce the larger amplitude of each side to 0-255
                    uint16_t la1 = (uint16_t)(d[5] | (d[6] << 8));
                    uint16_t la2 = (uint16_t)(d[9] | (d[10] << 8));
                    uint16_t ra1 = (uint16_t)(d[13] | (d[14] << 8));
                    uint16_t ra2 = (uint16_t)(d[17] | (d[18] << 8));
                    uint16_t l = (la1 > la2) ? la1 : la2;
                    uint16_t r = (ra1 > ra2) ? ra1 : ra2;
                    _rumbleLeft = (uint8_t)(l > 0xFF ? (l >> 8) : (l ? 1 : 0));
                    _rumbleRight = (uint8_t)(r > 0xFF ? (r >> 8) : (r ? 1 : 0));
                    _rumbleDirty = true;
                }
                break;
            case CMD_FEATURES:
                _featureRequested = true;
                break;
            default:
                break;   // PLAYERLED / RGB are unsupported (and not declared in features)
        }
    }

    Adafruit_USBD_HID _hid;
    uint8_t _serial[6] = {};
    volatile bool _featureRequested = false;
    volatile bool _rumbleDirty = false;
    volatile uint8_t _rumbleLeft = 0;
    volatile uint8_t _rumbleRight = 0;

    inline static SInputUsb *_instance = nullptr;
};
