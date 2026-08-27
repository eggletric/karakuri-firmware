#pragma once

#include <BLE.h>
#include <LocklessQueue.h>
#include <BluetoothLock.h>

// <btstack.h> (the umbrella header) does not build: hid_report_type_t from btstack_hid.h
// collides with TinyUSB. Declare only the gap functions we need here (keep the
// signatures in sync with pico-sdk/lib/btstack/src/gap.h). bluetooth_data_types.h is
// #define-only, so it is safe to include.
#include <bluetooth_data_types.h>
extern "C" {
    void gap_advertisements_set_data(uint8_t advertising_data_length, uint8_t *advertising_data);
    void gap_scan_response_set_data(uint8_t scan_response_data_length, uint8_t *scan_response_data);
}

// ==============================
// BLE link (the command path between the app and the Pico)
//
// Uses Nordic UART Service (NUS)-compatible UUIDs. The app finds the device by
// looking for this service UUID in the advertisement.
// Why not the core BLEServiceUART:
//   - its RX only accepts Write (with response), so Write Without Response,
//     which passthrough relies on heavily, is rejected at the ATT level
//   - we want to control the RX/TX buffer sizes and the TX pacing ourselves
//     (a bulk macro load pushes through tens of KB)
// ==============================
class BleLinkService : public BLEService, public BLECharacteristicCallbacks, public Print {
public:
    // NUS-compatible UUIDs (for reference):
    //   Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
    //   RX     : 6E400002-... / TX: 6E400003-...
    // BLEUUID's string constructor relies on sscanf's %llx, which fails to parse on real
    // hardware and yields an all-zero UUID (seen as the service UUID missing from the
    // advertisement). Always use the byte-array constructor.
    static constexpr uint8_t SERVICE_UUID_BYTES[16] = {
        0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
        0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E,
    };
    static constexpr uint8_t RX_UUID_BYTES[16] = {
        0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93,
        0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E,
    };
    static constexpr uint8_t TX_UUID_BYTES[16] = {
        0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93,
        0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E,
    };

    // Maximum bytes per notification. Keep it well below the effective MTU on macOS
    // (185 or more): BTstack silently truncates anything past MTU-3.
    static constexpr size_t TX_CHUNK = 100;
    // Minimum interval between notifications. BLECharacteristic::setValue overwrites the
    // value, so back-to-back calls can drop the previous notification before it goes
    // out. Send one chunk at a time, spaced wider than the connection interval.
    static constexpr uint32_t TX_INTERVAL_MS = 20;

    BleLinkService(int rxbuff = 4096, int txbuff = 1024)
        : BLEService(BLEUUID(SERVICE_UUID_BYTES)) {
        _rx = new BLECharacteristic(BLEUUID(RX_UUID_BYTES),
                                    BLEWrite | BLEWriteWithoutResponse, "Karakuri RX");
        _rx->setCallbacks(this);
        _tx = new BLECharacteristic(BLEUUID(TX_UUID_BYTES), BLERead | BLENotify, "Karakuri TX");
        addCharacteristic(_rx);
        addCharacteristic(_tx);
        _rxQueue = new LocklessQueue<uint8_t>(rxbuff);
        _txQueue = new LocklessQueue<uint8_t>(txbuff);
    }

    bool connected() { return con_handle != 0; }

    // ---- Receive (app -> Pico) ----
    int available() { return _rxQueue->available(); }

    int read() {
        uint8_t b;
        return _rxQueue->read(&b) ? (int)b : -1;
    }

    // Whether the RX queue overflowed and dropped data (reading clears the flag)
    bool overflow() {
        bool o = _overflow;
        _overflow = false;
        return o;
    }

    // ---- Send (Pico -> app). Print lets callers use client->println() ----
    size_t write(uint8_t c) override {
        if (!_txQueue->write(c)) {
            _txOverflow = true;
            return 0;
        }
        return 1;
    }

    size_t write(const uint8_t *buf, size_t len) override {
        size_t n = 0;
        while (n < len && _txQueue->write(buf[n])) n++;
        if (n < len) _txOverflow = true;
        return n;
    }

    // Whether the TX queue overflowed and dropped a response (reading clears the flag)
    bool txOverflow() {
        bool o = _txOverflow;
        _txOverflow = false;
        return o;
    }

    // Call every iteration from loop(). Flushes queued output one chunk per notify.
    void pump() {
        if (!connected()) {
            // Responses queued while disconnected have nowhere to go; drop them
            uint8_t b;
            while (_txQueue->read(&b)) {}
            return;
        }
        if (!_txQueue->available()) return;

        uint32_t now = millis();
        if ((int32_t)(now - _lastTx) < (int32_t)TX_INTERVAL_MS) return;

        uint8_t buf[TX_CHUNK];
        size_t len = 0;
        while (len < TX_CHUNK) {
            uint8_t b;
            if (!_txQueue->read(&b)) break;
            buf[len++] = b;
        }
        if (len) {
            _tx->setValue(buf, len);   // notified to the subscriber while connected
            _lastTx = now;
        }
    }

private:
    // Called from the BT stack's async context. Keep the work here light.
    void onWrite(BLECharacteristic *c) override {
        if (c != _rx) return;
        size_t len = c->valueLen();
        const uint8_t *data = (const uint8_t *)c->valueData();
        for (size_t i = 0; i < len; i++) {
            if (!_rxQueue->write(data[i])) _overflow = true;
        }
    }

    BLECharacteristic *_rx;
    BLECharacteristic *_tx;
    LocklessQueue<uint8_t> *_rxQueue;
    LocklessQueue<uint8_t> *_txQueue;
    volatile bool _overflow = false;
    bool _txOverflow = false;
    uint32_t _lastTx = 0;
};
