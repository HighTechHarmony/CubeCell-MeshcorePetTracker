/**
 * MeshCore Pet Tracker — Heltec CubeCell HTCC-AB02S (ASR6502)
 *
 * One-way deep-sleep GPS tracker. On each wake cycle:
 *   1. Power on Air530Z GPS and wait up to 60 s for a 2D/3D fix.
 *   2. On timeout (no fix): GPS off → deep sleep immediately.
 *   3. On fix: read battery voltage, build a MeshCore RAW_CUSTOM flood
 *      packet, transmit once via raw LoRa (no LoRaWAN, no ACK).
 *   4. Radio.Sleep() → GPS off → deep sleep for SLEEP_INTERVAL_MS.
 *
 * Radio:  SX1262 via LoRaWan_APP.h (native CubeCell Radio driver, no LoRaWAN)
 * GPS:    Air530Z via GPS_Air530Z.h
 * Sleep:  lowPowerHandler() — the correct high-level CubeCell sleep API.
 *         lowPowerHandler() internally calls CySysPmDeepSleep() (Cypress PSoC4)
 *         after safely disabling peripherals. It returns when any RTC interrupt
 *         fires (including our wakeup timer). This satisfies the requirement for
 *         CySysPmDeepSleep-based deep sleep per the ASR6502 SDK architecture.
 *         See: cores/asr650x/lora/system/low_power.c
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "GPS_Air530Z.h"

// GPS object — Air530Z is not a pre-declared singleton; must be instantiated here.
Air530ZClass GPS;

// Debug prints: can be overridden via build flags (e.g. -DDEBUG=0 for production)
#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#define DBG_PRINT(...)        \
    do { Serial.printf(__VA_ARGS__); } while (0)
#else
#define DBG_PRINT(...) do {} while (0)
#endif

// ─── Radio & Protocol Configuration ──────────────────────────────────────────

#define RF_FREQUENCY          915000000UL  // Hz — adjust for your region/band plan
#define TX_OUTPUT_POWER       22           // dBm (0–22)
#define LORA_BANDWIDTH        1            // 0=125 kHz | 1=250 kHz | 2=500 kHz
#define LORA_SPREADING_FACTOR 9
#define LORA_CODINGRATE       1            // 1=4/5 | 2=4/6 | 3=4/7 | 4=4/8
#define LORA_PREAMBLE_LENGTH  8            // symbols

// TODO: Verify this sync word against your MeshCore receiver node's
//       Radio.SetSyncWord() call. 0x12 = standard private LoRa network.
//       Common alternatives: 0x34 (p2p default), 0x13 (used in some stacks).
#define LORA_SYNC_WORD        0x12

// ─── Timing ───────────────────────────────────────────────────────────────────

#define SLEEP_INTERVAL_MS  300000UL   // 5-minute deep-sleep interval
#define GPS_TIMEOUT_MS      60000UL   // hard timeout waiting for GPS fix
#define TX_TIMEOUT_MS        5000UL   // radio TX watchdog (software)
// Beacon interval while waiting for GPS fix
#define BEACON_INTERVAL_MS   10000UL  // send a beacon every 10 seconds while acquiring

// Button / long-press configuration
// Optional: define BUTTON_PIN (e.g. via build flag `-DBUTTON_PIN=2`) to enable
// the on-board button long-press handler. When undefined, button code is
// omitted so the project builds without a board-specific pin mapping.
// Use the board variant symbol for the user button. USER_KEY maps to the
// on-board button in the CubeCell variant (e.g. P3_3). This avoids mismatches
// between Arduino numeric pin assumptions and the variant mapping.
#define BUTTON_PIN USER_KEY
#if defined(BUTTON_PIN)
#define BUTTON_AVAILABLE 1
#else
#define BUTTON_AVAILABLE 0
#endif
#define BUTTON_LONGPRESS_MS 3000UL
// Sampler parameters for diagnostics
#define SAMPLE_DURATION_MS 10000UL
#define SAMPLE_INTERVAL_MS 100U

// ─── MeshCore Wire-Format Constants ──────────────────────────────────────────
//
// Source: meshcore-dev/MeshCore — src/Packet.h + src/Packet.cpp
//
// Wire layout produced by Packet::writeTo() (no transport codes, empty path):
//   Byte 0:   header — bits[1:0]=route_type, bits[5:2]=payload_type, bits[7:6]=version
//   Byte 1:   path_len — 0x00 means no flood-path hashes accumulated yet
//   Byte 2+:  payload — entirely application-defined for PAYLOAD_TYPE_RAW_CUSTOM
//
// TODO: *** VERIFY ALL BYTE OFFSETS against the official MeshCore C++ library ***
//       https://github.com/meshcore-dev/MeshCore
//       Key files: src/Packet.h (class layout), src/Packet.cpp (writeTo / readFrom)
//       Confirm: route type encoding, path_len encoding, and that RAW_CUSTOM
//       payload bytes are passed through unmodified by the routing layer.

#define MC_ROUTE_TYPE_FLOOD        0x01  // ROUTE_TYPE_FLOOD from Packet.h
#define MC_PAYLOAD_TYPE_RAW_CUSTOM 0x0F  // PAYLOAD_TYPE_RAW_CUSTOM from Packet.h
#define MC_PAYLOAD_VER_1           0x00  // PAYLOAD_VER_1 from Packet.h

// header = (route[1:0]) | (type[3:0] << 2) | (ver[1:0] << 6)  →  0x3D
#define MC_HEADER_BYTE \
    ((MC_ROUTE_TYPE_FLOOD        & 0x03)       | \
     ((MC_PAYLOAD_TYPE_RAW_CUSTOM & 0x0F) << 2) | \
     ((MC_PAYLOAD_VER_1           & 0x03) << 6))

// ─── MeshCore Packet Struct ───────────────────────────────────────────────────

/**
 * Direct wire representation of a MeshCore ROUTE_TYPE_FLOOD /
 * PAYLOAD_TYPE_RAW_CUSTOM packet as serialised by Packet::writeTo().
 * (No transport codes, no accumulated flood path — originating node only.)
 *
 * mc_header and path_len mirror the MeshCore framing layer exactly.
 * All fields from sender_id onward are the RAW_CUSTOM application payload.
 * MeshCore imposes NO structure on RAW_CUSTOM bytes — the layout below is
 * entirely application-defined and must match whatever your receiver expects.
 *
 * TODO: *** VERIFY AGAINST official MeshCore library before network deployment ***
 *       meshcore-dev/MeshCore — src/Packet.h + src/Packet.cpp::writeTo()
 *       Confirm: MAX_HASH_SIZE (currently 8), broadcast dest_id convention,
 *       and that no additional framing bytes are inserted between fields.
 */
struct __attribute__((packed)) MeshCoreFrame {

    // ── MeshCore framing (2 bytes) ─────────────────────────────────────────
    uint8_t  mc_header;     // = MC_HEADER_BYTE (0x3D): flood | raw_custom | ver1
    uint8_t  path_len;      // = 0x00: originating node, no path accumulated yet

    // ── Application routing hints — inside RAW_CUSTOM payload (app-defined) ─
    // MeshCore uses 8-byte hashes (MAX_HASH_SIZE) as node identifiers.
    // These fields are PLACEHOLDERS. Populate sender_id with a stable,
    // unique 8-byte fingerprint for this tracker device (e.g. derived from
    // a hardware ID or a fixed constant unique to each unit).
    uint8_t  sender_id[8];  // Source node hash; fill with tracker's unique ID
    uint8_t  dest_id[8];    // Destination hash; all 0xFF = broadcast (app convention)
    uint8_t  flags;         // Application flags placeholder (0x00 = default)
    uint8_t  hop_limit;     // Hop-limit placeholder (typical MeshCore default: 3)

    // ── GPS & telemetry payload ────────────────────────────────────────────
    float    latitude;      // Decimal degrees, IEEE-754 single, little-endian (ARM native)
    float    longitude;     // Decimal degrees, IEEE-754 single, little-endian (ARM native)
    uint16_t battery_mv;    // Battery voltage in millivolts from getBatteryVoltage()
};

// Compile-time size check: 2 + 8 + 8 + 1 + 1 + 4 + 4 + 2 = 30 bytes
static_assert(sizeof(MeshCoreFrame) == 30, "MeshCoreFrame size mismatch — check struct padding");

// Default sender ID — replace per-device with a unique 8-byte identifier.
static const uint8_t SENDER_ID[8] = {
    0x50, 0x45, 0x54, 0x00,   // 'P','E','T', unit-class marker
    0x00, 0x00, 0x00, 0x01    // device instance — change per unit
};

// ─── Globals ──────────────────────────────────────────────────────────────────

static RadioEvents_t    RadioEvents;         // must outlive Radio.Init() call
static volatile bool    txDone    = false;
static TimerEvent_t     sleepTimer;
static volatile bool    wakeFlag  = false;   // set by sleep timer callback

// ─── Radio Event Callbacks ────────────────────────────────────────────────────

static void OnTxDone(void)    { txDone = true; DBG_PRINT("TX Done\r\n"); }
static void OnTxTimeout(void) { txDone = true; DBG_PRINT("TX Timeout\r\n"); }  // treat as completion; don't stall

// Required stubs — the CubeCell Radio driver may invoke these even in TX-only mode.
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    (void)payload; (void)size; (void)rssi; (void)snr;
}
static void OnRxTimeout(void) {}
static void OnRxError(void)   {}

// ─── Sleep Timer Callback ─────────────────────────────────────────────────────

static void onSleepTimer(void) {
    TimerStop(&sleepTimer);
    wakeFlag = true;
}

// ─── Helpers ──────────────────────────────────────────────────────────────────

static void vextOn(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);    // LOW = Vext rail enabled on CubeCell GPS board
}

static void vextOff(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, HIGH);   // HIGH = Vext rail disabled
}

/**
 * Set the RTC wakeup timer, then enter deep sleep via lowPowerHandler().
 * lowPowerHandler() calls CySysPmDeepSleep() internally and may return early
 * due to unrelated interrupts, so we loop until the timer callback fires.
 */
static void goToSleep(uint32_t ms) {
    wakeFlag = false;
    TimerInit(&sleepTimer, onSleepTimer);
    TimerSetValue(&sleepTimer, ms);
    TimerStart(&sleepTimer);
    while (!wakeFlag) {
        lowPowerHandler();  // internally calls CySysPmDeepSleep(); may return early
    }
}

// Helper: send a MeshCoreFrame and wait for completion (process radio IRQs)
static void sendMeshFrame(const MeshCoreFrame *frame) {
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetSyncWord(LORA_SYNC_WORD);
    Radio.SetTxConfig(
        MODEM_LORA,
        TX_OUTPUT_POWER,
        0,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        false,
        true,
        0,
        0,
        false,
        3000);

    txDone = false;
    Radio.Send((uint8_t *)frame, sizeof(*frame));
    uint32_t t0 = millis();
    while (!txDone && (millis() - t0) < TX_TIMEOUT_MS) {
        Radio.IrqProcess();
    }
}

// ─── Arduino Setup ────────────────────────────────────────────────────────────

void setup() {
    #if DEBUG
    Serial.begin(115200);
    delay(50);
    DBG_PRINT("Setup: Serial started\r\n");
    #endif
    // Configure on-board button input (active-low with internal pullup)
#if BUTTON_AVAILABLE
    pinMode(BUTTON_PIN, INPUT_PULLUP);
#else
    DBG_PRINT("Button feature disabled: define BUTTON_PIN to enable long-press advert\r\n");
#endif
    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone    = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError   = OnRxError;
    Radio.Init(&RadioEvents);
    Radio.Sleep();   // keep radio off until first TX; re-configured each cycle
    DBG_PRINT("Setup: Radio initialized and put to sleep\r\n");
}

// ─── Main Power Loop ──────────────────────────────────────────────────────────

void loop() {
    // Production: on-board button long-press advert (active-low)
#if BUTTON_AVAILABLE
    if (digitalRead(BUTTON_PIN) == LOW) {
        uint32_t t0 = millis();
        DBG_PRINT("Button pressed, measuring hold time...\r\n");
        while (digitalRead(BUTTON_PIN) == LOW) {
            if ((millis() - t0) >= BUTTON_LONGPRESS_MS) {
                uint32_t held = millis() - t0;
                DBG_PRINT("Button held %lu ms -> sending flood advert\r\n", held);
                MeshCoreFrame advert = {};
                advert.mc_header = MC_HEADER_BYTE;
                advert.path_len = 0x00;
                memcpy(advert.sender_id, SENDER_ID, sizeof(advert.sender_id));
                memset(advert.dest_id, 0xFF, sizeof(advert.dest_id));
                advert.flags = 0x02; // advert flag (application-defined)
                advert.hop_limit = 3;
                advert.latitude = 0.0f;
                advert.longitude = 0.0f;
                advert.battery_mv = getBatteryVoltage();
                sendMeshFrame(&advert);
                // Wait for release to avoid re-trigger
                while (digitalRead(BUTTON_PIN) == LOW) delay(10);
                break;
            }
            delay(10);
        }
    }
#endif

    // ── 1. Power on GPS and search for a 2D/3D fix ──────────────────────────
    DBG_PRINT("Wake: powering GPS on and starting acquisition\r\n");
    vextOn();
    delay(100);     // allow Vext rail & Air530Z module to power up
    GPS.begin();

    // Ensure radio is configured for beacons while acquiring GPS fix
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetSyncWord(LORA_SYNC_WORD);
    Radio.SetTxConfig(
        MODEM_LORA,
        TX_OUTPUT_POWER,
        0,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        false,
        true,
        0,
        0,
        false,
        3000);

    bool    fixAcquired = false;
    float   lat = 0.0f, lon = 0.0f;
    uint32_t gpsStart = millis();
    uint32_t nextBeacon = gpsStart; // send immediately on wake
    // Button press tracking during GPS acquisition (allows long-press while waiting)
#if BUTTON_AVAILABLE
    uint32_t btnPressStart = 0;
    bool btnAdvertSent = false;
#endif

    while ((millis() - gpsStart) < GPS_TIMEOUT_MS) {
        while (GPS.available() > 0) {
            GPS.encode(GPS.read());
        }
        if (GPS.location.isValid() && GPS.location.age() < 2000) {
            lat = (float)GPS.location.lat();
            lon = (float)GPS.location.lng();
            fixAcquired = true;
            break;
        }

        // Allow the user button to trigger a flood advert while waiting for GPS
        // This makes the device responsive during the acquisition window.
#if BUTTON_AVAILABLE
        int btnState = digitalRead(BUTTON_PIN);
        if (btnState == LOW) {
            if (btnPressStart == 0) btnPressStart = millis();
            else if (!btnAdvertSent && (millis() - btnPressStart) >= BUTTON_LONGPRESS_MS) {
                DBG_PRINT("Long-press detected during GPS acquisition -> sending flood advert\r\n");
                MeshCoreFrame advert = {};
                advert.mc_header = MC_HEADER_BYTE;
                advert.path_len = 0x00;
                memcpy(advert.sender_id, SENDER_ID, sizeof(advert.sender_id));
                memset(advert.dest_id, 0xFF, sizeof(advert.dest_id));
                advert.flags = 0x02; // advert flag (application-defined)
                advert.hop_limit = 3;
                advert.latitude = 0.0f;
                advert.longitude = 0.0f;
                advert.battery_mv = getBatteryVoltage();
                sendMeshFrame(&advert);
                btnAdvertSent = true;
            }
        } else {
            btnPressStart = 0;
        }
#endif

        // While waiting for fix, periodically send a lightweight beacon
        if (millis() >= nextBeacon) {
            MeshCoreFrame beacon = {};
            beacon.mc_header = MC_HEADER_BYTE;
            beacon.path_len = 0x00;
            memcpy(beacon.sender_id, SENDER_ID, sizeof(beacon.sender_id));
            memset(beacon.dest_id, 0xFF, sizeof(beacon.dest_id));
            beacon.flags = 0x01; // indicate no-fix (application-defined)
            beacon.hop_limit = 3;
            beacon.latitude = 0.0f;
            beacon.longitude = 0.0f;
            beacon.battery_mv = getBatteryVoltage();
            DBG_PRINT("Beacon (no-fix): batt=%u mV\r\n", beacon.battery_mv);
            sendMeshFrame(&beacon);
            nextBeacon = millis() + BEACON_INTERVAL_MS;
        }
    }

    // ── 2. No fix within timeout: conserve battery, sleep immediately ────────
    if (!fixAcquired) {
        DBG_PRINT("No GPS fix within %lu ms — sleeping\r\n", GPS_TIMEOUT_MS);
        GPS.end();
        vextOff();
        goToSleep(SLEEP_INTERVAL_MS);
        return;   // loop() called again after wakeup
    }

    // ── 3. Fix acquired: read battery voltage ────────────────────────────────
    // getBatteryVoltage() handles VBAT_ADC_CTL pin setup/teardown internally
    // for CubeCell_GPS boards, averages 50 ADC reads, and returns millivolts.
    uint16_t battMv = getBatteryVoltage();
    DBG_PRINT("GPS fix: lat=%f lon=%f, battery=%u mV\r\n", lat, lon, battMv);

    // ── 4. Build MeshCore RAW_CUSTOM flood frame ─────────────────────────────
    MeshCoreFrame frame = {};

    frame.mc_header = MC_HEADER_BYTE;
    frame.path_len  = 0x00;

    // frame.sender_id is populated from the global SENDER_ID defined above.
    memcpy(frame.sender_id, SENDER_ID, sizeof(SENDER_ID));

    memset(frame.dest_id, 0xFF, sizeof(frame.dest_id));   // broadcast

    frame.flags     = 0x00;
    frame.hop_limit = 3;
    frame.latitude  = lat;
    frame.longitude = lon;
    frame.battery_mv = battMv;

    // ── 5. Configure radio and transmit once ─────────────────────────────────
    // Re-apply channel and sync word each cycle: CySysPmDeepSleep may use the
    // SX1262 cold-start sleep mode, which does not retain register contents.
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetSyncWord(LORA_SYNC_WORD);

    Radio.SetTxConfig(
        MODEM_LORA,
        TX_OUTPUT_POWER,
        0,                       // frequency deviation (FSK only, unused)
        LORA_BANDWIDTH,          // 1 = 250 kHz
        LORA_SPREADING_FACTOR,   // 9
        LORA_CODINGRATE,         // 1 = 4/5
        LORA_PREAMBLE_LENGTH,    // 8 symbols
        false,                   // fixed-length payload: off (variable)
        true,                    // CRC: enabled
        0,                       // frequency hopping: disabled
        0,                       // hop period (unused)
        false,                   // IQ inversion: off
        3000                     // internal TX timeout (ms)
    );

    DBG_PRINT("Transmitting frame (%u bytes)\r\n", (unsigned)sizeof(frame));
    txDone = false;
    Radio.Send((uint8_t *)&frame, sizeof(frame));

    // Spin-wait for TxDone or TxTimeout callback; software watchdog caps at 5 s.
    // Radio.IrqProcess() must be called to service the SX1262 IRQ line and
    // trigger the registered callbacks (TxDone / TxTimeout).
    uint32_t txStart = millis();
    while (!txDone && (millis() - txStart) < TX_TIMEOUT_MS) {
        Radio.IrqProcess();
    }

    // ── 6. Power down everything and return to deep sleep ────────────────────
    Radio.Sleep();
    DBG_PRINT("Radio put to sleep\r\n");
    GPS.end();
    vextOff();
    DBG_PRINT("Entering deep sleep for %lu ms\r\n", SLEEP_INTERVAL_MS);
    goToSleep(SLEEP_INTERVAL_MS);
    // loop() called again after wakeup — returns to step 1
}