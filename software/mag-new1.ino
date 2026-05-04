// ============================================================
// MAGNETOMETER SIMULATOR FOR ARDUINO MEGA
// ------------------------------------------------------------
// Serial  = readable debug text to PC
// Serial2 = vendor-style CCSDS binary packet to ZES
//
// Mega pins:
//   TX2 = pin 16  -> connect to ZES RX (through voltage divider)
//   RX2 = pin 17  -> optional, not used here
// ============================================================

#include <Arduino.h>
#include <math.h>

#define DEBUG_BAUD 115200
#define ZES_BAUD   115200

#define MAG_APID     0x710
#define PAYLOAD_LEN  38
#define SEND_PERIOD_MS 100   // 10 Hz

static const uint8_t CCSDS_VERSION = 0x00;
static const uint8_t CCSDS_TYPE_TM = 0x00;
static const uint8_t CCSDS_SEC_HDR = 0x01;
static const uint8_t CCSDS_SEQ_FLAG_STANDALONE = 0x03;

uint16_t magSeq = 0;

// ------------------------------------------------------------
// CRC-16/CCITT-FALSE
// ------------------------------------------------------------
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
      else              crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
void putU16BE(uint8_t* buf, int& idx, uint16_t v) {
  buf[idx++] = (uint8_t)((v >> 8) & 0xFF);
  buf[idx++] = (uint8_t)(v & 0xFF);
}

void putU32BE(uint8_t* buf, int& idx, uint32_t v) {
  buf[idx++] = (uint8_t)((v >> 24) & 0xFF);
  buf[idx++] = (uint8_t)((v >> 16) & 0xFF);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFF);
  buf[idx++] = (uint8_t)(v & 0xFF);
}

// signed 24-bit big-endian
void putI24BE(uint8_t* buf, int& idx, int32_t v) {
  if (v >  8388607) v =  8388607;
  if (v < -8388608) v = -8388608;

  uint32_t raw = (uint32_t)(v & 0xFFFFFF);
  buf[idx++] = (uint8_t)((raw >> 16) & 0xFF);
  buf[idx++] = (uint8_t)((raw >> 8) & 0xFF);
  buf[idx++] = (uint8_t)(raw & 0xFF);
}

// ------------------------------------------------------------
// Build and send one CCSDS packet to ZES on Serial2
// ------------------------------------------------------------
void sendMagPacketToZES(uint16_t seq, int32_t x, int32_t y, int32_t z) {
  const int PRIMARY_HDR_LEN   = 6;
  const int SECONDARY_HDR_LEN = 6;
  const int CRC_LEN           = 2;
  const int TOTAL_LEN         = PRIMARY_HDR_LEN + SECONDARY_HDR_LEN + PAYLOAD_LEN + CRC_LEN;

  uint8_t packet[TOTAL_LEN];
  int idx = 0;

  // ---------------- Primary header ----------------
  uint16_t packetID =
      ((uint16_t)(CCSDS_VERSION & 0x07) << 13) |
      ((uint16_t)(CCSDS_TYPE_TM & 0x01) << 12) |
      ((uint16_t)(CCSDS_SEC_HDR & 0x01) << 11) |
      (MAG_APID & 0x07FF);

  uint16_t seqCtrl =
      ((uint16_t)(CCSDS_SEQ_FLAG_STANDALONE & 0x03) << 14) |
      (seq & 0x3FFF);

  uint16_t packetLength = (uint16_t)(SECONDARY_HDR_LEN + PAYLOAD_LEN + CRC_LEN - 1);

  putU16BE(packet, idx, packetID);
  putU16BE(packet, idx, seqCtrl);
  putU16BE(packet, idx, packetLength);

  // ---------------- Secondary header ----------------
  packet[idx++] = 0x20;   // service type placeholder
  packet[idx++] = 0x01;   // subtype placeholder
  putU32BE(packet, idx, millis());

  // ---------------- Payload (38 bytes) ----------------
  // [0..1]   seq
  // [2..4]   X int24
  // [5..7]   Y int24
  // [8..10]  Z int24
  // [11]     status
  // [12]     mode
  // [13..14] temp (centi-C)
  // [15..18] sample time ms
  // [19..20] output rate Hz
  // [21..37] reserved

  uint8_t payload[PAYLOAD_LEN];
  int p = 0;

  putU16BE(payload, p, seq);
  putI24BE(payload, p, x);
  putI24BE(payload, p, y);
  putI24BE(payload, p, z);

  payload[p++] = 0x00;  // status flags
  payload[p++] = 0x01;  // mode = nominal stream

  int16_t tempCentiC = 2450;  // 24.50 C
  payload[p++] = (uint8_t)((tempCentiC >> 8) & 0xFF);
  payload[p++] = (uint8_t)(tempCentiC & 0xFF);

  putU32BE(payload, p, millis());

  uint16_t rateHz = 1000 / SEND_PERIOD_MS;
  putU16BE(payload, p, rateHz);

  while (p < PAYLOAD_LEN) {
    payload[p++] = 0x00;
  }

  memcpy(packet + idx, payload, PAYLOAD_LEN);
  idx += PAYLOAD_LEN;

  // ---------------- CRC ----------------
  uint16_t crc = crc16_ccitt(packet, idx);
  putU16BE(packet, idx, crc);

  // ---------------- Send binary to ZES ----------------
  Serial2.write(packet, idx);
}

void setup() {
  Serial.begin(DEBUG_BAUD);   // readable output to laptop
  Serial2.begin(ZES_BAUD);    // binary packet output to ZES

  randomSeed(analogRead(A0) ^ micros());

  delay(500);
  Serial.println("Magnetometer simulator started");
  Serial.println("Readable debug on Serial");
  Serial.println("Binary CCSDS packets on Serial2 (TX2 pin 16)");
}

void loop() {
  static unsigned long lastSend = 0;

  if (millis() - lastSend >= SEND_PERIOD_MS) {
    lastSend = millis();

    float t = millis() / 1000.0f;

    // Realistic-looking values in nT
    int32_t x = (int32_t)(22000 + 3500 * sin(0.35f * t) + random(-100, 101));
    int32_t y = (int32_t)(-6000 + 2500 * cos(0.45f * t) + random(-100, 101));
    int32_t z = (int32_t)(41000 + 1800 * sin(0.20f * t + 0.8f) + random(-100, 101));

    // Human-readable debug
    Serial.print("MAG SEQ=");
    Serial.print(magSeq);
    Serial.print("  X=");
    Serial.print(x);
    Serial.print("  Y=");
    Serial.print(y);
    Serial.print("  Z=");
    Serial.println(z);

    // Binary packet to ZES
    sendMagPacketToZES(magSeq, x, y, z);

    magSeq++;
  }
}