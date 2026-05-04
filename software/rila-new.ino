// ============================================================
// RILA SIMULATOR - VENDOR STYLE APPROXIMATION
// ------------------------------------------------------------
// Output frame:
// [A5][5A][01][01][LEN_L][LEN_H][PAYLOAD...][CRC_L][CRC_H]
//
// Payload length = 514
// Payload structure here:
// [0..1]   sequence
// [2..3]   accumulation time
// [4..513] spectrum bytes (510 bytes)
// ============================================================

#include <Arduino.h>

#define BAUD 115200
#define PAYLOAD_LEN 514

uint16_t seq = 0;

// ---------------- CRC-16/CCITT-FALSE ----------------
uint16_t crc16(uint8_t *data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
      else crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void setup() {
  Serial.begin(BAUD);
  randomSeed(analogRead(A0) ^ micros());
}

void loop() {
  uint8_t packet[600];
  int idx = 0;

  // Header
  packet[idx++] = 0xA5;
  packet[idx++] = 0x5A;
  packet[idx++] = 0x01;
  packet[idx++] = 0x01;

  packet[idx++] = (uint8_t)(PAYLOAD_LEN & 0xFF);
  packet[idx++] = (uint8_t)((PAYLOAD_LEN >> 8) & 0xFF);

  // Payload
  packet[idx++] = (uint8_t)(seq & 0xFF);
  packet[idx++] = (uint8_t)((seq >> 8) & 0xFF);

  uint16_t accumSec = 30;
  packet[idx++] = (uint8_t)(accumSec & 0xFF);
  packet[idx++] = (uint8_t)((accumSec >> 8) & 0xFF);

  // 510 bytes of dummy spectrum data
  for (int i = 0; i < 510; i++) {
    packet[idx++] = (uint8_t)random(0, 32);
  }

  // CRC
  uint16_t crc = crc16(packet, idx);
  packet[idx++] = (uint8_t)(crc & 0xFF);
  packet[idx++] = (uint8_t)((crc >> 8) & 0xFF);

  Serial.write(packet, idx);

  seq++;
  delay(1000);
}