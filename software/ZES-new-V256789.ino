// ============================================================
// ZES Aggregation Test Code
// ------------------------------------------------------------
// Inputs:
//   RILA raw frame on Serial1
//   Magnetometer CCSDS sim frame on SERCOM3 (D0 RX / D6 TX)
//
// Output:
//   Aggregated ZES CCSDS packet on Serial (USB debug / stand-in OBC)
//
// APIDs:
//   MAG simulator input  = 0x710
//   ZES simulator output = 0x700
// ============================================================

#include <Arduino.h>
#include "wiring_private.h"

// ---------------- UARTs ----------------
#define RILA_PORT Serial1
Uart magSerial(&sercom3, 0, 6, SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM3_Handler() {
  magSerial.IrqHandler();
}

// ---------------- Constants ----------------
static const uint8_t RAW_SYNC1 = 0xA5;
static const uint8_t RAW_SYNC2 = 0x5A;
static const uint8_t RAW_VER   = 0x01;
static const uint8_t RAW_TYPE_RILA = 0x01;

static const uint16_t RILA_PAYLOAD_LEN = 514;
static const uint16_t MAX_PAYLOAD = 1024;

// APIDs
static const uint16_t MAG_SIM_APID   = 0x710;
static const uint16_t ZES_SIM_APID   = 0x700;

// CCSDS
static const uint8_t CCSDS_VER = 0x00;
static const uint8_t CCSDS_TM  = 0x00;
static const uint8_t CCSDS_SHF = 0x01;
static const uint8_t CCSDS_SEQ_FLAG_STANDALONE = 0x03;

// ---------------- CRC-16/CCITT-FALSE ----------------
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

// ============================================================
//  RILA RAW PARSER
// ============================================================
enum RawParserState {
  FIND_SYNC1,
  FIND_SYNC2,
  READ_HEADER,
  READ_PAYLOAD,
  READ_CRC
};

struct RilaContext {
  RawParserState state = FIND_SYNC1;
  uint8_t header[6];
  size_t hidx = 0;

  uint16_t payLen = 0;
  uint8_t payload[MAX_PAYLOAD];
  size_t pidx = 0;

  uint8_t crcBytes[2];
  size_t cidx = 0;

  bool newData = false;
  uint16_t seq = 0;
};

RilaContext rila;

void resetRila() {
  rila.state = FIND_SYNC1;
  rila.hidx = 0;
  rila.pidx = 0;
  rila.cidx = 0;
}

void processRilaByte(uint8_t b) {
  switch (rila.state) {
    case FIND_SYNC1:
      if (b == RAW_SYNC1) {
        rila.header[0] = b;
        rila.hidx = 1;
        rila.state = FIND_SYNC2;
      }
      break;

    case FIND_SYNC2:
      if (b == RAW_SYNC2) {
        rila.header[1] = b;
        rila.hidx = 2;
        rila.state = READ_HEADER;
      } else {
        rila.state = FIND_SYNC1;
      }
      break;

    case READ_HEADER:
      rila.header[rila.hidx++] = b;
      if (rila.hidx == 6) {
        uint8_t ver = rila.header[2];
        uint8_t type_ = rila.header[3];
        rila.payLen = (uint16_t)(rila.header[4] | (rila.header[5] << 8));

        if (ver != RAW_VER || type_ != RAW_TYPE_RILA || rila.payLen > MAX_PAYLOAD) {
          resetRila();
          break;
        }

        rila.pidx = 0;
        rila.state = READ_PAYLOAD;
      }
      break;

    case READ_PAYLOAD:
      rila.payload[rila.pidx++] = b;
      if (rila.pidx >= rila.payLen) {
        rila.cidx = 0;
        rila.state = READ_CRC;
      }
      break;

    case READ_CRC:
      rila.crcBytes[rila.cidx++] = b;
      if (rila.cidx == 2) {
        uint8_t frameBuf[6 + MAX_PAYLOAD];
        memcpy(frameBuf, rila.header, 6);
        memcpy(frameBuf + 6, rila.payload, rila.payLen);

        uint16_t calc = crc16_ccitt(frameBuf, 6 + rila.payLen);
        uint16_t recv = (uint16_t)(rila.crcBytes[0] | (rila.crcBytes[1] << 8));

        if (calc == recv) {
          if (rila.payLen >= 2) {
            rila.seq = (uint16_t)rila.payload[0] | ((uint16_t)rila.payload[1] << 8);
          }
          rila.newData = true;
        }
        resetRila();
      }
      break;
  }
}

void processRilaPort() {
  while (RILA_PORT.available() > 0) {
    processRilaByte((uint8_t)RILA_PORT.read());
  }
}

// ============================================================
//  CCSDS MAG PARSER
// ============================================================
struct MagContext {
  uint8_t buf[128];
  size_t idx = 0;
  size_t expectedLen = 0;
  bool receiving = false;

  bool newData = false;
  uint16_t seq = 0;
  int32_t magX = 0;
  int32_t magY = 0;
  int32_t magZ = 0;
  uint8_t status = 0;
};

MagContext mag;

uint16_t readU16BE(const uint8_t* p) {
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

uint32_t readU32BE(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) |
         ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8)  |
         ((uint32_t)p[3]);
}

int32_t readI24BE(const uint8_t* p) {
  int32_t v = ((int32_t)p[0] << 16) |
              ((int32_t)p[1] << 8)  |
              ((int32_t)p[2]);
  if (v & 0x800000) v |= 0xFF000000;
  return v;
}

void resetMagParser() {
  mag.idx = 0;
  mag.expectedLen = 0;
  mag.receiving = false;
}

void tryParseMagPacket() {
  if (mag.idx < 6) return;

  uint16_t packetId = readU16BE(&mag.buf[0]);
  uint16_t seqCtrl  = readU16BE(&mag.buf[2]);
  uint16_t pktLen   = readU16BE(&mag.buf[4]);

  uint16_t apid = packetId & 0x07FF;

  size_t totalLen = 6 + ((size_t)pktLen + 1);
  if (mag.idx < totalLen) return;

  if (apid != MAG_SIM_APID) {
    resetMagParser();
    return;
  }

  if (totalLen < 6 + 6 + 38 + 2) {
    resetMagParser();
    return;
  }

  uint16_t calc = crc16_ccitt(mag.buf, totalLen - 2);
  uint16_t recv = readU16BE(&mag.buf[totalLen - 2]);

  if (calc != recv) {
    resetMagParser();
    return;
  }

  // secondary header starts at byte 6, payload starts at 12
  const uint8_t* pl = &mag.buf[12];

  mag.seq = readU16BE(&pl[0]);
  mag.magX = readI24BE(&pl[2]);
  mag.magY = readI24BE(&pl[5]);
  mag.magZ = readI24BE(&pl[8]);
  mag.status = pl[11];

  mag.newData = true;
  resetMagParser();
}

void processMagPort() {
  while (magSerial.available() > 0) {
    uint8_t b = (uint8_t)magSerial.read();

    if (!mag.receiving) {
      mag.receiving = true;
      mag.idx = 0;
    }

    if (mag.idx < sizeof(mag.buf)) {
      mag.buf[mag.idx++] = b;
      tryParseMagPacket();
    } else {
      resetMagParser();
    }
  }
}

// ============================================================
//  ZES CCSDS OUTPUT
// ============================================================
uint16_t zesSeq = 0;

void putU16BE(uint8_t* buf, size_t& idx, uint16_t v) {
  buf[idx++] = (uint8_t)((v >> 8) & 0xFF);
  buf[idx++] = (uint8_t)(v & 0xFF);
}

void putU32BE(uint8_t* buf, size_t& idx, uint32_t v) {
  buf[idx++] = (uint8_t)((v >> 24) & 0xFF);
  buf[idx++] = (uint8_t)((v >> 16) & 0xFF);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFF);
  buf[idx++] = (uint8_t)(v & 0xFF);
}

void putI32BE(uint8_t* buf, size_t& idx, int32_t v) {
  buf[idx++] = (uint8_t)((v >> 24) & 0xFF);
  buf[idx++] = (uint8_t)((v >> 16) & 0xFF);
  buf[idx++] = (uint8_t)((v >> 8) & 0xFF);
  buf[idx++] = (uint8_t)(v & 0xFF);
}

void sendAggregatedPacket() {
  // Secondary header = 6 bytes
  // Payload layout:
  // 0..1   ZES seq
  // 2..3   RILA seq
  // 4..5   MAG seq
  // 6      flags (bit0 RILA valid, bit1 MAG valid)
  // 7      RILA status
  // 8      MAG status
  // 9..12  magX int32
  // 13..16 magY int32
  // 17..20 magZ int32
  // 21..24 millis timestamp
  // 25..28 rila payload length
  // total = 29 bytes

  const uint16_t payloadLen = 29;
  const uint16_t secHdrLen = 6;
  const uint16_t crcLen = 2;
  const uint16_t totalLen = 6 + secHdrLen + payloadLen + crcLen;

  uint8_t pkt[6 + 6 + 29 + 2];
  size_t idx = 0;

  uint16_t packetId =
      ((uint16_t)(CCSDS_VER & 0x07) << 13) |
      ((uint16_t)(CCSDS_TM & 0x01) << 12) |
      ((uint16_t)(CCSDS_SHF & 0x01) << 11) |
      (ZES_SIM_APID & 0x07FF);

  uint16_t seqCtrl =
      ((uint16_t)(CCSDS_SEQ_FLAG_STANDALONE & 0x03) << 14) |
      (zesSeq & 0x3FFF);

  uint16_t pktLengthField = (uint16_t)(secHdrLen + payloadLen + crcLen - 1);

  putU16BE(pkt, idx, packetId);
  putU16BE(pkt, idx, seqCtrl);
  putU16BE(pkt, idx, pktLengthField);

  // secondary header
  pkt[idx++] = 0x40; // dummy service type
  pkt[idx++] = 0x01; // dummy subtype
  putU32BE(pkt, idx, millis());

  // payload
  uint8_t flags = 0;
  if (rila.newData) flags |= 0x01;
  if (mag.newData)  flags |= 0x02;

  putU16BE(pkt, idx, zesSeq);
  putU16BE(pkt, idx, rila.seq);
  putU16BE(pkt, idx, mag.seq);
  pkt[idx++] = flags;
  pkt[idx++] = 0x00;          // RILA status placeholder
  pkt[idx++] = mag.status;    // MAG status

  putI32BE(pkt, idx, mag.magX);
  putI32BE(pkt, idx, mag.magY);
  putI32BE(pkt, idx, mag.magZ);

  putU32BE(pkt, idx, millis());
  putU32BE(pkt, idx, rila.payLen);

  uint16_t crc = crc16_ccitt(pkt, idx);
  putU16BE(pkt, idx, crc);

  // Serial.write(pkt, idx);  // disabled for Serial Monitor testing

  Serial.print("[ZES] APID=0x");
  Serial.print(ZES_SIM_APID, HEX);
  
  Serial.print(" ZSEQ=");
  Serial.print(zesSeq);
  
  Serial.print(" RILA_SEQ=");
  Serial.print(rila.seq);
  
  Serial.print(" MAG_SEQ=");
  Serial.print(mag.seq);
  
  Serial.print(" MAG_X=");
  Serial.print(mag.magX);
  
  Serial.print(" MAG_Y=");
  Serial.print(mag.magY);
  
  Serial.print(" MAG_Z=");
  Serial.print(mag.magZ);
  
  Serial.print(" FLAGS=0x");
  Serial.println(flags, HEX);

  zesSeq++;
  rila.newData = false;
  mag.newData = false;
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);       // stand-in OBC/debug output
  RILA_PORT.begin(115200);    // raw RILA sim
  magSerial.begin(115200);    // CCSDS MAG sim

  pinPeripheral(0, PIO_SERCOM);      // D0 RX
  pinPeripheral(6, PIO_SERCOM_ALT);  // D6 TX

  uint32_t start = millis();
  while (!Serial && (millis() - start) < 2000) {}

  Serial.println("ZES Aggregation Test Started");
  Serial.println("RILA input  : Serial1 raw");
  Serial.println("MAG input   : SERCOM3 CCSDS APID 0x710");
  Serial.println("ZES output  : CCSDS APID 0x700");
  Serial.println();
}

void loop() {
  processRilaPort();
  processMagPort();

  // send combined packet once both have fresh data
  if (rila.newData && mag.newData) {
    sendAggregatedPacket();
  }
}
