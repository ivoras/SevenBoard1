#include "Base32.h"

Base32 base32;

void scanI2C() {
  for (byte i = 1; i < 128; i++) {
    Wire.beginTransmission(i);

    if (Wire.endTransmission() == 0) {
      char buf[20];
      sprintf(buf, "I2C 0x%02x", i);
      Serial.println(buf);
    }
  }
}


bool isI2Cdevice(byte address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

uint16_t sum_u8(uint8_t arr[], uint16_t arr_size) {
  uint16_t sum = 0;

  for (uint16_t i = 0; i < arr_size; i++) {
    sum += arr[i];
  }
  return sum;
}

// This is horribly inefficient.
String b32str(byte *in, long len, bool padding) {
  byte *out;
  int outLen = base32.toBase32(in, len, (byte*&)out, padding);
  byte outStr[outLen+1];
  memcpy(outStr, out, outLen);
  outStr[outLen] = 0;
  free(out);
  return String((char*)outStr);
}
