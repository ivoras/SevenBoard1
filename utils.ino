
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
