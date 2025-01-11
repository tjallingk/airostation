// Function to read the 12-bit angle value from the AS5600
uint16_t readAngle() {
  uint8_t highByte = readI2CRegister(0x36, 0x0E);
  uint8_t lowByte = readI2CRegister(0x36, 0x0F);
  int angl = ((highByte & 0x0F) << 8) | lowByte;
  angl = map(angl, 0, 4095, 0, 360);

  // Combine the high and low bytes to form the 12-bit value
  return angl; // Only use lower 4 bits of highByte
}
