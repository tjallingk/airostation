#include "tca9548Multiplexer.h"
// Function to control the TCA9548A multiplexer
void selectMultiplexerPort(uint8_t port) {
  if (port > 7) return; // TCA9548A has 8 ports (0-7)
  Wire.beginTransmission(0x70); // Address of the TCA9548A
  Wire.write(1 << port);        // Select the port by writing to the control register
  Wire.endTransmission();
}

void initializeMultiplexerSensors() {
  // Initialize AS5600 sensors on ports 0, 1, 2, and 3
  for (int port = 0; port < 4; port++) {
    selectMultiplexerPort(port);
    as5600Connected[port] = as5600[port].begin();
    if (as5600Connected[port]) {
      Serial.print("AS5600 detected on multiplexer port ");
      Serial.println(port);
    }
  }

  // Initialize ENS160 and AHT21 on port 4
  selectMultiplexerPort(4);
  ensConnected = ens.begin();
  if (ensConnected) Serial.println("ENS160 detected on multiplexer port 4.");

  ahtConnected = aht.begin(); // Use default address
  if (ahtConnected) Serial.println("AHT21 detected on multiplexer port 4.");
}
