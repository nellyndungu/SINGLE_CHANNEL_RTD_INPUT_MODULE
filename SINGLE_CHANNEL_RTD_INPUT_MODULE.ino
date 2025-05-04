// NDUNG'U NELLY W.
// 01/05/2025
#include <SPI.h>

// Globals
// Pin assignments
const int CS_PIN;           // Chip Select pin for MCP3204
const float VREF = 5.0;     // Voltage reference from microcontroller
const float RREF = 3300.0;  // Reference resistor in ohms

// Callendar–Van Dusen equation coefficients
const float RTD_A = 3.9083e-3;
const float RTD_B = -5.775e-7;

// RTD-type selection
const int RTD_SELECT_PIN;  // HIGH = Pt1000, LOW = Pt100

void setup() {
  //Initialize Serial monitor
  Serial.begin(115200);
  SPI.begin();
  // Configure pins
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(RTD_SELECT_PIN, INPUT_PULLUP);  // Default HIGH (Pt1000)
}

void loop() {
  // Read from ADC channel 0
  uint16_t adcRaw = readADC(0);
  // Convert ADC value to voltage across RTD
  float vRTD = (adcRaw / 4095.0) * VREF;
  // Compute RTD resistance
  float rRTD = (vRTD * RREF) / VREF;
  // Determine R0 based on selected RTD type
  float R0 = digitalRead(RTD_SELECT_PIN) == HIGH ? 1000.0 : 100.0;
  // Compute temperature from RTD resistance
  float temperature = calculateTemperature(rRTD, R0);
  delay(1000);
}

// SPI Read function, single-ended
uint16_t readADC(uint8_t channel) {
  uint8_t command = 0b00000110 | ((channel & 0x04) >> 2);
  uint8_t msb, lsb;

  digitalWrite(CS_PIN, LOW);
  SPI.transfer(command);
  msb = SPI.transfer(channel << 6);
  lsb = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);

  return ((msb & 0x0F) << 8) | lsb;  // 12-bit output
}

// Inverse Callendar–Van Dusen equation
float calculateTemperature(float resistance, float R0) {
  float Z = (resistance / R0) - 1.0;
  float discriminant = RTD_A * RTD_A - 4 * RTD_B * Z;

  if (discriminant < 0) return -999.0;

  float temp = (-RTD_A + sqrt(discriminant)) / (2 * RTD_B);
  return temp;
}
