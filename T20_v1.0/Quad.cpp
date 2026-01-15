// Serial functions should not be called when the arduino is running without a usb connection.
// disable this when the program is ready for a wireless use.
#define ENABLE_SERIAL 0

#include "Quad.h"

void Quad::initialize(byte _i2c_address){
  i2c_address = _i2c_address;

	// Software reset
	writeRegister(ADS7138_REG_GENERAL_CFG, ADS7138_GENERAL_CFG_RST);
  delay(100);

	// Get system status
	byte status = readRegister(ADS7138_REG_SYSTEM_STATUS);
#if SERIAL_ENABLE
	Serial.print("ads7138 status: ");	
	Serial.println(status, BIN);	
#endif

	// Set data configuration (enable channel id field)
	writeRegister(ADS7138_REG_DATA_CFG, ADS7138_DATA_CFG_CH_ID);
	
	// Configure pins: 
  // pins 0 - 3 : GPIO out
  // pins 4 - 7 : GPIO out
	writeRegister(ADS7138_REG_PIN_CFG, 0b00001111);
	writeRegister(ADS7138_REG_GPIO_CFG, 0b00001111);

  // Turn off all LEDs
	writeRegister(ADS7138_REG_GPO_VALUE, 0b11111111);

	// Manual mode, conversions are initiated by host
	writeRegister(ADS7138_REG_OPMODE_CFG, 0);

	// Manual sequence mode
	writeRegister(ADS7138_REG_SEQUENCE_CFG, 0);
}

uint16_t Quad::readCell(uint8_t i)
{
  if(i >= 4)
    return 0;

  // turn on the LED
  uint8_t led = (uint8_t)~(1 << i);
	writeRegister(ADS7138_REG_GPO_VALUE, led);

  // time for the photodiode to respond (according to the datasheet)
  // 200 gives a higher output... tradeoff between time and sensitivity
  // 100 -> approx. 2 ms increase in the whole data acquisition time.
  delayMicroseconds(100);

	// select ADC channel
  uint8_t ch = i + 4; // channels 4 to 7 are used for adc input.
	writeRegister(ADS7138_REG_CHANNEL_SEL, ch);

  // read the channel
  Wire.requestFrom(i2c_address, 2);
  while(!Wire.available()){}
  uint16_t high = Wire.read();
  while(!Wire.available()){}
  uint16_t low = Wire.read();

  // turn off the LED
	writeRegister(ADS7138_REG_GPO_VALUE, 0b11111111);

  return (high << 4) | (low >> 4);
}

uint8_t Quad::readRegister(uint8_t address){
  Wire.beginTransmission(i2c_address);
  Wire.write(ADS7138_OPCODE_READ);  // this is needed unlike other chips...
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(i2c_address, 1);
  while(!Wire.available()){}
  return Wire.read();
}

void Quad::writeRegister(uint8_t address, uint8_t data){
  Wire.beginTransmission(i2c_address);
  Wire.write(ADS7138_OPCODE_WRITE);  // this is needed unlike other chips...
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

