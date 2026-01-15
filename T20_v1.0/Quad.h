#include <Arduino.h>
#include <Wire.h>

// ADS7138 opcodes
#define ADS7138_OPCODE_READ		0x10
#define ADS7138_OPCODE_WRITE	0x08

// ADS7138 registers and their fields
#define ADS7138_REG_SYSTEM_STATUS	0x0
#define ADS7138_REG_GENERAL_CFG		0x1
#define ADS7138_GENERAL_CFG_RST		0x1
#define ADS7138_REG_DATA_CFG		0x2
#define ADS7138_DATA_CFG_CH_ID	 	0x10
#define ADS7138_REG_OPMODE_CFG		0x4
#define ADS7138_REG_PIN_CFG			0x5
#define ADS7138_REG_GPIO_CFG			0x7
#define ADS7138_REG_GPO_VALUE			0xB
#define ADS7138_REG_SEQUENCE_CFG	0x10
#define ADS7138_REG_CHANNEL_SEL		0x11

class Quad
{
public:
  Quad(){}
  void initialize(byte _i2c_address);
  uint16_t readCell(uint8_t i);

private:
  byte i2c_address;
  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t data);
};

