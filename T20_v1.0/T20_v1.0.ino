#include <Wire.h>

// Serial functions should not be called when the arduino is running without a usb connection.
// disable this when the program is ready for a wireless use.
#define ENABLE_SERIAL 0

// BLE ////////////////////////////////////////////////////////////////////////

#include <bluefruit.h>

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

void startAdv(void){
  // ADDED
  // Bluefruit.setName("T20-HW1");
  // Bluefruit.Advertising.addName();

  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  Bluefruit.ScanResponse.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

#if ENABLE_SERIAL
  Serial.print("Connected to ");
  Serial.println(central_name);
#endif
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

#if ENABLE_SERIAL
  Serial.println();
  Serial.print("Disconnected, reason = 0x"); 
  Serial.println(reason, HEX);
#endif
}

// T20 transmits a 40-byte packet 50 times a second, which is demanding for ble.
// the ble parameters should be set carefully to meet this high bandwidth requirements.
// see comments starting with //* below for parameter settings with this consideration.
void setupBLE(){
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); //* max TX packet queue
  Bluefruit.configPrphConn(
    247,    //* max MTU size
    20,     // Event length in 1.25ms units (25ms total); can be tuned
    10,     // HVN TX queue size (notifications)
    10      // Write without response TX queue size247);
  );
  Bluefruit.begin();
  Bluefruit.setTxPower(8);      //* max TX power in dBm
  Bluefruit.Periph.setConnInterval(6, 12);  //* shorter connection interval (7.5ms to 15ms)
  
  const char *device_name = getMcuUniqueID();
  char full_name[32];
  snprintf(full_name, sizeof(full_name), "T20-%s", device_name);

#if ENABLE_SERIAL
  Serial.print("Setting the device name to "); 
  Serial.println(full_name); 
#endif

  Bluefruit.setName(full_name);

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  bledfu.begin();
  bledis.setManufacturer("HCIL");
  bledis.setModel("T20");
  bledis.begin();
  bleuart.begin();
  blebas.begin();
  blebas.write(100);

  startAdv();
}

// Data packet ////////////////////////////////////////////////////////////////////

// #pragma pack to avoid padding between the member variables of the structure.
#pragma pack(push, 1)
struct Packet
{
  uint8_t header[3];  // "T20" for sync
  uint8_t battery;    // battery level
  float q[4];         // Q0 - Q3
  uint8_t face[20];   // prox and press on the 20 faces
};
#pragma pack(pop)

static_assert(sizeof(Packet) == 40);

Packet packet;

void init_packet(){
  // prepare the ble packet
  packet.header[0] = 'T';
  packet.header[1] = '2';
  packet.header[2] = '0';
}

void print_packet(){
#if ENABLE_SERIAL
  for(int i = 0; i < 4; i++)
    Serial.printf("%.4f,", packet.q[i]);
  for(int i = 0; i < 20; i++)
    Serial.printf("%d,", packet.face[i]);
  Serial.printf("B%d\n", packet.battery);
#endif
}

// DRV2605 ////////////////////////////////////////////////////////////////////

#include "Adafruit_DRV2605.h"

// use a second I2C because the maximum clock frequency is different.
#define SDA1_PIN 1  // D1 on Seeeduino XIAO
#define SCL1_PIN 2  // D2 on Seeeduino XIAO

TwoWire myWire1 = TwoWire(NRF_TWIM1, NRF_TWIS1, SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SDA1_PIN, SCL1_PIN);

Adafruit_DRV2605 actuator;

void init_actuator(){
  myWire1.begin();
  myWire1.setClock(400000);

  // following basic.ino sample program
  // https://github.com/adafruit/Adafruit_DRV2605_Library/blob/master/examples/basic/basic.ino
  if (!actuator.begin(&myWire1)) {
#if ENABLE_SERIAL
    Serial.println("Could not find DRV2605");
#endif
  }

  // drv2605 can drive LRA or ERM
  actuator.useLRA();

  // trigger by sending 'go' command (instead of using an external trigger) 
  actuator.setMode(DRV2605_MODE_INTTRIG);

  // configure the waveform to play
  // 1 to 5 for ERM libraries and 6 for LRA library
  actuator.selectLibrary(6);

  // fill out the sequence list with effect numbers. effect number 0 means end
  // see https://www.ti.com/lit/ds/symlink/drv2605.pdf for the complete list.
  actuator.setWaveform(0, 24);       // 1st effect
  actuator.setWaveform(1, 0);       // 2nd effect
}

// IMU ///////////////////////////////////////////////////////////////////////////////////

void init_imu(){
  // the baud rate of ebimu should be set in advance
  Serial1.begin(921600);
  Serial1.print("<sof2>");    // set output mode to quaternion
  Serial1.print("<sor20>");    // set output period to #
  Serial1.print("<start>");    // set output period to #
}

bool parse_ebimu_output(char *line, float *quat) {
  // Check if it starts with '*'
  if (line[0] != '*') return false;

  // Move pointer past '*'
  char *p = line + 1;

  // Parse up to 4 comma-separated floats
  int k = 0;
  while (k < 4) {
    char *end = strchr(p, ',');  // find the next comma
    if (end != nullptr) {
      *end = '\0';               // terminate the current token
      quat[k] = atof(p);         // convert to float
      p = end + 1;               // move to next token
    } else {
      quat[k] = atof(p);         // last (or only) token
      k++;
      break;
    }
    k++;
  }

  return (k == 4);
}

// QUAD /////////////////////////////////////////////////////////////////////////////////

#include "Quad.h"

// ADS7138 i2c addresses (the first five)
// Module 1: R1 = 0, R2 = NO    -> 0x17
// Module 2: R1 = 11k, R2 = NO  -> 0x16
// Module 3: R1 = 33k, R2 = NO  -> 0x15
// Module 4: R1 = 100k, R2 = NO -> 0x14
// Module 5: R1 = NO, R2 = NO   -> 0x10

#define NUM_QUADS 5

Quad quads[NUM_QUADS];

uint8_t quad_address[] = {0x17, 0x16, 0x15, 0x14, 0x10};

void init_quads(){
  Wire.begin();
  Wire.setClock(1000000);

  for(int i = 0; i < NUM_QUADS; i++)
    quads[i].initialize(quad_address[i]);
}

// Battery ////////////////////////////////////////////////////////////////////

void init_battery(){
  // The battery charging current is selectable as 50mA or 100mA, where you can 
  // set PIN_CHARGING_CURRENT as high or low to change it to 50mA or 100mA. 
  // The low current charging current is at the input model set up as HIGH LEVEL 
  // and the high current charging current is at the output model set up as LOW LEVEL.
  // https://wiki.seeedstudio.com/XIAO_BLE/#battery-charging-current
  pinMode(PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW);

  // set VBAT_ENABLE to LOW so that the voltage at P0.31_AIN7_BAT may not exceed
  // 3.3V. Why? see the schematic.
  // https://files.seeedstudio.com/wiki/XIAO-BLE/Seeed-Studio-XIAO-nRF52840-Sense-v1.1.pdf
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
}

uint8_t read_battery(){
  // PIN_VBAT is 31, but it should be 32 according to the schematic...?
  return (uint8_t)(analogRead(PIN_VBAT) >> 4);
}

// LEDs ////////////////////////////////////////////////////////////////////////////////

#define LED_RED                 (11)
#define LED_GREEN               (13)
#define LED_BLUE                (12)

void init_LEDs(){
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
}

void red_on() {digitalWrite(LED_RED, LOW);}
void red_off() {digitalWrite(LED_RED, HIGH);}
void green_on() {digitalWrite(LED_GREEN, LOW);}
void green_off() {digitalWrite(LED_GREEN, HIGH);}
void blue_on() {digitalWrite(LED_BLUE, LOW);}
void blue_off() {digitalWrite(LED_BLUE, HIGH);}

//////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
#if ENABLE_SERIAL
  Serial.begin(115200);
  delay(1000);    // wait until Serial is ready.
#endif

  // initialize modules. order does not matter.  
  init_LEDs();
  init_imu();
  init_actuator();
  init_quads();
  init_battery();
  init_packet();

  setupBLE();
}

// use ebimu as a clock -- loop will run once for each data packet from ebimu
// to change loop period, change the sampling rate setting in init_imu()

char ebimu_output[64];  // large enough to handle a line from ebimu

void loop()
{
  red_on();

  // read imu
  // a line from ebimu ends with \r\n
  size_t len = Serial1.readBytesUntil('\n', ebimu_output, sizeof(ebimu_output) - 1);
  ebimu_output[len] = '\0';  // null-terminate
  // Serial.println(ebimu_output);
  parse_ebimu_output(ebimu_output, packet.q);

  // read 20 faces
  for(int i = 0; i < NUM_QUADS; i++){
    for(int j = 0; j < 4; j++){
      packet.face[j + 4 * i] = (uint8_t)(quads[i].readCell(j) >> 4);   // 12bit adc
    }
  }

  // test lra
#if ENABLE_SERIAL
  if(Serial.available()){
    actuator.go();
    while(Serial.available())
      Serial.read();
  }
#endif

  while (bleuart.available()){
    actuator.go();
    while(bleuart.available())
      bleuart.read();
  }

  // read battery level
  packet.battery = read_battery();

  // send packet  
  bleuart.write((byte *)&packet, sizeof(packet));

  print_packet();

  red_off();
  
  unsigned long t0 = millis();
  while(!Serial1.available()){
    // Put the CPU to sleep until an event occurs (e.g., serial interrupt)
    __WFE();  // Wait For Event
  }
#if ENABLE_SERIAL
  Serial.println(millis() - t0);
#endif
}
