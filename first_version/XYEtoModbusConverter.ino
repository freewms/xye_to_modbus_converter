/*
XYE HVAC to Modbus converter

Hardware:
  - Arduino Mega 2560
  - 2x RS485 shield based on MAX485CSA (with manual TE)

Libraries:
  - ModbusRTUSlave library (https://github.com/CMB27/ModbusRTUSlave).
  - MAX_RS485 library (https://github.com/vacmg/MAX_RS485)

Circuit:
  - The center pin of a potentiometer to pin A0, and the outside pins of the potentiometer to your board's logic level voltage (5V or 3.3V) and GND
  - The center pin of a potentiometer to pin A1, and the outside pins of the potentiometer to your board's logic level voltage (5V or 3.3V) and GND
  - A pushbutton switch from pin 2 to GND
  - A pushbutton switch from pin 3 to GND
  - A LED from pin 5 to GND with a 1K ohm series resistor
  - A LED from pin 6 to GND with a 1K ohm series resistor
  - A LED from pin 7 to GND with a 1K ohm series resistor
  - A LED from pin 8 to GND with a 1K ohm series resistor
  - RX pin (typically pin 0 or pin 10 if using SoftwareSerial) to TX pin of the master/client board
  - TX pin (typically pin 1 or pin 11 if using SoftwareSerial) to RX pin of the master/client board
  - GND to GND of the master/client board
  - Pin 13 is set up as the driver enable pin. This pin will be HIGH whenever the board is transmitting.

XYE protocol reverse:
  https://codeberg.org/xye/xye

  The bus is using [RS-485][RS485] as physical layer, X is A, Y is B and E is GND.
  With 4800 baud, 8n1 (8 bits, no-parity, 1 stop bit).

  A master/slave model is used. The CCM polls all possible 64 unit ids. Per units a 130ms
  time slice is used (30ms for sending the query, followed by 100ms timeout, wating for an
  answer from the unit).

  - query and command frames the CCM are 16bytes long
  - answer frames from the units are 32 bytes long

  Every frame start with 0xAA preamble and ends with a CRC followed by 0x55 prologue.

  The CRC is the calculated over the full frames (incl. preamble, CRC as zero and prologue).

      CRC = 255 - sum(data) % 256 + 1

  ### Query/Command from Master:

  | Byte | Field           | Description                                          |
  | ---- | --------------- | ---------------------------------------------------- |
  | 0x00 | preambel        | 0xAA                                                 |
  | 0x01 | command         | 0xc0 - Query, 0xc3 - Set, 0xcc - Lock, 0xcd - Unlock |
  | 0x02 | destination     | 0x00 .. 0x3f - device id, 0xff broadcast             |
  | 0x03 | Source / Own Id | 0x00 .. 0x3f - master device id                      |
  | 0x04 | from master     | 0x80                                                 |
  | 0x05 | Source / Own Id | 0x00 .. 0x3f - master device id                      |
  | 0x06 | payload         |                                                      |
  | 0x07 | payload         |                                                      |
  | 0x08 | payload         |                                                      |
  | 0x09 | payload         |                                                      |
  | 0x0A | payload         |                                                      |
  | 0x0B | payload         |                                                      |
  | 0x0C | payload         |                                                      |
  | 0x0D | command check   | (255 - command code)                                 |
  | 0x0E | CRC             | 255 - sum(data) % 256 + 1                            |
  | 0x0F | prologue        | 0x55                                                 |

  ### Query PayLoad:

  Query, Lock, Unlock: 7 x 0x00
  Set:

  | Byte | Field           | Description                                                                                                |
  | ---- | --------------- | ---------------------------------------------------------------------------------------------------------- |
  | 0x06 | Oper Mode       | 0x00 - off, 0x80 - auto, 0x88 - Cool, 0x82 - Dry, 0x84 - Heat, 0x81 - Fan                                  |
  | 0x07 | Fan             | 0x80 - Auto, 0x01 - High, 0x02 - Medium, 0x03 - Low                                                        |
  | 0x08 | Set Temp        | °C (18°C), (0xff in Fan Mode)                                                                              |
  | 0x09 | Mode Flags      | 0x02 - Aux Heat (Turbo), 0x00 - norm, 0x01 - ECO Mode (sleep), 0x04 - SWING, 0x88 VENT                     |
  | 0x0A | Timer Start     | Sum of: 0x01 - 15min, 0x02 - 30min, 0x04 - 1h, 0x08 - 2h, 0x10 - 4h, 0x20 - 8h, 0x40 - 16h  0x80 - invalid |
  | 0x0B | Timer Stop      | Sum of: 0x01 - 15min, 0x02 - 30min, 0x04 - 1h, 0x08 - 2h, 0x10 - 4h, 0x20 - 8h, 0x40 - 16h  0x80 - invalid |
  | 0x0C | ????            | 0x00

  ### Answer/Status Report to Master:

  | Byte | Field           | Description                                                                                                |
  | ---- | --------------- | ---------------------------------------------------------------------------------------------------------- |
  | 0x00 | preambel        | 0xAA                                                                                                       |
  | 0x01 | response code   | 0xc0 - Query, 0xc3 - Set, 0xcc - Lock, 0xcd - Unlock                                                       |
  | 0x02 | to master       | 0x80                                                                                                       |
  | 0x03 | destination     | 0 .. 0x3f - master device id                                                                               |
  | 0x04 | Source / Own Id | 0 .. 0x3f - device id                                                                                      |
  | 0x05 | destination     | 0 .. 0x3f - master device id                                                                               |
  | 0x06 | ????            | 0x30 - maybe capabilities                                                                                  |
  | 0x07 | capabilities    | 0x80 - extended temp (16 .. 32 °C), 0x10 has SWING                                                         |
  | 0x08 | Oper Mode       | 0x00 - off, 0x80 - auto, 0x88 - Cool, 0x82 - Dry, 0x84 - Heat, 0x81 - Fan                                  |
  | 0x09 | Fan             | 0x80 - Auto, 0x01 - High, 0x02 - Medium -0x03 Low                                                          |
  | 0x0A | Set Temp        | in °C                                                                                                      |
  | 0x0B | T1 Temp         | in 0.5 °C - 0x30                                                                                           |
  | 0x0C | T2A Temp        | in 0.5 °C - 0x30                                                                                           |
  | 0x0D | T2B Temp        | in 0.5 °C - 0x30                                                                                           |
  | 0x0E | T3 Temp         | in 0.5 °C - 0x30                                                                                           |
  | 0x0F | Current         | 0 .. 99 Amps                                                                                               |
  | 0x10 | ????            | 0xff - could be frequency                                                                                  |
  | 0x11 | Timer Start     | Sum of: 0x01 - 15min, 0x02 - 30min, 0x04 - 1h, 0x08 - 2h, 0x10 - 4h, 0x20 - 8h, 0x40 - 16h  0x80 - invalid |
  | 0x12 | Timer Stop      | Sum of: 0x01 - 15min, 0x02 - 30min, 0x04 - 1h, 0x08 - 2h, 0x10 - 4h, 0x20 - 8h, 0x40 - 16h  0x80 - invalid |
  | 0x13 | ????            | 0x01 - run?                                                                                                |
  | 0x14 | Mode Flags      | 0x02 - Aux Heat (Turbo), 0x00 - norm, 0x01 - ECO Mode (sleep), 0x04 - SWING, 0x88 VENT                     |
  | 0x15 | Oper Flags      | 0x04 - water pump running, 0x80 - locked                                                                   |
  | 0x16 | error           | E + bit pos, (0..7)                                                                                        |
  | 0x17 | error           | E + bit pos, (7..f)                                                                                        |
  | 0x18 | protect         | P + bit pos, (0..7)                                                                                        |
  | 0x19 | protect         | P + bit pos, (7..f)                                                                                        |
  | 0x1A | CCM Comm Error  | 00 .. 02                                                                                                   |
  | 0x1B | ????            | (0x00)                                                                                                     |
  | 0x1C | ????            | (0x00)                                                                                                     |
  | 0x0E | CRC             | 255 - sum(data) % 256 + 1                                                                                  |
  | 0x0F | prologue        | 0x55

!NOTE
Build

Created: 2024-08-06
By: Chuksin Dmirty aka glitcher
https://github.com/freewms

*/

#include <ModbusRTUSlave.h>
// #include <SoftwareSerial.h>
#include <MAX_RS485.h>
#include <EEPROM.h>
#include <avr/wdt.h>

// Device settings
#define EEPROM_SETTINGS_ADDRESS 0x02FF
#define SETTINGS_COUNT 7
uint16_t currSettings[SETTINGS_COUNT];
#define MODBUS_DEVICE_PORT_SPEED currSettings[0]
#define MODBUS_DEVICE_ID currSettings[1]
#define XYE_DEVICE_ID currSettings[2]
#define XYE_DEVICE_MAP_START 3
#define XYE_DEVICE_MAP_COUNT 4

const uint16_t portSpeed[4] = {4800, 9600, 19200, 38400};

// HVAC definition
uint8_t *hvacID = nullptr;
uint8_t *hvacGetStatusCount = nullptr;
uint8_t hvacCount = 0;
#define HVAC_COMMAND_SIZE 8
#define HVAC_ATTEMPT_COUNT 3

// XYE payload counters
#define HVAC_DATA_COUNT 22
#define XYE_REQUEST_SIZE 16
#define XYE_RESPONSE_SIZE 32

// HVAC commands
#define HVAC_COMMAND_QUERY 0xC0
#define HVAC_COMMAND_SET 0xC3
#define HVAC_COMMAND_LOCK 0xCC
#define HVAC_COMMAND_UNLOCK 0xCD

// HVAC_ANSWER_ADDR
#define HVAC_ANSWER_OPER_MODE 0x08
#define HVAC_ANSWER_FAN 0x09
#define HVAC_ANSWER_SET_TEMP 0x0A
#define HVAC_ANSWER_MODE_FLAGS 0x14
#define HVAC_ANSWER_TIMER_START 0x11
#define HVAC_ANSWER_TIMER_STOP 0x12
#define HVAC_ANSWER_UNKNOWN 0x10

// HVAC_QUERY_ADDR //with offset!!!
#define HVAC_QUERY_OPER_MODE 0x00 + SETTINGS_COUNT + 1
#define HVAC_QUERY_FAN 0x01 + SETTINGS_COUNT + 1
#define HVAC_QUERY_SET_TEMP 0x02 + SETTINGS_COUNT + 1
#define HVAC_QUERY_MODE_FLAGS 0x03 + SETTINGS_COUNT + 1
#define HVAC_QUERY_TIMER_START 0x04 + SETTINGS_COUNT + 1
#define HVAC_QUERY_TIMER_STOP 0x05 + SETTINGS_COUNT + 1
#define HVAC_QUERY_UNKNOWN 0x06 + SETTINGS_COUNT + 1

// HVAC polling frequency, ms
#define MILLIS_QUERY_XYE 5000
#define MILLIS_XYE_TIMEOUT 150
uint32_t xyeBisyTimer = 0;

// for cycle query HVAC
unsigned long xyeCycle = 0;
uint8_t currHVACindex = 0;
uint8_t sendHVACindex = 0xFF;
uint8_t checkHVACindex = 0xFF;
uint8_t command = HVAC_COMMAND_QUERY;
uint8_t payload[7];

uint16_t *holdingRegisters = nullptr;
uint16_t *inputRegisters = nullptr;

// define modbus
// #define MODBUS_ENABLE_PIN 8
ModbusRTUSlave modbus(Serial);

// define xye
#define XYE_TX_PIN 8
#define XYE_RX_PIN 9
#define XYE_ENABLE_PIN 10
MAX_RS485 xye(XYE_RX_PIN, XYE_TX_PIN, XYE_ENABLE_PIN, XYE_ENABLE_PIN);

void setup()
{

  uint16_t eeAddress = EEPROM_SETTINGS_ADDRESS; // EEPROM address to start reading from
  bool isNeedUpdateSettings = false;
  EEPROM.get(eeAddress, currSettings);
  eeAddress += SETTINGS_COUNT * sizeof(uint16_t);
  unsigned long crc_calculated = eeprom_crc_by_value((byte *)&currSettings, SETTINGS_COUNT * sizeof(uint16_t));
  unsigned long crc_eeprom;
  EEPROM.get(eeAddress, crc_eeprom);
  if (crc_calculated != crc_eeprom)
  {
    MODBUS_DEVICE_PORT_SPEED = 0;
    MODBUS_DEVICE_ID = 1;
    XYE_DEVICE_ID = 0x3F;
    for (uint8_t i = XYE_DEVICE_MAP_START; i < XYE_DEVICE_MAP_START + XYE_DEVICE_MAP_COUNT; i++)
    {
      currSettings[i] = 0x00;
    }
    isNeedUpdateSettings = true;
  }

  if (MODBUS_DEVICE_PORT_SPEED > 3)
  {
    MODBUS_DEVICE_PORT_SPEED = 0;
    isNeedUpdateSettings = true;
  }

  if (MODBUS_DEVICE_ID == 0 || MODBUS_DEVICE_ID > 254)
  {
    MODBUS_DEVICE_ID = 1;
    isNeedUpdateSettings = true;
  }

  if (XYE_DEVICE_ID == 0 || XYE_DEVICE_ID > 0x3F)
  {
    XYE_DEVICE_ID = 0x3F;
    isNeedUpdateSettings = true;
  }

  if (isNeedUpdateSettings == true)
  {
    update_device_config();
  }

  // device count
  hvacCount = 0;
  for (int map = 0; map < XYE_DEVICE_MAP_COUNT; map++)
  {
    for (int i = 0; i < sizeof(uint16_t) * 8; i++)
    {
      if (bitRead(currSettings[map + XYE_DEVICE_MAP_START], i) && (map * 16 + i != XYE_DEVICE_ID))
      {
        hvacCount++;
      }
    }
  }

  // array of device index
  hvacID = (uint8_t *)malloc(sizeof(uint8_t) * hvacCount);
  for (int map = 0; map < XYE_DEVICE_MAP_COUNT; map++)
  {
    for (int i = 0; i < sizeof(uint16_t) * 8; i++)
    {
      if (bitRead(currSettings[map + XYE_DEVICE_MAP_START], i) && (map * 16 + i != XYE_DEVICE_ID))
      {
        hvacID[currHVACindex] = map * 16 + i;
        currHVACindex++;
      }
    }
  }
  currHVACindex = 0;
  
  // array of device query count
  hvacGetStatusCount = (uint8_t *)malloc(sizeof(uint8_t) * hvacCount);
  memset(hvacGetStatusCount, 0x00, sizeof(uint8_t)*hvacCount);

  // holding registry memory allocation
  uint16_t iRegCount = hvacCount * (HVAC_DATA_COUNT);
  inputRegisters = (uint16_t *)malloc(sizeof(uint16_t) * iRegCount);
  if (inputRegisters == nullptr)
  {
    delay(500);
    reboot();
  }
  memset(inputRegisters, 0xFF, sizeof(uint16_t) * (SETTINGS_COUNT + hvacCount * (HVAC_DATA_COUNT)));

  uint16_t hRegCount = SETTINGS_COUNT + HVAC_COMMAND_SIZE;
  holdingRegisters = (uint16_t *)malloc(sizeof(uint16_t) * hRegCount);
  if (holdingRegisters == nullptr)
  {
    delay(500);
    reboot();
  }
  memset(holdingRegisters, 0xFF, sizeof(uint16_t) * (SETTINGS_COUNT + HVAC_COMMAND_SIZE));

  // copy settings to register
  for (int index = 0; index < SETTINGS_COUNT; ++index)
  {
    holdingRegisters[index] = currSettings[index];
  }

  // add device ID to input regs
  for (int index = 0; index < hvacCount * HVAC_DATA_COUNT; index += HVAC_DATA_COUNT)
  {
    inputRegisters[index] = hvacID[currHVACindex];
    currHVACindex++;
  }
  currHVACindex = 0;

  modbus.configureInputRegisters(inputRegisters, iRegCount);
  modbus.configureHoldingRegisters(holdingRegisters, hRegCount);
  modbus.begin(MODBUS_DEVICE_ID, portSpeed[MODBUS_DEVICE_PORT_SPEED]);

  xye.begin(4800, 100);
  
  currHVACindex = hvacCount;
  memset(&payload[0], 0xFF, sizeof(payload));
  xyeCycle = 0;
}

void loop()
{
  
  modbus.poll();
  check_settings();
  if (is_xye_bisy()) {
    return;
  }

  if (is_need_check_data()) {
    check_data_from_hvac();
  }

  if (is_need_send_data()) {
    send_data_to_hvac();  
  }
  
}

bool is_xye_bisy() {
  if (millis() > xyeBisyTimer) {
    xyeBisyTimer = 0;
    return false;
  }
  return true;
}

bool is_need_check_data(){
  if (checkHVACindex == 0xFF) {
    return false;
  }
  return true;
}

bool is_need_send_data(){
  
  if (hvacCount == 0) {
    return false;
  }

  if (is_get_modbus_command()){
    return true;
  }

  if (sendHVACindex == 0xFF && millis() < xyeCycle) {
    return false;
  }

  if (sendHVACindex == 0xFF){
    sendHVACindex = 0;
    return true;
  }

  sendHVACindex++;
  if (sendHVACindex >= hvacCount) 
  {  
    sendHVACindex = 0xFF;
    xyeCycle = millis() + MILLIS_QUERY_XYE;
    return false;
  }

  return true;
  
}
 
bool is_get_modbus_command()
{
  if (command == HVAC_COMMAND_SET && hvacGetStatusCount[checkHVACindex] >= HVAC_ATTEMPT_COUNT) {
    clear_modbus_command();
    return false;
  }

  if (command == HVAC_COMMAND_SET) {
    return true;
  }
  
  if (holdingRegisters[SETTINGS_COUNT] == 0xFFFF) {
    return false;
  }

  uint16_t mbCommand[HVAC_COMMAND_SIZE];
  uint16_t *ptr = holdingRegisters;
  memcpy(&mbCommand, (ptr + SETTINGS_COUNT), sizeof(uint16_t)*HVAC_COMMAND_SIZE);
  for (uint8_t c = 0; c < hvacCount; c++)
    {
      if (hvacID[c] == mbCommand[0])
      {
        sendHVACindex = c;
        hvacGetStatusCount[c]  = 0;
        command = HVAC_COMMAND_SET;
        uint8_t currPayload = 0;
        for (uint8_t i = 1; i < HVAC_COMMAND_SIZE; i++) {
          payload[currPayload] = mbCommand[i];
          currPayload++;
        }
        return true;
      }
    }

  clear_modbus_command();
  return false;
}

void send_data_to_hvac()
{
  
  
  //uint8_t currPayload = 0;
  //  for (uint8_t i = SETTINGS_COUNT + 1; i < SETTINGS_COUNT + HVAC_COMMAND_SIZE; i++)
  //  {
  //    if (command == HVAC_COMMAND_SET) 
  //    {
  //      payload[currPayload] = holdingRegisters[i];
  //    } else {
  //      payload[currPayload] = 0x00;
  //    }
  //  currPayload++;
  //  }

  uint8_t data[XYE_REQUEST_SIZE];
  /*
  xye.println();
  xye.print("HVAC ID: ");
  xye.println(hvacID[sendHVACindex]);

  xye.print("Attempt: ");
  xye.println(hvacGetStatusCount[sendHVACindex]);
  */
  hvac_query(data, hvacID[sendHVACindex], XYE_DEVICE_ID, command, payload);
  while (xye.available())
  {
    xye.read();
  }
  /*
  xye.print("write:");
  for (uint8_t c = 0; c < XYE_REQUEST_SIZE; c++)
  {
    xye.print(" 0x");
    xye.print(data[c], HEX);
  }
  xye.println();
  */
  
  if (hvacGetStatusCount[sendHVACindex] < HVAC_ATTEMPT_COUNT) {
    hvacGetStatusCount[sendHVACindex]++;
  }

  checkHVACindex = sendHVACindex;
  xye.write(data, XYE_REQUEST_SIZE);
  xye.flush();

  xyeBisyTimer = millis() + MILLIS_XYE_TIMEOUT;
}

void check_data_from_hvac()
{
  if (!(xye.available()))
  {
    return;
  }

  if (xye.peek() != 0xAA)
  {
    while (xye.available())
    {
      xye.read();
    }
    return;
  }

  uint8_t status[XYE_RESPONSE_SIZE];
  xye.readBytes(status, XYE_RESPONSE_SIZE);
  if (
      status[0x02] != 0x80 ||
      status[0x03] != XYE_DEVICE_ID ||
      status[0x05] != XYE_DEVICE_ID ||
      status[0x1F] != 0x55)
  {
    return;
  }

  if (
      status[0x01] == 0xC3 &&
      status[0x04] == hvacID[checkHVACindex])
  {
    
  }

  uint8_t currCRC = status[0x1E];
  status[0x1E] = 0x00;
  if (hvac_crc(status, 32) != currCRC)
  {
    return false;
  };

  xye.println();
  xye.print("receive:");
  for (uint8_t c = 0; c < XYE_RESPONSE_SIZE; c++)
  {
    xye.print(" 0x");
    xye.print(status[c], HEX);
  }
  xye.println();

  if (hvacID[checkHVACindex] == status[0x04])
    {
      uint8_t currOffset = SETTINGS_COUNT + checkHVACindex * HVAC_DATA_COUNT + 1;
      for (uint8_t j = 0; j < HVAC_DATA_COUNT - 1; j++)
      {
        inputRegisters[currOffset + j] = (uint16_t)status[6 + j];
      }
      hvacGetStatusCount[checkHVACindex]=0;
      checkHVACindex = 0xFF;
      return;
  }
  
}

void clear_modbus_command()
{ 
  sendHVACindex= hvacCount;
  command = HVAC_COMMAND_QUERY;
  //uint16_t *ptr_hReg = holdingRegisters;
  memset((&holdingRegisters[0] + SETTINGS_COUNT), 0xFFFF, sizeof(uint16_t)*HVAC_COMMAND_SIZE);
  //uint8_t* ptr_payload = payload;
  memset(&payload[0], 0xFF, sizeof(payload));
}

bool check_settings()
{

  bool isNeedUpdateSettings = false;
  for (int index = 0; index < SETTINGS_COUNT; ++index)
  {
    if (holdingRegisters[index] != currSettings[index])
    {
      currSettings[index] = holdingRegisters[index];
      isNeedUpdateSettings = true;
    }
  }

  if (isNeedUpdateSettings)
  {
    update_device_config();
  }
}

unsigned long eeprom_crc_by_value(byte *data, uint16_t length)
{

  const unsigned long crc_table[16] = {
      0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
      0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
      0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
      0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};

  unsigned long crc = ~0L;
  for (int index = 0; index < length; ++index)
  {
    crc = crc_table[(crc ^ data[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (data[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

void update_device_config()
{
  uint16_t eeAddress = EEPROM_SETTINGS_ADDRESS;
  EEPROM.put(eeAddress, currSettings);
  eeAddress += SETTINGS_COUNT * sizeof(uint16_t);
  EEPROM.put(eeAddress, eeprom_crc_by_value((byte *)&currSettings, SETTINGS_COUNT * sizeof(uint16_t)));
  delay(1000);
  reboot();
}

void reboot()
{
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1)
  {
  }
}

void hvac_query(uint8_t query[], uint8_t slaveID, uint8_t masterID, uint8_t command, uint8_t payload_req[])
{
  //| Field           | Description                                                                                                |
  //| --------------- | ---------------------------------------------------------------------------------------------------------- |
  query[0x00] = 0xAA;           //| prologue        | 0xAA                                                                                                       |
  query[0x01] = command;        //| command         | 0xc0 - Query, 0xc3 - Set, 0xcc - Lock, 0xcd - Unlock                                                       |
  query[0x02] = slaveID;        //| destination     | 0x00 .. 0x3f - device id, 0xff broadcast                                                                   |
  query[0x03] = masterID;       //| Source / Own Id | 0x00 .. 0x3f - master device id                                                                            |
  query[0x04] = 0x80;           //| from master     | 0x80                                                                                                       |
  query[0x05] = masterID;       //| Source / Own Id | 0x00 .. 0x3f - master device id                                                                            |
  query[0x06] = payload_req[0x00];  //| Oper Mode       | 0x00 - off, 0x80 - auto, 0x88 - Cool, 0x82 - Dry, 0x84 - Heat, 0x81 - Fan                                  |
  query[0x07] = payload_req[0x01];  //| Fan             | 0x80 - Auto, 0x01 - High, 0x02 - Medium, 0x03 - Low                                                        |
  query[0x08] = payload_req[0x02];  //| Set Temp        | °C (18°C), (0xff in Fan Mode)                                                                              |
  query[0x09] = payload_req[0x03];  //| Mode Flags      | 0x02 - Aux Heat (Turbo), 0x00 - norm, 0x01 - ECO Mode (sleep), 0x04 - SWING, 0x88 VENT                     |
  query[0x0A] = payload_req[0x04];  //| Timer Start     | Sum of: 0x01 - 15min, 0x02 - 30min, 0x04 - 1h, 0x08 - 2h, 0x10 - 4h, 0x20 - 8h, 0x40 - 16h  0x80 - invalid |
  query[0x0B] = payload_req[0x05];  //| Timer Stop      | Sum of: 0x01 - 15min, 0x02 - 30min, 0x04 - 1h, 0x08 - 2h, 0x10 - 4h, 0x20 - 8h, 0x40 - 16h  0x80 - invalid |
  query[0x0C] = payload_req[0x06];  //| ????            | 0x00                                                                                                       |
  query[0x0D] = 0xFF - command; //| command check   | (255 - command code)                                                                                       |
  query[0x0E] = 0x00;           //| CRC             | 255 - sum(data) % 256 + 1                                                                                  |
  query[0x0F] = 0x55;           //| epilogue        | 0x55                                                                                                       |
  query[0x0E] = hvac_crc(query, XYE_REQUEST_SIZE);
}

void hvac_answer(uint8_t slaveID, uint8_t masterID, uint8_t response, uint8_t payload_resp[])
{
  uint8_t hvac_answer[31] = {
      //| Byte | Field           | Description                                                                                                |
      //| ---- | --------------- | -----------------------------------------------------------------------------------------------------------|
      0xAA,          //| 0x00 | prologue        | 0xAA                                                                                                       |
      response,      //| 0x01 | response code   | 0xc0 - Query, 0xc3 - Set, 0xcc - Lock, 0xcd - Unlock                                                       |
      0x80,          //| 0x02 | to master       | 0x80                                                                                                       |
      masterID,      //| 0x03 | destination     | 0 .. 0x3f - master device id                                                                               |
      slaveID,       //| 0x04 | Source / Own Id | 0 .. 0x3f - device id                                                                                      |
      masterID,      //| 0x05 | destination     | 0 .. 0x3f - master device id                                                                               |
      0x30,          //| 0x06 | ????            | 0x30 - maybe capabilities                                                                                  |
      payload_resp[0x00], //| 0x07 | capabilities    | 0x80 - extended temp (16 .. 32 °C), 0x10 has SWING                                                         |
      payload_resp[0x01], //| 0x08 | Oper Mode       | 0x00 - off, 0x80 - auto, 0x88 - Cool, 0x82 - Dry, 0x84 - Heat, 0x81 - Fan                                  |
      payload_resp[0x02], //| 0x09 | Fan             | 0x80 - Auto, 0x01 - High, 0x02 - Medium -0x03 Low                                                          |
      payload_resp[0x03], //| 0x0A | Set Temp        | in °C                                                                                                      |
      payload_resp[0x04], //| 0x0B | T1 Temp         | in 0.5 °C - 0x30                                                                                           |
      payload_resp[0x05], //| 0x0C | T2A Temp        | in 0.5 °C - 0x30                                                                                           |
      payload_resp[0x06], //| 0x0D | T2B Temp        | in 0.5 °C - 0x30                                                                                           |
      payload_resp[0x07], //| 0x0E | T3 Temp         | in 0.5 °C - 0x30                                                                                           |
      payload_resp[0x08], //| 0x0F | Current         | 0 .. 99 Amps                                                                                               |
      payload_resp[0x09], //| 0x10 | ????            | 0xff - could be frequency                                                                                  |
      payload_resp[0x0A], //| 0x11 | Timer Start     | Sum of: 0x01 - 15min, 0x02 - 30min, 0x04 - 1h, 0x08 - 2h, 0x10 - 4h, 0x20 - 8h, 0x40 - 16h  0x80 - invalid |
      payload_resp[0x0B], //| 0x12 | Timer Stop      | Sum of: 0x01 - 15min, 0x02 - 30min, 0x04 - 1h, 0x08 - 2h, 0x10 - 4h, 0x20 - 8h, 0x40 - 16h  0x80 - invalid |
      payload_resp[0x0C], //| 0x13 | ????            | 0x01 - run?                                                                                                |
      payload_resp[0x0D], //| 0x14 | Mode Flags      | 0x02 - Aux Heat (Turbo), 0x00 - norm, 0x01 - ECO Mode (sleep), 0x04 - SWING, 0x88 VENT                     |
      payload_resp[0x0E], //| 0x15 | Oper Flags      | 0x04 - water pump running, 0x80 - locked                                                                   |
      payload_resp[0x0F], //| 0x16 | error           | E + bit pos, (0..7)                                                                                        |
      payload_resp[0x10], //| 0x17 | error           | E + bit pos, (7..f)                                                                                        |
      payload_resp[0x11], //| 0x18 | protect         | P + bit pos, (0..7)                                                                                        |
      payload_resp[0x12], //| 0x19 | protect         | P + bit pos, (7..f)                                                                                        |

      payload_resp[0x13], //| 0x1A | CCM Comm Error  | 00 .. 02                                                                                                   |
      payload_resp[0x14], //| 0x1B | ????            | (0x00)                                                                                                     |
      payload_resp[0x15], //| 0x1C | ????            | (0x00)                                                                                                     |
      0x00,          //| 0x1D | CRC             | 255 - sum(data) % 256 + 1                                                                                  |
      0x00           //| 0x1E | prologue        | 0x55                                                                                                       |
  };
  hvac_answer[0x1D] = hvac_crc(hvac_answer, XYE_RESPONSE_SIZE);
  return hvac_answer;
}

uint8_t hvac_crc(uint8_t data[], uint8_t len)
{
  uint16_t sum = 0;
  xye.write(sizeof(&data));
  for (int index = 0; index < len; ++index)
  {
    sum += data[index];
  }
  sum %= 256;
  sum = 255 - sum;
  return sum;
}
