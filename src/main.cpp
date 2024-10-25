/*
XYE HVAC to Modbus converter

Hardware:
  - Arduino pro mini 328p
  - 2x RS485 shield based on MAX485CSA (with manual TE)

Libraries:
  - ModbusRTUSlave library (https://github.com/CMB27/ModbusRTUSlave).
  - MAX_RS485 library (https://github.com/vacmg/MAX_RS485)

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
#include <Arduino.h>
#include <ModbusRTUSlave.h>
#include <MAX_RS485.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// modbus settings
#define MODBUS_HOLDING_REGISTER_COUNT 15
#define MODBUS_INPUT_REGISTER_COUNT 17
//#define MODBUS_COILS_COUNT 2
//bool coils[MODBUS_COILS_COUNT];
uint16_t holdingRegisters[MODBUS_HOLDING_REGISTER_COUNT];
uint16_t inputRegisters[MODBUS_INPUT_REGISTER_COUNT];
//uint16_t *holdingRegisters = nullptr;
//uint16_t *inputRegisters = nullptr;
const uint32_t portSpeed[6] = {4800, 9600, 19200, 38400, 57600, 115200};
ModbusRTUSlave modbus(RS485_HARDWARE_UART, RS485_RE_DE_PIN);
#define DEFAULT_MODBUS_ID 10
#define DEFAULT_MODBUS_SPEED 0

#define N_MODBUS_DEVICE_ID holdingRegisters[0]
#define N_MODBUS_SPEED holdingRegisters[1]
#define N_XYE_DEVICE_ID holdingRegisters[2]
#define N_MAP_1 holdingRegisters[3]
#define N_MAP_2 holdingRegisters[4]
#define N_MAP_3 holdingRegisters[5]
#define N_MAP_4 holdingRegisters[6]
#define N_COMMAND_HVAC_ID holdingRegisters[7]
#define N_COMMAND_HVAC_MODE holdingRegisters[8]
#define N_COMMAND_HVAC_FAN holdingRegisters[9]
#define N_COMMAND_HVAC_TEMP holdingRegisters[10]
#define N_COMMAND_HVAC_FLAGS holdingRegisters[11]
#define N_COMMAND_HVAC_T_START holdingRegisters[12]
#define N_COMMAND_HVAC_T_STOP holdingRegisters[13]
#define N_COMMAND_HVAC_UNKNOWN holdingRegisters[14]

#define C_MODBUS_DEVICE_ID inputRegisters[0]
#define C_MODBUS_SPEED inputRegisters[1]
#define C_XYE_DEVICE_ID inputRegisters[2]
#define C_HVAC_MAP1 inputRegisters[3]
#define C_HVAC_MAP2 inputRegisters[4]
#define C_HVAC_MAP3 inputRegisters[5]
#define C_HVAC_MAP4 inputRegisters[6]
#define STATE inputRegisters[7]
#define S_NEED_UPDATE_EEPROM 1
#define S_NEED_REBOOT 5
#define ERROR_FLAGS inputRegisters[8]
#define E_EEPROM_ERROR 0
#define E_DEFAULT_SETTINGS 1
#define L_COMMAND_HVAC_ID inputRegisters[9]
#define L_COMMAND_HVAC_MODE inputRegisters[10]
#define L_COMMAND_HVAC_FAN inputRegisters[11]
#define L_COMMAND_HVAC_TEMP inputRegisters[12]
#define L_COMMAND_HVAC_FLAGS inputRegisters[13]
#define L_COMMAND_HVAC_T_START inputRegisters[14]
#define L_COMMAND_HVAC_T_STOP inputRegisters[15]
#define L_COMMAND_HVAC_UNKNOWN inputRegisters[16]


// eeprom settings
#define EEPROM_SETTINGS_ADDRESS 0x0200
#define SETTINGS_COUNT 7

// HVAC settings
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


// global vars
unsigned long xyeBisyTimer = 0;
unsigned long currTimer = 0;
unsigned long currBlinkTimer = 0;
const uint16_t blinkDuration[] = {1000, 100};

// XYE settings
MAX_RS485 xye(XYE_SOFTWARE_UART_RX_PIN, XYE_SOFTWARE_UART_TX_PIN, XYE_SOFTWARE_RE_DE_PIN, XYE_SOFTWARE_RE_DE_PIN);

// function declarations:
void eepromSetup();
void modbusSetup();
void settingsUpdate();
void reboot();
//byte CRC8(byte, size_t);u
void eepromWriteSettings();
void eepromReadSettings();
void blink(unsigned long, unsigned long, uint8_t);

void setup() {

  ERROR_FLAGS = 0x0000;
  STATE = 0x0000;
  pinMode(DEFAULT_SETTINGS_PIN, INPUT_PULLUP);
  pinMode(ARDUINO_LED_PIN, OUTPUT);

  blink (50, 50, 3);

  eepromSetup();
  modbusSetup();

  blink (300, 300, 5);

  wdt_enable (WDTO_1S);
  
  currTimer = millis();
  currBlinkTimer = millis();

}

void loop() {
  
  wdt_reset();
    
  modbus.poll();

  if (currBlinkTimer < millis()) {
    currBlinkTimer = millis() + blinkDuration[int(ERROR_FLAGS==0)];
    digitalWrite(ARDUINO_LED_PIN, !digitalRead(ARDUINO_LED_PIN));
  }

  settingsUpdate();

}

void settingsUpdate(){

  if (N_MODBUS_DEVICE_ID != 0xFFFF) {
    if ((N_MODBUS_DEVICE_ID < 255) && (N_MODBUS_DEVICE_ID > 0) && (N_MODBUS_DEVICE_ID != C_MODBUS_DEVICE_ID)) {
      C_MODBUS_DEVICE_ID = N_MODBUS_DEVICE_ID;      
      bitSet(STATE, S_NEED_UPDATE_EEPROM);
      bitSet(STATE, S_NEED_REBOOT); 
    }
    N_MODBUS_DEVICE_ID = 0xFFFF;
  }
  
  if (N_MODBUS_SPEED != 0xFFFF) {
    if ((N_MODBUS_SPEED <= 5) && (N_MODBUS_SPEED != C_MODBUS_SPEED)) {
      C_MODBUS_SPEED = N_MODBUS_SPEED;
      bitSet(STATE, S_NEED_UPDATE_EEPROM);
      bitSet(STATE, S_NEED_REBOOT);
    }
    N_MODBUS_SPEED = 0xFFFF;
  }

  if (bitRead(STATE, S_NEED_UPDATE_EEPROM)) eepromWriteSettings();
  if (bitRead(STATE, S_NEED_REBOOT)) reboot();
  
}

void eepromSetup(){

  if (!digitalRead(DEFAULT_SETTINGS_PIN)) {
    bitSet(ERROR_FLAGS, E_DEFAULT_SETTINGS);
  }
  eepromReadSettings();

}

void modbusSetup(){

  for (uint8_t i=0; i<MODBUS_HOLDING_REGISTER_COUNT; i++) {
    holdingRegisters[i] = 0xFFFF;
  }
  
  modbus.configureInputRegisters(inputRegisters, MODBUS_INPUT_REGISTER_COUNT);
  modbus.configureHoldingRegisters(holdingRegisters, MODBUS_HOLDING_REGISTER_COUNT);
  if (bitRead(ERROR_FLAGS, E_DEFAULT_SETTINGS) || C_MODBUS_DEVICE_ID > 255 || C_MODBUS_DEVICE_ID < 1 || C_MODBUS_SPEED > 5 || bitRead(ERROR_FLAGS, E_EEPROM_ERROR)) {
    modbus.begin(DEFAULT_MODBUS_ID, portSpeed[DEFAULT_MODBUS_SPEED]);
  } else {
    modbus.begin(C_MODBUS_DEVICE_ID, portSpeed[C_MODBUS_SPEED]);
  }
  
}

void reboot(){
  
  asm volatile ("jmp 0x0");

}

byte CRC8(const byte *data, size_t dataLength)
{
  byte crc = 0x00;
  while (dataLength--) 
  {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--)
  {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum)
    {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

void eepromWriteSettings() {
  uint16_t settings[SETTINGS_COUNT];

  settings[0] = C_MODBUS_DEVICE_ID;
  settings[1] = C_MODBUS_SPEED;
  settings[2] = C_XYE_DEVICE_ID;
  settings[3] = C_HVAC_MAP1;
  settings[4] = C_HVAC_MAP2;
  settings[5] = C_HVAC_MAP3;
  settings[6] = C_HVAC_MAP4;
  
  uint8_t crc = CRC8((void *)&settings, sizeof(settings));

  eeprom_write_block((void*)&settings, (void*)EEPROM_SETTINGS_ADDRESS, sizeof(settings));
  eeprom_write_block((void*)&crc, (void*)(EEPROM_SETTINGS_ADDRESS + sizeof(settings)), sizeof(crc));
  bitClear(STATE,S_NEED_UPDATE_EEPROM);

}

void eepromReadSettings() {

  uint16_t settings[SETTINGS_COUNT];
  uint8_t crc_eeprom;
  uint8_t crc_calculated;

  eeprom_read_block((void*)&settings, (void*)EEPROM_SETTINGS_ADDRESS, sizeof(settings));
  eeprom_read_block((void*)&crc_eeprom, (void*)(EEPROM_SETTINGS_ADDRESS + sizeof(settings)), sizeof(crc_eeprom));
  crc_calculated = CRC8((void *)&settings, sizeof(settings));
  
  if (crc_calculated != crc_eeprom) {
    bitSet(ERROR_FLAGS, E_EEPROM_ERROR);
  }
  C_MODBUS_DEVICE_ID = settings[0];
  C_MODBUS_SPEED = settings[1];
  C_XYE_DEVICE_ID = settings[2];
  C_HVAC_MAP1 = settings[3];
  C_HVAC_MAP2 = settings[4];
  C_HVAC_MAP3 = settings[5];
  C_HVAC_MAP4 = settings[6];
  bitClear(ERROR_FLAGS, E_EEPROM_ERROR);
}

void blink(unsigned long timeHigh, unsigned long timeLow, uint8_t count){

  pinMode(ARDUINO_LED_PIN, OUTPUT);
  digitalWrite(ARDUINO_LED_PIN, LOW);
  delay(1000);
  for (uint8_t i=0; i<count; i++){
    digitalWrite(ARDUINO_LED_PIN, HIGH);
    delay(timeHigh);
    digitalWrite(ARDUINO_LED_PIN, LOW);
    delay(timeLow);
  }

}