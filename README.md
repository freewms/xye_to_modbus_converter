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
