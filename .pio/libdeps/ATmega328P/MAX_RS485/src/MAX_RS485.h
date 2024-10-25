
#ifndef MAX_RS485_H
#define MAX_RS485_H

#ifndef ARDUINO_ARCH_AVR
  #error “This library only supports boards with an AVR or SAM processor.”
#endif

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

class MAX_RS485: public Stream
{
  private:
    uint8_t rePin;
    uint8_t dePin;
	
    long txDelay;
    long rxDelay;

    SoftwareSerial* ss = nullptr;
    HardwareSerial* hs = nullptr;

    bool isSoftwareSerial;

    void setModeToReceiver();
    void setModeToTransmitter();

  public:
    MAX_RS485(uint8_t rxPin, uint8_t txPin, uint8_t receiverEnablePin, uint8_t driveEnablePin);
    MAX_RS485(HardwareSerial* hardwareSerial, uint8_t receiverEnablePin, uint8_t driveEnablePin);

    void begin(long speed, unsigned long timeout, byte hwConfig = SERIAL_8N1);
	
    long getChangeToTransmitterDelay();
    long getChangeToReceiverDelay();
    void setDelays(long txDelay, long rxDelay);

    int available();
    int read();
    int peek();

    int availableForWrite();
    void flush();

    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);
};

#endif
