

#include "MAX_RS485.h"

MAX_RS485::MAX_RS485(uint8_t rxPin, uint8_t txPin, uint8_t receiverEnablePin, uint8_t driveEnablePin)
{
    this->rePin = receiverEnablePin;
    this->dePin = driveEnablePin;
    this->txDelay = 10;
    this->rxDelay = 1;

    this->ss = new SoftwareSerial(rxPin, txPin);
    this->hs = nullptr;
    this->isSoftwareSerial = true;
}

MAX_RS485::MAX_RS485(HardwareSerial* hardwareSerial, uint8_t receiverEnablePin, uint8_t driveEnablePin)
{
    this->rePin = receiverEnablePin;
    this->dePin = driveEnablePin;
    this->txDelay = 10;
    this->rxDelay = 1;

    this->hs = hardwareSerial;
    this->ss = nullptr;
    this->isSoftwareSerial = false;
}


void MAX_RS485::setModeToReceiver()
{
    flush();
    delay(rxDelay);
    digitalWrite(rePin, LOW);
    digitalWrite(dePin, LOW);
}


void MAX_RS485::setModeToTransmitter()
{
    delay(txDelay);
    digitalWrite(rePin, HIGH);
    digitalWrite(dePin, HIGH);
}


long MAX_RS485::getChangeToTransmitterDelay()
{
    return txDelay;
}


long MAX_RS485::getChangeToReceiverDelay()
{
    return rxDelay;
}


void MAX_RS485::setDelays(long txDelay, long rxDelay)
{
    this->txDelay = txDelay;
    this->rxDelay = rxDelay;
}


void MAX_RS485::begin(long speed, unsigned long timeout, byte hwConfig)
{
    if(isSoftwareSerial)
    {
        ss->begin(speed);
        ss->setTimeout(timeout);
    }
    else
    {
        hs->begin(speed, hwConfig);
        hs->setTimeout(timeout);
    }

    pinMode(dePin, OUTPUT);
    pinMode(rePin, OUTPUT);
    setModeToReceiver();
}


int MAX_RS485::available()
{
    int res;
    if(isSoftwareSerial)
    {
        res = ss->available();
    }
    else
    {
        res = hs->available();
    }
    return res;
}


int MAX_RS485::read()
{
    int res;
    if(isSoftwareSerial)
    {
        res = ss->read();
    }
    else
    {
        res = hs->read();
    }
    return res;
}


int MAX_RS485::peek()
{
    int res;
    if(isSoftwareSerial)
    {
        res = ss->peek();
    }
    else
    {
        res = hs->peek();
    }
    return res;
}


int MAX_RS485::availableForWrite()
{
    int res;
    if(isSoftwareSerial)
    {
        res = ss->availableForWrite();
    }
    else
    {
        res = hs->availableForWrite();
    }
    return res;
}


void MAX_RS485::flush()
{
    if(isSoftwareSerial)
    {
        ss->flush();
    }
    else
    {
        hs->flush();
    }
}

size_t MAX_RS485::write(uint8_t byte)
{
    size_t res;
    setModeToTransmitter();
    if(isSoftwareSerial)
    {
        res = ss->write(byte);
    }
    else
    {
        res = hs->write(byte);
    }
    setModeToReceiver();
    return res;
}


size_t MAX_RS485::write(const uint8_t *buffer, size_t size)
{
    size_t res;
    setModeToTransmitter();
    if(isSoftwareSerial)
    {
        res = ss->write(buffer, size);
    }
    else
    {
        res = hs->write(buffer, size);
    }
    setModeToReceiver();
    return res;
}
