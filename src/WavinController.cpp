#include <ESP8266WiFi.h>
#include "WavinController.h"


WavinController::WavinController(uint8_t pin, bool swapSerialPins, uint16_t timeout_ms)
{
  txEnablePin = pin;
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);

  recieveTimeout_ms = timeout_ms;

  Serial.begin(38400);

  if(swapSerialPins)
  {
    Serial.swap();
  }
}


unsigned int WavinController::calculateCRC(uint8_t *frame, uint8_t bufferSize)
{
  uint16_t temp = 0xFFFF;
  bool flag;

  for (uint8_t i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (uint8_t j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
      {
        temp ^= 0xA001;
      }
    }
  }

  return temp;
}


bool WavinController::recieve(uint16_t *reply, uint8_t cmdtype)
{
  uint8_t buffer[RECIEVE_BUFFER_SIZE];
  uint8_t n = 0;
  unsigned long start_time = millis();

  while (millis() - start_time < recieveTimeout_ms)
  {
    while (Serial.available() && n<RECIEVE_BUFFER_SIZE)
    {
      buffer[n] = Serial.read();
      n++;

      if (n > 5 &&
        buffer[0] == MODBUS_DEVICE &&
        buffer[1] == cmdtype &&
        buffer[2] + 5 == n)
      {
        // Complete package
        uint16_t crc = calculateCRC(buffer, n);
        if (crc != 0) return false;

        // CRC ok, copy to reply buffer
        for (int j = 0; j < buffer[2] / 2; j++)
        {
          reply[j] = (buffer[3 + j * 2] << 8) + buffer[4 + j * 2];
        }
        return true;
      }
    }
  }
  return false;
}


void WavinController::transmit(uint8_t *data, uint8_t lenght)
{
  // Empty recieve buffer before sending
  while (Serial.read() != -1);

  digitalWrite(txEnablePin, HIGH);

  Serial.write(data, lenght);

  Serial.flush(); // Wait for data to be sent
  delayMicroseconds(250); // Wait for last char to be transmitted

  digitalWrite(txEnablePin, LOW);
}


bool WavinController::readRegisters(uint8_t category, uint8_t page, uint8_t index, uint8_t count, uint16_t *reply)
{
  uint8_t message[8];

  message[0] = MODBUS_DEVICE;
  message[1] = MODBUS_READ_REGISTER;
  message[2] = category;
  message[3] = index;
  message[4] = page;
  message[5] = count;

  uint16_t crc = calculateCRC(message, 6);

  message[6] = crc & 0xFF;
  message[7] = crc >> 8;

  transmit(message, 8);

  return recieve(reply, MODBUS_READ_REGISTER);
}


bool WavinController::writeRegister(uint8_t category, uint8_t page, uint8_t index, uint16_t value)
{
  uint8_t message[10];

  message[0] = MODBUS_DEVICE;
  message[1] = MODBUS_WRITE_REGISTER;
  message[2] = category;
  message[3] = index;
  message[4] = page;
  message[5] = 1;
  message[6] = value >> 8;
  message[7] = value & 0xFF;

  uint16_t crc = calculateCRC(message, 8);

  message[8] = crc & 0xFF;
  message[9] = crc >> 8;

  transmit(message, 10);

  uint16_t reply[1];
  return recieve(reply, MODBUS_WRITE_REGISTER); // Recieve reply but ignore it. Asume it's ok
}

bool WavinController::writeMaskedRegister(uint8_t category, uint8_t page, uint8_t index, uint16_t value, uint16_t mask)
{
  uint8_t message[12];

  message[0] = MODBUS_DEVICE;
  message[1] = MODBUS_WRITE_MASKED_REGISTER;
  message[2] = category;
  message[3] = index;
  message[4] = page;
  message[5] = 1;
  message[6] = value >> 8;
  message[7] = value & 0xFF;
  message[8] = mask >> 8;
  message[9] = mask & 0xFF;

  uint16_t crc = calculateCRC(message, 10);

  message[10] = crc & 0xFF;
  message[11] = crc >> 8;

  transmit(message, 12);

  uint16_t reply[1];
  return recieve(reply, MODBUS_WRITE_MASKED_REGISTER); // Recieve reply but ignore it. Asume it's ok
}

bool WavinController::getPumpState(uint8_t& tevent)
{
    uint16_t raw;

    if (!readRegisters(CATEGORY_RELAYS, 0, 0x00, 1, &raw)) {
        return false;
    }

    tevent = raw & 0x0F;
    return true;
}

bool WavinController::getInletTemperature(float& temp)
{
    uint16_t status;

    // MAIN STATUS = 0x08
    if (!readRegisters(CATEGORY_MAIN, 0, 0x08, 1, &status)) {
        return false;
    }

    // Check "INLET SENSOR PRESENT" (bit 11)
    if (!(status & (1 << 11))) {
        return false;
    }

    uint16_t raw;

    // INLET TEMP = 0x0F
    if (!readRegisters(CATEGORY_MAIN, 0, 0x0F, 1, &raw)) {
        return false;
    }

    if (raw == 0x7FFF) {
        return false;
    }

    temp = raw / 10.0f;
    return true;
}

bool WavinController::getElementData(
    uint8_t el,
    float& temp,
    float& hum,
    float& dew,
    float& rssi)
{
    uint16_t raw;

    // Temperature = 0x04
    if (readRegisters(CATEGORY_ELEMENTS, el, 0x04, 1, &raw) && raw != 0x7FFF)
        temp = raw / 10.0f;
    else
        temp = NAN;

    // Humidity = 0x07
    if (readRegisters(CATEGORY_ELEMENTS, el, 0x07, 1, &raw) && raw != 0x7FFF)
        hum = raw;
    else
        hum = NAN;

    // Dew point = 0x06
    if (readRegisters(CATEGORY_ELEMENTS, el, 0x06, 1, &raw) && raw != 0x7FFF)
        dew = raw / 10.0f;
    else
        dew = NAN;

    // RSSI = 0x09
    if (readRegisters(CATEGORY_ELEMENTS, el, 0x09, 1, &raw)) {
        int8_t rssiRaw = raw & 0xFF;
        rssi = -74.0f + (rssiRaw * 0.5f);
    } else {
        rssi = NAN;
    }

    return true;
}

bool WavinController::getChannelCurrent(uint8_t ch, float& current)
{
    uint16_t raw;

    if (!readRegisters(CATEGORY_CHANNELS, ch, 0x01, 1, &raw)) {
        return false;
    }

    current = raw * 0.54f;
    return true;
}

bool WavinController::getActuatorMotion(uint16_t& interval, uint16_t& duration)
{
    if (!readRegisters(CATEGORY_MAIN, 0, 0x1C, 1, &interval)) {
        return false;
    }

    if (!readRegisters(CATEGORY_MAIN, 0, 0x1D, 1, &duration)) {
        return false;
    }

    return true;
}

bool WavinController::getActuatorMotion(uint16_t& interval, uint16_t& duration)
{
    if (!readRegisters(CATEGORY_MAIN, 0, 0x1C, 1, &interval)) {
        return false;
    }

    if (!readRegisters(CATEGORY_MAIN, 0, 0x1D, 1, &duration)) {
        return false;
    }

    return true;
}
