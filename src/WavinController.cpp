#include <ESP8266WiFi.h>
#include "WavinController.h"


WavinController::WavinController(uint8_t pin, bool swapSerialPins, uint16_t timeout_ms, uint16_t minGapMs)
{
  txEnablePin = pin;
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);

  recieveTimeoutMs = timeout_ms;
  minTransactionGapMs = minGapMs;

  Serial.begin(38400);

  if(swapSerialPins)
  {
    Serial.swap();
  }
}


WavinController::ModbusDiagnostics WavinController::getModbusDiagnostics() const
{
  return modbusDiagnostics;
}

WavinController::ModbusFailureDetails WavinController::getModbusFailureDetails() const
{
  return failureDetails;
}

WavinController::CategoryDiagnostics WavinController::getCategoryDiagnostics(uint8_t category) const
{
  return categoryDiagnostics[category < 8 ? category : 8];
}

WavinController::ResponseTimingDiagnostics WavinController::getResponseTimingDiagnostics() const
{
  return responseTimingDiagnostics;
}

WavinController::UartDiagnostics WavinController::getUartDiagnostics() const
{
  return uartDiagnostics;
}

WavinController::ReadRetryDiagnostics WavinController::getReadRetryDiagnostics() const
{
  return readRetryDiagnostics;
}

void WavinController::recordRequestResult(bool successful, uint8_t command,
    uint8_t category, uint8_t page, uint8_t index, uint8_t count)
{
  // Mark the end of this transaction after receive() has returned.
  // millis() subtraction is safe across the timer rollover.
  lastTransactionEndMs = millis();
  hasCompletedTransaction = true;
  recordTransaction(successful);
  CategoryDiagnostics &categoryStats = categoryDiagnostics[category < 8 ? category : 8];
  if (successful) ++categoryStats.successes;
  else ++categoryStats.errors;

  if (responseFirstByteSeen) {
    ++responseTimingDiagnostics.firstByteSamples;
    responseTimingDiagnostics.firstByteLastMs = responseFirstByteMs;
    if (responseFirstByteMs > responseTimingDiagnostics.firstByteMaxMs)
      responseTimingDiagnostics.firstByteMaxMs = responseFirstByteMs;
    responseTimingDiagnostics.firstByteTotalMs += responseFirstByteMs;
  } else if (!successful && receiveFailureReason == 1) {
    ++responseTimingDiagnostics.noByteTimeouts;
  }
  if (successful) {
    ++responseTimingDiagnostics.completedResponses;
    responseTimingDiagnostics.completeLastMs = responseCompleteMs;
    if (responseCompleteMs > responseTimingDiagnostics.completeMaxMs)
      responseTimingDiagnostics.completeMaxMs = responseCompleteMs;
    responseTimingDiagnostics.completeTotalMs += responseCompleteMs;
    return;
  }
  if (command == MODBUS_READ_REGISTER) ++failureDetails.readErrors;
  else ++failureDetails.writeErrors;
  switch (receiveFailureReason) {
    case 2: ++failureDetails.crcErrors; break;
    case 3: ++failureDetails.lengthErrors; break;
    case 4: ++failureDetails.headerErrors; break;
    case 5: ++failureDetails.overflowErrors; break;
    default: ++failureDetails.timeoutErrors; break;
  }
  failureDetails.command = command;
  failureDetails.category = category;
  failureDetails.page = page;
  failureDetails.index = index;
  failureDetails.count = count;
  failureDetails.reason = receiveFailureReason;
  failureDetails.hasFailure = true;
}

void WavinController::recordTransaction(bool successful)
{
  modbusDiagnostics.hasAttemptedTransaction = true;
  if (successful)
  {
    ++modbusDiagnostics.successes;
    modbusDiagnostics.consecutiveFailures = 0;
    modbusDiagnostics.lastSuccessMs = millis();
    modbusDiagnostics.hasSuccessfulTransaction = true;
  }
  else
  {
    ++modbusDiagnostics.errors;
    if (modbusDiagnostics.consecutiveFailures != UINT32_MAX)
      ++modbusDiagnostics.consecutiveFailures;
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


bool WavinController::receive(
  uint16_t *reply,
  uint8_t cmdtype,
  uint8_t expectedRegisterCount)
{
  receiveFailureReason = 1;
  responseFirstByteSeen = false;
  responseFirstByteMs = 0;
  responseCompleteMs = 0;
  if (expectedRegisterCount == 0 || expectedRegisterCount > 22) { receiveFailureReason = 3; return false; }
  uint8_t buffer[RECIEVE_BUFFER_SIZE];
  uint8_t n = 0;
  unsigned long start_time = millis();

  while (millis() - start_time < recieveTimeoutMs)
  {
    while (Serial.available() && n < RECIEVE_BUFFER_SIZE)
    {
      buffer[n] = Serial.read();
      if (!responseFirstByteSeen) {
        responseFirstByteSeen = true;
        responseFirstByteMs = millis() - start_time;
      }
      n++;
      if (n >= 3 && buffer[2] > RECIEVE_BUFFER_SIZE - 5) { receiveFailureReason = 5; return false; }

      if (n > 5 &&
          buffer[0] == MODBUS_DEVICE &&
          buffer[1] == cmdtype &&
          buffer[2] + 5 == n)
      {
        // Complete package received.
        uint16_t crc = calculateCRC(buffer, n);

        if (crc != 0)
        {
          receiveFailureReason = 2;
          return false;
        }

        // Each register contains 2 bytes.
        uint8_t expectedDataBytes = expectedRegisterCount * 2;

        // Make sure the controller returned exactly the number
        // of registers that the caller has allocated space for.
        if (buffer[2] != expectedDataBytes)
        {
          receiveFailureReason = 3;
          return false;
        }

        // CRC and response size are valid.
        // It is now safe to copy the registers to reply.
        for (uint8_t j = 0; j < expectedRegisterCount; j++)
        {
          reply[j] =
            (static_cast<uint16_t>(buffer[3 + j * 2]) << 8) |
            static_cast<uint16_t>(buffer[4 + j * 2]);
        }

        responseCompleteMs = millis() - start_time;
        return true;
      }
    }
    // Give ESP8266 Wi-Fi stack and watchdog time to run
    // while waiting for the Wavin controller to respond.
    yield();
  }

  // A partial or unexpected reply may be a header mismatch, rather than silence.
  if (n > 0) {
    receiveFailureReason = 4;
    ++uartDiagnostics.partialResponseFailures;
    uartDiagnostics.partialResponseBytes += n;
    if (n > uartDiagnostics.maxPartialResponseBytes)
      uartDiagnostics.maxPartialResponseBytes = n;
  } else {
    ++uartDiagnostics.zeroByteFailures;
  }
  return false;
}


void WavinController::transmit(uint8_t *data, uint8_t length)
{
  // Phase 3.4.2: enforce a minimum inter-transaction quiet period.
  // This applies to reads and both write commands, including after timeouts.
  // It never delays the first request after boot.
  if (hasCompletedTransaction && minTransactionGapMs > 0)
  {
    while ((uint32_t)(millis() - lastTransactionEndMs) < minTransactionGapMs)
      yield();
  }

  // Empty recieve buffer before sending
  uint16_t discarded = 0;
  while (Serial.read() != -1) {
    ++discarded;
  }
  if (discarded) {
    ++uartDiagnostics.preTransmitNonemptyFlushes;
    uartDiagnostics.preTransmitDiscardedBytes += discarded;
  }

  digitalWrite(txEnablePin, HIGH);

  Serial.write(data, length);

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

  bool ok = receive(reply, MODBUS_READ_REGISTER, count);
  // Record the first physical request even if it will be retried.
  // This keeps all existing raw error and timing diagnostics comparable.
  const bool retryable = !ok && receiveFailureReason == 1 && !responseFirstByteSeen;
  recordRequestResult(ok, MODBUS_READ_REGISTER, category, page, index, count);
  if (!retryable) return ok;

  ++readRetryDiagnostics.attempts;
  // Wait beyond the existing minimum transaction gap before one retry.
  // delay() yields to the ESP8266 Wi-Fi stack and watchdog.
  delay(READ_RETRY_DELAY_MS);
  transmit(message, 8);
  ok = receive(reply, MODBUS_READ_REGISTER, count);
  recordRequestResult(ok, MODBUS_READ_REGISTER, category, page, index, count);
  if (ok) ++readRetryDiagnostics.recovered;
  else ++readRetryDiagnostics.failed;
  return ok;
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
  bool ok = receive(reply, MODBUS_WRITE_REGISTER, 1);
  recordRequestResult(ok, MODBUS_WRITE_REGISTER, category, page, index, 1);
  return ok; // Receive reply but ignore its register value
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
  bool ok = receive(reply, MODBUS_WRITE_MASKED_REGISTER, 1);
  recordRequestResult(ok, MODBUS_WRITE_MASKED_REGISTER, category, page, index, 1);
  return ok; // Receive reply but ignore its register value
}

bool WavinController::getPumpState(uint8_t& tevent)
{
    uint16_t raw;

    if (!readRegisters(CATEGORY_RELAYS, 0, RELAY_TIMER_EVENT, 1, &raw)) {
        return false;
    }

    tevent = raw & 0x0F;
    return true;
}

bool WavinController::getInletTemperature(float& temp)
{
    uint16_t status;

    // MAIN STATUS = 0x08
    if (!readRegisters(CATEGORY_MAIN, 0, MAIN_STATUS_L, 1, &status)) {
        return false;
    }

    // Check "INLET SENSOR PRESENT" (bit 11)
    if (!(status & (1 << 11))) {
        return false;
    }

    uint16_t raw;

    // INLET TEMP = 0x0F
    if (!readRegisters(CATEGORY_MAIN, 0, MAIN_INLET_SENSOR_TEMP, 1, &raw)) {
        return false;
    }

    if (raw == 0x7FFF) {
        return false;
    }

    temp = static_cast<int16_t>(raw) / 10.0f;
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
    if (readRegisters(CATEGORY_ELEMENTS, el, EL_AIR_TEMP, 1, &raw) && raw != 0x7FFF)
        temp = static_cast<int16_t>(raw) / 10.0f;
    else
        temp = NAN;

    // Humidity = 0x07
    if (readRegisters(CATEGORY_ELEMENTS, el, EL_HUMIDITY, 1, &raw) && raw != 0x7FFF)
        hum = static_cast<float>(raw);
    else
        hum = NAN;

    // Dew point = 0x06
    if (readRegisters(CATEGORY_ELEMENTS, el, EL_DEW_POINT, 1, &raw) && raw != 0x7FFF)
        dew = static_cast<int16_t>(raw) / 10.0f;
    else
        dew = NAN;

    // RSSI = 0x09
    if (readRegisters(CATEGORY_ELEMENTS, el, EL_RSSI, 1, &raw)) {
        int8_t rssiRaw = raw & 0xFF;
        rssi = -74.0f + (rssiRaw * 0.5f);
    } else {
        rssi = NAN;
    }

    return true;
}
// Get the current consumption of a channel in milliamperes but is not inused in the current implementation. It is here for future use. Perhaps
bool WavinController::getChannelCurrent(
    uint8_t ch,
    uint16_t& raw,
    float& current)
{
    if (!readRegisters(
        CATEGORY_CHANNELS,
        ch,
        CHANNELS_CURRENT_CONSUMPTION,
        1,
        &raw))
    {
        return false;
    }

    current = raw * 0.54f;

    return true;
}

bool WavinController::getActuatorMotion(uint16_t& interval, uint16_t& duration)
{
    if (!readRegisters(CATEGORY_MAIN, 0, MAIN_ACTUATOR_ACT_INTERVAL, 1, &interval)) {
        return false;
    }

    if (!readRegisters(CATEGORY_MAIN, 0, MAIN_ACTUATOR_ACT_DURATION, 1, &duration)) {
        return false;
    }

    return true;
}
