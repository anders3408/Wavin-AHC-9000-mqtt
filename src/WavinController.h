#include <inttypes.h>


class WavinController
{
  public:
    WavinController(uint8_t pin, bool swapSerialPins, uint16_t timeout_ms);
    bool readRegisters(uint8_t category, uint8_t page, uint8_t index, uint8_t count, uint16_t *reply);
    bool writeRegister(uint8_t category, uint8_t page, uint8_t index, uint16_t value);
    bool writeMaskedRegister(uint8_t category, uint8_t page, uint8_t index, uint16_t value, uint16_t mask);

    // NEW helper methods
    bool getPumpState(uint8_t& tevent);
    bool getInletTemperature(float& temp);

    bool getElementData(
        uint8_t el,
        float& temp,
        float& hum,
        float& dew,
        float& rssi
    );

    bool getChannelCurrent(uint8_t ch, float& current);
    bool getActuatorMotion(uint16_t& interval, uint16_t& duration);
    bool getChannelAlarms(uint8_t ch, bool& high, bool& low);
    
    static const uint8_t CATEGORY_MAIN =        0x00;
    static const uint8_t CATEGORY_ELEMENTS =    0x01;
    static const uint8_t CATEGORY_PACKED_DATA = 0x02;
    static const uint8_t CATEGORY_CHANNELS =    0x03;
    static const uint8_t CATEGORY_RELAYS =      0x04;
    static const uint8_t CATEGORY_CLOCK =       0x05;
    static const uint8_t CATEGORY_SCHEDULES =   0x06;
    static const uint8_t CATEGORY_INFO =        0x07;

    static const uint8_t ELEMENTS_AIR_TEMPERATURE = 0x04;
    static const uint8_t ELEMENTS_BATTERY_STATUS  = 0x0A;
    static const uint8_t ELEMENTS_SYNC_GROUP      = 0x0B;

    static const uint8_t PACKED_DATA_MANUAL_TEMPERATURE = 0x00;
    static const uint8_t PACKED_DATA_STANDBY_TEMPERATURE = 0x04;
    static const uint8_t PACKED_DATA_CONFIGURATION = 0x07;
    static const uint8_t PACKED_DATA_CONFIGURATION_MODE_MASK = 0x07;
    static const uint8_t PACKED_DATA_CONFIGURATION_MODE_MANUAL = 0x00;
    static const uint8_t PACKED_DATA_CONFIGURATION_MODE_STANDBY = 0x01;

    static const uint8_t  NUMBER_OF_CHANNELS = 16;
    static const uint8_t  CHANNELS_TIMER_EVENT = 0x00;
    static const uint16_t CHANNELS_TIMER_EVENT_OUTP_ON_MASK = 0x0010;
    static const uint8_t  CHANNELS_CURRENT_CONSUMPTION = 0x01;
    static const uint8_t  CHANNELS_PRIMARY_ELEMENT = 0x02;
    static const uint16_t CHANNELS_PRIMARY_ELEMENT_ELEMENT_MASK = 0x003f;
    static const uint16_t CHANNELS_PRIMARY_ELEMENT_ALL_TP_LOST_MASK = 0x0400;
    
    static const uint8_t RELAY_TIMER_EVENT     = 0x00;
    static const uint8_t RELAY_ASSIGNMENT_MAP  = 0x01;
    static const uint8_t RELAY_START_DELAY     = 0x02;
    static const uint8_t RELAY_STOP_DELAY      = 0x03;
    static const uint8_t RELAY_ACT_INTERVAL    = 0x04;
    static const uint8_t RELAY_ACT_DURATION    = 0x05;
    static const uint8_t RELAY_TIMER           = 0x06; // reserved

    static const uint8_t RELAY_EVENT_OUTPUT_ON        = 0x07;
    static const uint8_t RELAY_EVENT_STOP_DELAY       = 0x0A;
    static const uint8_t RELAY_EVENT_PERIODIC_CYCLE   = 0x0D;
    static const uint8_t RELAY_EVENT_IDLE             = 0x00;

    static const uint8_t MAIN_STATUS_L                 = 0x08;
    static const uint8_t MAIN_DHW_SENSOR_TEMP          = 0x0E; // not used in your setup
    static const uint8_t MAIN_INLET_SENSOR_TEMP        = 0x0F;
    static const uint8_t MAIN_TOTAL_CURRENT_L          = 0x10;
    static const uint8_t MAIN_TOTAL_CURRENT_H          = 0x11;
    static const uint8_t MAIN_ACTUATOR_ACT_INTERVAL    = 0x1C;
    static const uint8_t MAIN_ACTUATOR_ACT_DURATION    = 0x1D;
    static const uint8_t MAIN_ACTUATOR_POLARITY        = 0x1E;

    static const uint8_t EL_ADDRESS_L         = 0x00;
    static const uint8_t EL_ADDRESS_H         = 0x01;
    static const uint8_t EL_ASSIGNMENT_MAP_L  = 0x02;
    static const uint8_t EL_ASSIGNMENT_MAP_H  = 0x03;
    static const uint8_t EL_AIR_TEMP          = 0x04;
    static const uint8_t EL_FLOOR_TEMP        = 0x05; // likely unused in your setup
    static const uint8_t EL_DEW_POINT         = 0x06;
    static const uint8_t EL_HUMIDITY          = 0x07;
    static const uint8_t EL_STATUS            = 0x08;
    static const uint8_t EL_RSSI              = 0x09;
    static const uint8_t EL_BATTERY           = 0x0A;
    static const uint8_t EL_SYNC_GROUP        = 0x0B;
    static const uint8_t EL_LIVE_TIMER        = 0x0C;
	
    static const uint8_t  INFO_HW_VERSION = 0x02;
    static const uint8_t  INFO_HW_VERSION_MASK = 0x7F;
    static const uint8_t  INFO_SW_VERSION = 0x03;
    static const uint8_t  INFO_SW_VERSION_MASK = 0xFF;
    static const uint8_t  INFO_SW_BETA_VERSION_MASK = 0x0F;
    
  private:
    uint8_t txEnablePin;
    uint16_t recieveTimeout_ms;
    void transmit(uint8_t *data, uint8_t lenght);
    bool recieve(uint16_t *reply, uint8_t cmdtype);
    unsigned int calculateCRC(unsigned char *frame, unsigned char bufferSize);

    const uint8_t MODBUS_DEVICE = 0x01;
    const uint8_t MODBUS_READ_REGISTER = 0x43;
    const uint8_t MODBUS_WRITE_REGISTER = 0x44;
    const uint8_t MODBUS_WRITE_MASKED_REGISTER = 0x45;
    
    // Largest page contains 22 registers of 2 bytes + 5 bytes header
    const uint8_t RECIEVE_BUFFER_SIZE =  22 * 2 + 5;
};
