#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "WavinController.h"
#include "PrivateConfig.h"

// MQTT defines
// Esp8266 MAC will be added to the device name, to ensure unique topics
// Default is topics like 'heat/floorXXXXXXXXXXXX/3/target', where 3 is the output id and XXXXXXXXXXXX is the mac
const String   MQTT_PREFIX              = "heat/";       // include tailing '/' in prefix
const String   MQTT_DEVICE_NAME         = "floor";       // only alfanumeric and no '/'   
const String   MQTT_ONLINE              = "/online"; 
const String   MQTT_AVAILABILITY        = "/availability"; // availability topic for Home Assistant
const String   MQTT_SUFFIX_TEMPERATURE  = "/temperature";    // include heading '/' in all suffixes
const String   MQTT_SUFFIX_SETPOINT_GET = "/target";
const String   MQTT_SUFFIX_SETPOINT_SET = "/target_set";
const String   MQTT_SUFFIX_MODE_GET     = "/mode";
const String   MQTT_SUFFIX_MODE_SET     = "/mode_set";
const String   MQTT_SUFFIX_BATTERY      = "/battery";
const String   MQTT_SUFFIX_OUTPUT       = "/output";

const String   MQTT_VALUE_MODE_STANDBY  = "off";
const String   MQTT_VALUE_MODE_MANUAL   = "heat";

const String   MQTT_CLIENT = "Wavin-AHC-9000-mqtt";       // mqtt client_id prefix. Will be suffixed with Esp8266 mac to make it unique

String mqttDeviceNameWithMac;
String mqttClientWithMac;
String deviceSwVersion = "1.0.0";

// Operating mode is controlled by the MQTT_SUFFIX_MODE_ topic.
// When mode is set to MQTT_VALUE_MODE_MANUAL, temperature is set to the value of MQTT_SUFFIX_SETPOINT_
// When mode is set to MQTT_VALUE_MODE_STANDBY, the following temperature will be used
const float STANDBY_TEMPERATURE_DEG = 5.0;

const uint8_t TX_ENABLE_PIN = 5;
const bool SWAP_SERIAL_PINS = true;
const uint16_t RECIEVE_TIMEOUT_MS = 1000;
WavinController wavinController(TX_ENABLE_PIN, SWAP_SERIAL_PINS, RECIEVE_TIMEOUT_MS);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastUpdateTime = 0;

const uint16_t POLL_TIME_MS = 5000;

struct lastKnownValue_t {
  uint16_t temperature;
  uint16_t setpoint;
  uint16_t battery;
  uint16_t status;
  uint16_t mode;

  // NEW fields
  uint16_t dewPoint;
  uint16_t humidity;
  uint16_t rssiRaw;
  uint16_t currentRaw;
  uint16_t alarmHigh;
  uint16_t alarmLow;

  uint16_t roomSensor;

  uint16_t heatDemand;
  uint16_t lowBattery;
  uint16_t availability;

} lastSentValues[WavinController::NUMBER_OF_CHANNELS];

struct lastSentSystem_t {
  uint16_t pumpRunning;
  uint16_t pumpMode;
  uint16_t inletTemp;
  uint16_t actuatorInterval;
  uint16_t actuatorDuration;
};

lastSentSystem_t lastSentSystemValues;

const uint16_t LAST_VALUE_UNKNOWN = 0xFFFF;

bool configurationPublished[WavinController::NUMBER_OF_CHANNELS];


// Read a float value from a non zero terminated array of bytes and
// return 10 times the value as an integer
uint16_t temperatureFromString(String payload)
{
  float targetf = payload.toFloat();
  return (unsigned short)(targetf * 10);
}


// Returns temperature in degrees with one decimal
String temperatureAsFloatString(uint16_t temperature)
{
  float temperatureAsFloat = ((float)temperature) / 10;
  return String(temperatureAsFloat, 1);
}


uint8_t getIdFromTopic(char* topic)
{
  unsigned int startIndex = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/").length();
  int i = 0;
  uint8_t result = 0;

  while(topic[startIndex+i] != '/' && i<3)
  {
    result = result * 10 + (topic[startIndex+i]-'0');
    i++;
  }

  return result;
}


void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  String topicString = String(topic);
  
  char terminatedPayload[length+1];
  for(unsigned int i=0; i<length; i++)
  {
    terminatedPayload[i] = payload[i];
  }
  terminatedPayload[length] = 0;
  String payloadString = String(terminatedPayload);

  uint8_t id = getIdFromTopic(topic);

  if(topicString.endsWith(MQTT_SUFFIX_SETPOINT_SET))
  {
    uint16_t target = temperatureFromString(payloadString);
    wavinController.writeRegister(WavinController::CATEGORY_PACKED_DATA, id, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, target);
  }
  else if(topicString.endsWith(MQTT_SUFFIX_MODE_SET))
  {
    if(payloadString == MQTT_VALUE_MODE_MANUAL) 
    {
      wavinController.writeMaskedRegister(
        WavinController::CATEGORY_PACKED_DATA,
        id,
        WavinController::PACKED_DATA_CONFIGURATION,
        WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL,
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
    else if (payloadString == MQTT_VALUE_MODE_STANDBY)
    {
      wavinController.writeMaskedRegister(
        WavinController::CATEGORY_PACKED_DATA, 
        id, 
        WavinController::PACKED_DATA_CONFIGURATION, 
        WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY, 
        ~WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK);
    }
  }

  // Force re-read of registers from controller now
  lastUpdateTime = 0;

}


void resetLastSentValues()
{
  for(int8_t i=0; i<WavinController::NUMBER_OF_CHANNELS; i++)
  {
    lastSentValues[i].temperature = LAST_VALUE_UNKNOWN;
    lastSentValues[i].setpoint = LAST_VALUE_UNKNOWN;
    lastSentValues[i].battery = LAST_VALUE_UNKNOWN;
    lastSentValues[i].status = LAST_VALUE_UNKNOWN;
    lastSentValues[i].mode = LAST_VALUE_UNKNOWN;

    lastSentValues[i].dewPoint   = LAST_VALUE_UNKNOWN;
    lastSentValues[i].humidity   = LAST_VALUE_UNKNOWN;
    lastSentValues[i].rssiRaw    = LAST_VALUE_UNKNOWN;
    lastSentValues[i].currentRaw = LAST_VALUE_UNKNOWN;
    lastSentValues[i].alarmHigh  = LAST_VALUE_UNKNOWN;
    lastSentValues[i].alarmLow   = LAST_VALUE_UNKNOWN;

    lastSentValues[i].roomSensor = LAST_VALUE_UNKNOWN;

    lastSentValues[i].heatDemand = LAST_VALUE_UNKNOWN;
    lastSentValues[i].lowBattery = LAST_VALUE_UNKNOWN;
    lastSentValues[i].availability = LAST_VALUE_UNKNOWN;

    configurationPublished[i] = false;
  }
}

void resetLastSentSystemValues()
{
  lastSentSystemValues.pumpRunning = LAST_VALUE_UNKNOWN;
  lastSentSystemValues.pumpMode = LAST_VALUE_UNKNOWN;
  lastSentSystemValues.inletTemp = LAST_VALUE_UNKNOWN;
  lastSentSystemValues.actuatorInterval = LAST_VALUE_UNKNOWN;
  lastSentSystemValues.actuatorDuration = LAST_VALUE_UNKNOWN;
}


void publishIfNewValue(String topic, String payload, uint16_t newValue, uint16_t *lastSentValue)
{
  if (newValue != *lastSentValue)
  {
    if (mqttClient.publish(topic.c_str(), payload.c_str(), true))
    {
        *lastSentValue = newValue;
    }
    else
    {
      *lastSentValue = LAST_VALUE_UNKNOWN;
    }
  }
}


// Publish discovery messages for HomeAssistant
// See https://www.home-assistant.io/docs/mqtt/discovery/
void publishConfiguration(uint8_t channel)
{
  String channelStr = String(channel);
  String baseStateTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channelStr);
  String availabilityTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_AVAILABILITY);

  String deviceJson = String(
    "{"
      "\"manufacturer\":\"Wavin\","
      "\"model\":\"AHC 9000\","
      "\"sw_version\":\"" + deviceSwVersion + "\","
      "\"name\":\"" + mqttDeviceNameWithMac + "\","
      "\"identifiers\":[\"" + mqttDeviceNameWithMac + "\"]"
    "}"
  );

  // =========================
  // Climate entity
  // =========================
  String climateTopic = String(
    "homeassistant/climate/" + mqttDeviceNameWithMac + "_" + channelStr + "_climate/config"
  );

  String climateMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_climate\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_climate_id\","
      "\"action_topic\":\"" + baseStateTopic + MQTT_SUFFIX_OUTPUT + "\","
      "\"current_temperature_topic\":\"" + baseStateTopic + MQTT_SUFFIX_TEMPERATURE + "\","
      "\"temperature_command_topic\":\"" + baseStateTopic + MQTT_SUFFIX_SETPOINT_SET + "\","
      "\"temperature_state_topic\":\"" + baseStateTopic + MQTT_SUFFIX_SETPOINT_GET + "\","
      "\"mode_command_topic\":\"" + baseStateTopic + MQTT_SUFFIX_MODE_SET + "\","
      "\"mode_state_topic\":\"" + baseStateTopic + MQTT_SUFFIX_MODE_GET + "\","
      "\"modes\":[\"" + String(MQTT_VALUE_MODE_MANUAL) + "\",\"" + String(MQTT_VALUE_MODE_STANDBY) + "\"],"
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"min_temp\":" + String(MIN_TEMP, 1) + ","
      "\"max_temp\":" + String(MAX_TEMP, 1) + ","
      "\"temp_step\":" + String(TEMP_STEP, 1) + ","
      "\"device\":" + deviceJson + ","
      "\"icon\":\"mdi:home-thermometer\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Battery sensor
  // =========================
  String batteryTopic = String(
    "homeassistant/sensor/" + mqttDeviceNameWithMac + "_" + channelStr + "_battery/config"
  );

  String batteryMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_battery\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_battery_id\","
      "\"state_topic\":\"" + baseStateTopic + "/battery\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"device_class\":\"battery\","
      "\"unit_of_measurement\":\"%\","
      "\"state_class\":\"measurement\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Humidity sensor
  // =========================
  String humidityTopic = String(
    "homeassistant/sensor/" + mqttDeviceNameWithMac + "_" + channelStr + "_humidity/config"
  );

  String humidityMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_humidity\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_humidity_id\","
      "\"state_topic\":\"" + baseStateTopic + "/humidity\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"unit_of_measurement\":\"%\","
      "\"device_class\":\"humidity\","
      "\"state_class\":\"measurement\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Dew point sensor
  // =========================
  String dewPointTopic = String(
    "homeassistant/sensor/" + mqttDeviceNameWithMac + "_" + channelStr + "_dew_point/config"
  );

  String dewPointMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_dew_point\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_dew_point_id\","
      "\"state_topic\":\"" + baseStateTopic + "/dew_point\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"unit_of_measurement\":\"°C\","
      "\"device_class\":\"temperature\","
      "\"state_class\":\"measurement\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // RSSI sensor
  // =========================
  String rssiTopic = String(
    "homeassistant/sensor/" + mqttDeviceNameWithMac + "_" + channelStr + "_rssi/config"
  );

  String rssiMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_rssi\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_rssi_id\","
      "\"state_topic\":\"" + baseStateTopic + "/rssi\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"unit_of_measurement\":\"dBm\","
      "\"device_class\":\"signal_strength\","
      "\"state_class\":\"measurement\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Channel current sensor
  // =========================
  String currentTopic = String(
    "homeassistant/sensor/" + mqttDeviceNameWithMac + "_" + channelStr + "_current/config"
  );

  String currentMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_current\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_current_id\","
      "\"state_topic\":\"" + baseStateTopic + "/channel_current\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"unit_of_measurement\":\"mA\","
      "\"device_class\":\"current\","
      "\"state_class\":\"measurement\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Alarm high binary sensor
  // =========================
  String alarmHighTopic = String(
    "homeassistant/binary_sensor/" + mqttDeviceNameWithMac + "_" + channelStr + "_alarm_high/config"
  );

  String alarmHighMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_alarm_high\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_alarm_high_id\","
      "\"state_topic\":\"" + baseStateTopic + "/alarm_high\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"payload_on\":\"True\","
      "\"payload_off\":\"False\","
      "\"device_class\":\"problem\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Alarm low binary sensor
  // =========================
  String alarmLowTopic = String(
    "homeassistant/binary_sensor/" + mqttDeviceNameWithMac + "_" + channelStr + "_alarm_low/config"
  );

  String alarmLowMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_alarm_low\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_" + channelStr + "_alarm_low_id\","
      "\"state_topic\":\"" + baseStateTopic + "/alarm_low\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"payload_on\":\"True\","
      "\"payload_off\":\"False\","
      "\"device_class\":\"problem\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  String roomSensorTopic = "homeassistant/sensor/" + mqttDeviceNameWithMac + "_" + channel + "_room_sensor/config";

  String roomSensorMessage =
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_" + channel + "_room_sensor\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_" + channel + "_room_sensor\","
      "\"state_topic\":\"" + MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/room_sensor\","
      "\"availability_topic\":\"" + MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_AVAILABILITY + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"icon\":\"mdi:home-group\","
      "\"device_class\": null,"
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}";

  // Publish discovery
  mqttClient.publish(climateTopic.c_str(), climateMessage.c_str(), true);
  mqttClient.publish(batteryTopic.c_str(), batteryMessage.c_str(), true);
  mqttClient.publish(humidityTopic.c_str(), humidityMessage.c_str(), true);
  mqttClient.publish(dewPointTopic.c_str(), dewPointMessage.c_str(), true);
  mqttClient.publish(rssiTopic.c_str(), rssiMessage.c_str(), true);
  mqttClient.publish(currentTopic.c_str(), currentMessage.c_str(), true);
  mqttClient.publish(alarmHighTopic.c_str(), alarmHighMessage.c_str(), true);
  mqttClient.publish(alarmLowTopic.c_str(), alarmLowMessage.c_str(), true);
  mqttClient.publish(roomSensorTopic.c_str(), roomSensorMessage.c_str(), true);

  configurationPublished[channel] = true;
}

void publishSystemConfiguration()
{
  String availabilityTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_AVAILABILITY);
  String baseStateTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/system");

  String deviceJson = String(
    "{"
      "\"manufacturer\":\"Wavin\","
      "\"model\":\"AHC 9000\","
      "\"sw_version\":\"" + deviceSwVersion + "\","
      "\"name\":\"" + mqttDeviceNameWithMac + "\","
      "\"identifiers\":[\"" + mqttDeviceNameWithMac + "\"]"
    "}"
  );

  // =========================
  // Pump running (binary sensor)
  // =========================
  String pumpRunningTopic = String(
    "homeassistant/binary_sensor/" + mqttDeviceNameWithMac + "_pump_running/config"
  );

  String pumpRunningMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_pump_running\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_pump_running_id\","
      "\"state_topic\":\"" + baseStateTopic + "/pump_running\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"payload_on\":\"True\","
      "\"payload_off\":\"False\","
      "\"device_class\":\"running\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Pump mode (sensor)
  // =========================
  String pumpModeTopic = String(
    "homeassistant/sensor/" + mqttDeviceNameWithMac + "_pump_mode/config"
  );

  String pumpModeMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_pump_mode\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_pump_mode_id\","
      "\"state_topic\":\"" + baseStateTopic + "/pump_mode\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"icon\":\"mdi:pump\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Inlet temperature (sensor)
  // =========================
  String inletTempTopic = String(
    "homeassistant/sensor/" + mqttDeviceNameWithMac + "_inlet_temperature/config"
  );

  String inletTempMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_inlet_temperature\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_inlet_temperature_id\","
      "\"state_topic\":\"" + baseStateTopic + "/inlet_temperature\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"unit_of_measurement\":\"°C\","
      "\"device_class\":\"temperature\","
      "\"state_class\":\"measurement\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Actuator motion interval (sensor)
  // =========================
  String actuatorIntervalTopic = String(
    "homeassistant/sensor/" + mqttDeviceNameWithMac + "_actuator_motion_interval/config"
  );

  String actuatorIntervalMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_actuator_motion_interval\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_actuator_motion_interval_id\","
      "\"state_topic\":\"" + baseStateTopic + "/actuator_motion_interval\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"unit_of_measurement\":\"s\","
      "\"state_class\":\"measurement\","
      "\"icon\":\"mdi:timer-cog\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // =========================
  // Actuator motion duration (sensor)
  // =========================
  String actuatorDurationTopic = String(
    "homeassistant/sensor/" + mqttDeviceNameWithMac + "_actuator_motion_duration/config"
  );

  String actuatorDurationMessage = String(
    "{"
      "\"name\":\"" + mqttDeviceNameWithMac + "_actuator_motion_duration\","
      "\"unique_id\":\"" + mqttDeviceNameWithMac + "_actuator_motion_duration_id\","
      "\"state_topic\":\"" + baseStateTopic + "/actuator_motion_duration\","
      "\"availability_topic\":\"" + availabilityTopic + "\","
      "\"payload_available\":\"True\","
      "\"payload_not_available\":\"False\","
      "\"unit_of_measurement\":\"s\","
      "\"state_class\":\"measurement\","
      "\"icon\":\"mdi:timer-play-outline\","
      "\"device\":" + deviceJson + ","
      "\"entity_category\":\"diagnostic\","
      "\"qos\":0"
    "}"
  );

  // Publish discovery
  mqttClient.publish(pumpRunningTopic.c_str(), pumpRunningMessage.c_str(), true);
  mqttClient.publish(pumpModeTopic.c_str(), pumpModeMessage.c_str(), true);
  mqttClient.publish(inletTempTopic.c_str(), inletTempMessage.c_str(), true);
  mqttClient.publish(actuatorIntervalTopic.c_str(), actuatorIntervalMessage.c_str(), true);
  mqttClient.publish(actuatorDurationTopic.c_str(), actuatorDurationMessage.c_str(), true);
}

String buildDeviceVersionString()
{
    String hwVersion = "unknown";
    String swVersion = "unknown";
    uint16_t reg[1];

    if (wavinController.readRegisters(WavinController::CATEGORY_INFO, 0,
                                      WavinController::INFO_HW_VERSION, 1, reg))
    {
        hwVersion = "MC110" + String(reg[0] & WavinController::INFO_HW_VERSION_MASK);
    }

    if (wavinController.readRegisters(WavinController::CATEGORY_INFO, 0,
                                      WavinController::INFO_SW_VERSION, 1, reg))
    {
        swVersion = "MC610" + String((reg[0] >> 4) & WavinController::INFO_SW_VERSION_MASK);

        uint8_t betaVersion = reg[0] & WavinController::INFO_SW_BETA_VERSION_MASK;
        if (betaVersion) {
            swVersion += "b" + String(betaVersion);
        }
    }

    return hwVersion + " / " + swVersion;
}


void setup()
{
  uint8_t mac[6];
  WiFi.macAddress(mac);

  char macStr[13] = {0};
  sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  mqttDeviceNameWithMac = String(MQTT_DEVICE_NAME + macStr);
  mqttClientWithMac = String(MQTT_CLIENT + macStr);

  mqttClient.setServer(MQTT_SERVER.c_str(), MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}


void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID.c_str(), WIFI_PASS.c_str());

    if (WiFi.waitForConnectResult() != WL_CONNECTED) return;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    if (!mqttClient.connected())
    {
      String will = String(MQTT_PREFIX + mqttDeviceNameWithMac + MQTT_ONLINE);
      if (mqttClient.connect(mqttClientWithMac.c_str(), MQTT_USER.c_str(), MQTT_PASS.c_str(), will.c_str(), 1, true, "False") )
      {
          String setpointSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+" + MQTT_SUFFIX_SETPOINT_SET);
          mqttClient.subscribe(setpointSetTopic.c_str(), 1);
          
          String modeSetTopic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/+" + MQTT_SUFFIX_MODE_SET);
          mqttClient.subscribe(modeSetTopic.c_str(), 1);
          
          mqttClient.publish(will.c_str(), (const uint8_t *)"True", 4, true);

          // Forces resending of all parameters to server
          resetLastSentValues();

          deviceSwVersion = buildDeviceVersionString();

          publishSystemConfiguration();
      }
      else
      {
          return;
      }
    }
  
    // Process incomming messages and maintain connection to the server
    if(!mqttClient.loop())
    {
        return;
    }

    if (lastUpdateTime + POLL_TIME_MS < millis())
    {
      lastUpdateTime = millis();

      // =========================
      // SYSTEM VALUES (cached)
      // =========================

      // -------- Pump state --------
      uint8_t tevent;
      if (wavinController.getPumpState(tevent))
      {
        bool running = (
            tevent == WavinController::RELAY_EVENT_OUTPUT_ON ||
            tevent == WavinController::RELAY_EVENT_STOP_DELAY ||
            tevent == WavinController::RELAY_EVENT_PERIODIC_CYCLE
        );

        // pump_running (binary)
        {
          String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/system/pump_running");

          publishIfNewValue(
              topic,
              running ? "True" : "False",
              running ? 1 : 0,
              &(lastSentSystemValues.pumpRunning)
          );
        }

        // pump_mode (string, cache by tevent)
        {
          String mode = "idle";
          if (tevent == WavinController::RELAY_EVENT_OUTPUT_ON) mode = "heating";
          else if (tevent == WavinController::RELAY_EVENT_STOP_DELAY) mode = "stop_delay";
          else if (tevent == WavinController::RELAY_EVENT_PERIODIC_CYCLE) mode = "exercise";

          String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/system/pump_mode");

          publishIfNewValue(
              topic,
              mode,
              tevent,   // ✅ cache raw event
              &(lastSentSystemValues.pumpMode)
          );
        }
      }

      // -------- Inlet temperature --------
      float inletTemp;
      if (wavinController.getInletTemperature(inletTemp))
      {
          uint16_t inletTempRaw = (uint16_t)(inletTemp * 10.0f);

          String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/system/inlet_temperature");
          String payload = String(inletTemp, 1);

          publishIfNewValue(
              topic,
              payload,
              inletTempRaw,
              &(lastSentSystemValues.inletTemp)
          );
      }

      // -------- Actuator motion --------
      uint16_t interval, duration;
      if (wavinController.getActuatorMotion(interval, duration))
      {
          // interval
          {
              String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/system/actuator_motion_interval");

              publishIfNewValue(
                  topic,
                  String(interval),
                  interval,
                  &(lastSentSystemValues.actuatorInterval)
              );
          }

          // duration
          {
              String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/system/actuator_motion_duration");

              publishIfNewValue(
                  topic,
                  String(duration),
                  duration,
                  &(lastSentSystemValues.actuatorDuration)
              );
          }
      }

      uint16_t registers[11];

      for(uint8_t channel = 0; channel < WavinController::NUMBER_OF_CHANNELS; channel++)
      {
        if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_PRIMARY_ELEMENT, 1, registers))
        {
          uint16_t primaryElement = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ELEMENT_MASK;
          bool allThermostatsLost = registers[0] & WavinController::CHANNELS_PRIMARY_ELEMENT_ALL_TP_LOST_MASK;
          bool alarmHigh = registers[0] & WavinController::CH_PRI_ALARM_HIGH;
          bool alarmLow  = registers[0] & WavinController::CH_PRI_ALARM_LOW;

          // ==========================
          // Room sensor mapping (NEW)
          // ==========================
          {
            // Convert to 0-based index (0 means "not used")
            uint16_t elementIndex = (primaryElement > 0) ? (primaryElement - 1) : 0xFFFF;

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/room_sensor");

            String payload;
            if (primaryElement == 0)
            {
              payload = "none";
            }
            else
            {
              payload = String(elementIndex);
            }

            publishIfNewValue(
                topic,
                payload,
                elementIndex,
                &(lastSentValues[channel].roomSensor)
            );
          }

          if(primaryElement==0)
          {
              // Channel not used
              continue;
          }

          if(!configurationPublished[channel])
          {
            uint16_t standbyTemperature = STANDBY_TEMPERATURE_DEG * 10;
            wavinController.writeRegister(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_STANDBY_TEMPERATURE, standbyTemperature);

            publishConfiguration(channel);
          }

          // Read the f setpoint programmed for channel
          if (wavinController.readRegisters(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_MANUAL_TEMPERATURE, 1, registers))
          {
            uint16_t setpoint = registers[0];

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_SETPOINT_GET);
            String payload = temperatureAsFloatString(setpoint);

            publishIfNewValue(topic, payload, setpoint, &(lastSentValues[channel].setpoint));
          }

          // Read the current mode for the channel
          if (wavinController.readRegisters(WavinController::CATEGORY_PACKED_DATA, channel, WavinController::PACKED_DATA_CONFIGURATION, 1, registers))
          {
            uint16_t mode = registers[0] & WavinController::PACKED_DATA_CONFIGURATION_MODE_MASK; 

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_MODE_GET);
            if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_STANDBY)
            {
              publishIfNewValue(topic, MQTT_VALUE_MODE_STANDBY, mode, &(lastSentValues[channel].mode));
            }
            else if(mode == WavinController::PACKED_DATA_CONFIGURATION_MODE_MANUAL)
            {
              publishIfNewValue(topic, MQTT_VALUE_MODE_MANUAL, mode, &(lastSentValues[channel].mode));
            }            
          }

          // Read the current status of the output for channel
          if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_TIMER_EVENT, 1, registers))
          {
            uint16_t status = registers[0] & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK;

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_OUTPUT);
            String payload;
            if (status & WavinController::CHANNELS_TIMER_EVENT_OUTP_ON_MASK)
              payload = "heating";
            else
              payload = "off";

            publishIfNewValue(topic, payload, status, &(lastSentValues[channel].status));
          }
          
          // ==========================
          // Channel alarms (NEW)
          // ==========================
          {
            uint16_t alarmHighValue = alarmHigh ? 1 : 0;
            uint16_t alarmLowValue  = alarmLow  ? 1 : 0;

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/alarm_high");
            publishIfNewValue(topic,
                              alarmHigh ? "True" : "False",
                              alarmHighValue,
                              &(lastSentValues[channel].alarmHigh));

            topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/alarm_low");
            publishIfNewValue(topic,
                              alarmLow ? "True" : "False",
                              alarmLowValue,
                              &(lastSentValues[channel].alarmLow));
          }

          // ==========================
          // Channel current (NEW)
          // ==========================
          if (wavinController.readRegisters(WavinController::CATEGORY_CHANNELS, channel, WavinController::CHANNELS_CURRENT_CONSUMPTION, 1, registers))
          {
            uint16_t currentRaw = registers[0];

            String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/channel_current");
            String payload = String(currentRaw * 0.54f, 2);

            publishIfNewValue(topic,
                              payload,
                              currentRaw,
                              &(lastSentValues[channel].currentRaw));
          }

          // If a thermostat for the channel is connected to the controller
          if(!allThermostatsLost)
          {
            // Read values from the primary thermostat connected to this channel
            // Primary element from controller is returned as index+1, so 1 is subtracted here
            if (wavinController.readRegisters(WavinController::CATEGORY_ELEMENTS, primaryElement-1, 0, 11, registers))
            {
              uint16_t temperature = registers[WavinController::EL_AIR_TEMP];
              uint16_t dewPoint    = registers[WavinController::EL_DEW_POINT];
              uint16_t humidity    = registers[WavinController::EL_HUMIDITY];
              uint16_t rssiRaw     = registers[WavinController::EL_RSSI];
              uint16_t battery     = registers[WavinController::EL_BATTERY]; // In 10% steps
              uint16_t status      = registers[WavinController::EL_STATUS];

              // Temperature
              String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_TEMPERATURE);
              String payload = temperatureAsFloatString(temperature);
              publishIfNewValue(topic, payload, temperature, &(lastSentValues[channel].temperature));

              // Battery
              topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + MQTT_SUFFIX_BATTERY);
              payload = String(battery * 10);
              publishIfNewValue(topic, payload, battery, &(lastSentValues[channel].battery));

              // Dew point
              if (dewPoint != LAST_VALUE_UNKNOWN)
              {
                topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/dew_point");
                payload = temperatureAsFloatString(dewPoint);

                publishIfNewValue(topic,
                                  payload,
                                  dewPoint,
                                  &(lastSentValues[channel].dewPoint));
              }

              // Humidity
              if (humidity != LAST_VALUE_UNKNOWN)
              {
                topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/humidity");
                payload = String(humidity);

                publishIfNewValue(topic,
                                  payload,
                                  humidity,
                                  &(lastSentValues[channel].humidity));
              }

              // RSSI
              {
                int8_t rssiByte = rssiRaw & 0xFF;
                float rssi = -74.0f + (rssiByte * 0.5f);

                topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/rssi");
                payload = String(rssi, 1);

                publishIfNewValue(topic,
                                  payload,
                                  rssiRaw,
                                  &(lastSentValues[channel].rssiRaw));
              }

              // Decode bits (see Modbus spec 1.3.10)
              bool alive     = status & WavinController::ELEMENT_STATUS_ALIVE;
              bool lost      = status & WavinController::ELEMENT_STATUS_LOST;
              bool lowBatt   = status & WavinController::ELEMENT_STATUS_LOW_BATT;
              bool heatCall  = status & WavinController::ELEMENT_STATUS_TP_ACT;

              // -------- heat_demand --------
              {
                String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/heat_demand");

                publishIfNewValue(
                    topic,
                    heatCall ? "True" : "False",
                    heatCall ? 1 : 0,
                    &(lastSentValues[channel].heatDemand)
                );
              }

              // -------- low_battery --------
              {
                String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/low_battery");

                publishIfNewValue(
                    topic,
                    lowBatt ? "True" : "False",
                    lowBatt ? 1 : 0,
                    &(lastSentValues[channel].lowBattery)
                );
              }

              // -------- availability (REAL, replaces hardcoded "online") --------
              {
                bool available = (alive && !lost);

                String topic = String(MQTT_PREFIX + mqttDeviceNameWithMac + "/" + channel + "/availability");

                publishIfNewValue(
                    topic,
                    available ? "True" : "False",
                    available ? 1 : 0,
                    &(lastSentValues[channel].availability)
                );
              }

            }
          }
        }

        // Process incomming messages and maintain connection to the server
        if(!mqttClient.loop())
        {
            return;
        }
      }
    }
  }
}
