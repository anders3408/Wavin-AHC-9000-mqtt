#include <ESP8266WiFi.h>

const String   WIFI_SSID = "UniFi";         // wifi ssid
const String   WIFI_PASS = "berkohund";     // wifi password

const String   MQTT_SERVER = "192.168.2.2"; // mqtt server address without port number
const String   MQTT_USER   = "";       // mqtt user. Use "" for no username
const String   MQTT_PASS   = "";       // mqtt password. Use "" for no password
const uint16_t MQTT_PORT   = 49154;                             // mqtt port

const float    MIN_TEMP    = 6.0;                              // minimum temperature to set
const float    MAX_TEMP    = 40.0;                             // maximum temperature to set
const float    TEMP_STEP   = 0.5;                              // the temperature step size, either 0.5 or 1.0
