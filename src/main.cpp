/*
  readFID Solo. Basic-Authentication Firmware flavor.
  RFID Reader with embedded antenna and WiFi connectivity.

  Copyright (C) 2020  MOVASIM (https://movasim.com/)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 =================== 
  = Sensor MFRC522 =
  ==================
  The MFRC522 is a highly integrated reader/writer IC for contactless communication at 13.56 MHz.
  The MFRC522 reader supports ISO/IEC 14443 A/MIFARE and NTAG.
  connections:
  - SPI SS -> D4
  - SCK -> D5
  - SPI MOSI -> D7
  - SPI MISO -> D6
  - RST/Reset -> D3
  - VCC -> 3V3
  - GND -> GND
  
  ==========================
  = Real Time Clock DS3231 =
  ==========================
  The DS3231 is a extremely accurate I2C real-time clock (RTC) with integrated temperature compensated
  crystal oscillator. The device incorporates a battery input.
  connections:
  - VCC -> 3V3
  - GND -> GND
  - SCL -> SCL
  - SDA -> SDA

  ================
  = PIEZO BUZZER =
  ================
  Piezo buzzers are simple devices that can generate basic beeps and tones.
  connections:
  - VCC -> D8
  - GND -> GND

  ==========
  = Jumper =
  ==========
  The Juper (JP1) is used to switch enviroSense into Debug/Production mode.
  connections:
  - D1 -> GND: Debug Mode
  - D1 -> N/C: Production Mode 

  Debug Mode: Useful information will be published in the Serial Port.
  Production Mode. Few infromation will be published to the Serial Port.

*/

#include <Arduino.h>                // https://github.com/arduino/ArduinoCore-avr
#include <ESP8266WiFi.h>            // https://github.com/esp8266/Arduino
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager/tree/development
#include <PubSubClient.h>           // https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>            // https://github.com/bblanchon/ArduinoJson
#include <SPI.h>                    // https://github.com/arduino/ArduinoCore-avr
#include <MFRC522.h>                // https://github.com/miguelbalboa/rfid
#include "RTClib.h"                 // https://github.com/adafruit/RTClib
#include "mqtt.configuration.h"     // MQTT file configuration.

// ========== Start Device User Parametrization ================================================================

#define Alias "Production";                     // Friendly name for the Reader location.
unsigned int mqttDeviceReportPeriod = 600000;    // Device Report Period (Miliseconds).
int resetPortal = 180;                          // Number of seconds until the WiFiManager resests ESP8266.
#define AP_Password "readFID.movasim"           // AP password.

// ========== Start Device Development Parametrization (ONLY MODIFY WHEN NEW HW/FW VERSION IS RELASED) =========

#define DeviceType "readFID" 
#define DeviceModel "Solo"
#define DeviceVersion "1.0.0"
#define FirmwareFlavor "Basic-Auth"
#define FirmwareVersion "1.0.0"
#define JUMPER 10
#define SCL D1
#define SDA D2
#define SS_PIN D4
#define RST_PIN D3
#define BUZZER D8
#define LED D0

// ========== End Device Parametrization =======================================================================

const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;
const char* mqtt_server = MQTT_SERVER;
IPAddress mqtt_server_ip(MQTT_SERVER_IP);
int mqtt_server_port = MQTT_SERVER_PORT;
const char* mqtt_clientId;
const char* mqtt_publish_topic = MQTT_PUBLISH_TOPIC;
const char* mqtt_subscribe_topic = MQTT_SUBSCRIBE_TOPIC;
unsigned long currentTime;
unsigned long previousTime = 0;
String strClientId; 
bool debug;

// Helper functions declarations
String dump_byte_array_to_string(byte *buffer, byte bufferSize);

String GetDeviceName(void);
String GetMyMACAddress(void);
void reconnectMqttBroker(void);
void messageReceived(char* p_topic, byte* p_payload, unsigned int p_length);
void publishSensorsData(String tag_ID, String tag_Type, uint32_t timeStamp);
void publishDeviceData(String dev_t, String dev_m, String dev_v, String fw_f, String fw_v, String mac, String ip, byte s_qty, unsigned long up, uint32_t timestamp, float temperature, String rst_r, unsigned int free_heap, byte heap_frg);

// Create  object of the class.
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
RTC_DS3231 rtc; // Create RTC instance.
WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
  Serial.begin(115200); // Initialize serial communications with the PC
  while (!Serial);      // Wait for hardware serial to appear.
  SPI.begin();          // Init SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522

  // Check JUMPER Status.
  pinMode(JUMPER, INPUT_PULLUP);
  Serial.println();
  if (digitalRead(JUMPER) == LOW)
  {
    debug = true;
    Serial.println("DEBUG Mode ON");
  } else
  {
    debug = false;
    Serial.println("DEBUG Mode OFF");
  }

 // Generate ClientID with Device name and MAC.
  strClientId = GetDeviceName();
  mqtt_clientId = strClientId.c_str();

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // wifiManager.resetSettings(); //Reset settings - wipe credentials for testing

  if (debug == true)
  {
    wifiManager.setDebugOutput(true); // Send debugging info to serial port.
  } else
  {
    wifiManager.setDebugOutput(false);
  }

  wifiManager.setConfigPortalTimeout(resetPortal); // auto close configportal after n seconds

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "ES-XXXX with the last 4 digits of the MAC Address"),
  // then goes into a blocking loop awaiting configuration and will return success result
  String APName = GetDeviceName();
  bool res;
  res = wifiManager.autoConnect((const char*)APName.c_str(),AP_Password); // password protected ap
  if(!res)
  {
    Serial.println("Failed to connect");
    ESP.restart();
  } 
  else
  {
    // If you get here you have connected to the WiFi    
    Serial.println("Connected to the WiFi Network!");
  }

  // Setup the MQTT Client
  client.setServer(mqtt_server_ip, mqtt_server_port); // Connect to the MQTT Broker using IP
  //client.setServer(mqtt_server, mqtt_server_port); // Connect to the MQTT Broker using URL
  client.setBufferSize(2048);
  client.setCallback(messageReceived);

  /* read and printout the MFRC522 version (valid values 0x91 & 0x92)*/
  Serial.print(F("Ver: 0x"));
  byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.println(readReg, HEX);

  if (! rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  pinMode(LED, OUTPUT);     // Initialize LED pin as output.
  digitalWrite(LED, HIGH);  // Initialize the LED off by making the voltage HIGH.
  pinMode(BUZZER, OUTPUT);  // Initialize BUZZER pin as output.

  Serial.println(F("End setup"));
}

void loop()
{
  currentTime = millis();
  DateTime now;
  
  // Check if MQTT connection is active. Otherwise reconnect.
  if (!client.connected())
    {
      reconnectMqttBroker();
    }
  client.loop();

  if (currentTime - previousTime >= mqttDeviceReportPeriod)
      {
        previousTime = currentTime;

        String deviceType = DeviceType;
        String deviceModel = DeviceModel;
        String deviceVersion = DeviceVersion;
        String firmwareFlavor = FirmwareFlavor;
        String firmwareVersion = FirmwareVersion;
        String deviceMAC = GetMyMACAddress();
        String deviceIP = WiFi.localIP().toString();
        now = rtc.now();                                // Get timestamp.
        uint32_t timeStamp = now.unixtime();
        float temperature = rtc.getTemperature();       // Get temperature in Celsius Degrees.

        // Device WiFi Signal Quality
        byte signalQuality = 0; 
        long rssi = WiFi.RSSI();
        // dBm to Signal Quality [%]:
          if(rssi <= -100)
              signalQuality = 0;
          else if(rssi >= -50)
              signalQuality = 100;
          else
              signalQuality = 2 * (rssi + 100);

        unsigned long deviceUptime = millis(); // Device Uptime
        String deviceResetReason = ESP.getResetReason(); // Returns a String containing the last reset reason in human readable format.
        unsigned int deviceFreeHeap = ESP.getFreeHeap(); // Returns the free heap size.
        byte deviceHeapFragmentation = ESP.getHeapFragmentation(); // Returns the fragmentation metric (0% is clean, more than ~50% is not harmless)

        publishDeviceData(deviceType, deviceModel, deviceVersion, firmwareFlavor, firmwareVersion, deviceMAC, deviceIP, signalQuality, deviceUptime, timeStamp, temperature, deviceResetReason, deviceFreeHeap, deviceHeapFragmentation);
      }

    // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
    if ( ! mfrc522.PICC_IsNewCardPresent())
        return;

    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial())
        return;

    String tag_ID = dump_byte_array_to_string(mfrc522.uid.uidByte, mfrc522.uid.size);   // Get Tag ID.
    MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);                // Get Tag Type.
    String tag_Type = mfrc522.PICC_GetTypeName(piccType);

    now = rtc.now();
    uint32_t timeStamp = now.unixtime();                                // Get timestamp in UNIX format.
    
    publishSensorsData(tag_ID, tag_Type, timeStamp);

    digitalWrite(LED, LOW);   // Turn the LED on by making the voltage LOW
    digitalWrite(BUZZER, HIGH);   // Turn the BUZZER on by making the voltage HIGH
    delay(250);
    digitalWrite(LED, HIGH);  // Turn the LED off by making the voltage HIGH
    digitalWrite(BUZZER, LOW);   // Turn the BUZZER off by making the voltage LOW
    delay(1000);
}

// Helper function definitions

// Helper routine to dump a byte array as hex values to String.
String dump_byte_array_to_string(byte *buffer, byte bufferSize) {
  String tagID = "0x";
  for (byte i = 0; i < bufferSize; i++) {
    tagID += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
    tagID += String(mfrc522.uid.uidByte[i], HEX);
  }
  return(tagID);
}

// Function to get the Device Name. Device name is DeviceType + the last 4 MAC characters.
// This Device Name is used for WiFi SSID and MQTT clientId.
String GetDeviceName()
{
  String ssid1 = DeviceType;
  uint8_t mac[6];
  char macStr[6] = {0};
  String ssidDeviceName;
  WiFi.macAddress(mac);
  sprintf(macStr, "%02X%02X", mac[4], mac[5]);
  ssidDeviceName = ssid1 + String(macStr);
  return  ssidDeviceName;  
}

// Function to get the Device MAC Address.
String GetMyMACAddress()
{
  uint8_t mac[6];
  char macStr[18] = {0};
  WiFi.macAddress(mac);
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],  mac[1], mac[2], mac[3], mac[4], mac[5]); 
  return  String(macStr);
}

//Function to connect to MQTT Broker.
void reconnectMqttBroker()
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection");
    // Attempt to connect
      if (client.connect(mqtt_clientId, mqtt_user, mqtt_password))
      {
      if (debug == true)
      {
        Serial.print(" with clientID ");
        Serial.print(mqtt_clientId);
      } 
      Serial.println("...Connected!");
      // Subscribe to topics
      client.subscribe(mqtt_subscribe_topic);
      if (debug == true)
      {
        Serial.print("Subscribed to topic: ");
        Serial.println(mqtt_subscribe_topic);
      }
    } else
    {
      Serial.print("...failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// function called when a MQTT message arrived.
void messageReceived(char* topic, byte* payload, unsigned int length)
{
  if (debug == true)
  {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i=0;i<length;i++) 
    {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  }

  // Deserialize the JSON document
  DynamicJsonDocument jsonReceivedCommand(2048);
  DeserializationError error = deserializeJson(jsonReceivedCommand, payload);

  // Test if parsing succeeds.
  if (error)
  {
    if (debug == true)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }
    return;
  }
  
  // Fetch values.
  if(jsonReceivedCommand["value"].as<String>() == "restart")
  {
    if (debug == true)
    {
      Serial.print("Disconnecting from ");
      Serial.println(mqtt_server);
    }
    client.disconnect();
    if (debug == true)
    {
      Serial.println("ESP8266 is going to be restarted");
    }
    delay (1000);
    ESP.restart();
  }
}

// function called to publish Sensors data (Temperature, Pressure, Humidity, IAQ and Lux).
void publishSensorsData(String tag_ID, String tag_Type, uint32_t timeStamp)
{
  StaticJsonDocument<128> jsonSensorsData;
  jsonSensorsData["msg_type"] = "srs";
  // BME680
  jsonSensorsData["tag_id"] = tag_ID;
  jsonSensorsData["tag_type"] = tag_Type;
  jsonSensorsData["ts"] = timeStamp;
  jsonSensorsData["alias"] = Alias;
  
  char buffer[128];
  serializeJson(jsonSensorsData, buffer);
  if (debug == true)
  {
    Serial.println(buffer);
  }
  client.publish(mqtt_publish_topic, buffer, true);
  yield();
}

// function called to publish Device information (Type, Model, Version, Firmware Flavor, Firmware version, MAC, IP, WiFi Signal Quality, Uptime, etc.).
void publishDeviceData(String dev_t, String dev_m, String dev_v, String fw_f, String fw_v, String mac, String ip, byte s_qty, unsigned long up, uint32_t timeStamp, float temperature, String rst_r, unsigned int free_heap, byte heap_frg)
{
  StaticJsonDocument<512> jsonDeviceData;
  jsonDeviceData["msg_type"] = "dev";
  // Device
  jsonDeviceData["dev_t"] = dev_t;
  jsonDeviceData["dev_m"] = dev_m;
  jsonDeviceData["dev_v"] = dev_v;
  jsonDeviceData["fw_f"] = fw_f;
  jsonDeviceData["fw_v"] = fw_v;
  jsonDeviceData["mac"] = mac;
  jsonDeviceData["ip"] = ip;
  jsonDeviceData["s_qty"] = s_qty;
  jsonDeviceData["up"] = up;
  jsonDeviceData["ts"] = timeStamp;
  jsonDeviceData["t"] = temperature;
  jsonDeviceData["rst_r"] = rst_r;
  jsonDeviceData["free_heap"] = free_heap;
  jsonDeviceData["heap_frg"] = heap_frg;

  char buffer[512];
  serializeJson(jsonDeviceData, buffer);
  if (debug == true)
  {
    Serial.println(buffer);
  }
  client.publish(mqtt_publish_topic, buffer, true);
  yield();
}
