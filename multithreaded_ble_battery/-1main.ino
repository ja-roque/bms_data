/*
 * Xiaoxiang BMS to MQTT via WiFi
 * by Bas Vermulst - https://github.com/BeaverUI/ESP32-BluetoothBMS2MQTT
 * 
 * Based on original work from https://github.com/kolins-cz/Smart-BMS-Bluetooth-ESP32/blob/master/README.md
 * 
 
   === configuring ===
   Using the #define parameters in the CONFIGURATION section, do the following:
   1) configure WiFi via WIFI_SSID and WIFI_PASSWORD
   2) configure MQTT broker via MQTTSERVER
   3) set unique node name via NODE_NAME
   4) ensure the BMS settings are OK. You can verify the name and address using the "BLE Scanner" app on an Android phone.
   

   === compiling ===
   1) Add ESP-WROVER to the board manager:
   - File --> Preferences, add to board manager URLs: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   - Then, Tools --> Board --> Board manager... --> install ESP32 package v2.0.3 or later.
   

   2) Install required libraries:
   - MQTT (by Joel Gaehwiler)
   Install via "Manage Libraries..." under Tools.

   3) Configure board:
   Select Tools --> Board --> "ESP32 Arduino" --> "ESP WRover module"

   4) Connect board via USB.
   
   5) Configure programming options:
   Select appropriate programming port under Tools --> Port
   Select correct partitioning: 1.9MB app (with OTA) or 2MB app with 2 MB SPIFFS - otherwise it won't fit

   6) Program and go!
 
 */
 
// ==== CONFIGURATION ====
// BMS
#define BMS_MAX_CELLS 15 // defines size of data types
#define BMS_POLLING_INTERVAL 10*60*1000 // data output interval (shorter = connect more often = more battery consumption from BMS) in ms

// BLE
#define BLE_MIN_RSSI -95 // minimum signal strength before connection is attempted
#define BLE_NAME "Juanli" // name of BMS
#define BLE_ADDRESS "a5:c2:37:30:cb:1a" // address of BMS

#define BLE_SCAN_DURATION 1 // duration of scan in seconds
#define BLE_REQUEST_DELAY 500 // package request delay after connecting - make this large enough to have the connection established in ms
#define BLE_TIMEOUT 10*1000 // timeout of scan + gathering packets (too short will fail collecting all packets) in ms

#define BLE_CALLBACK_DEBUG true // send debug messages via MQTT & serial in callbacks (handy for finding your BMS address, name, RSSI, etc)

// MQTT
#define MQTTSERVER "homeassistant.local"
#define MQTT_USERNAME "javier" // leave empty if no credentials are needed
#define MQTT_PASSWORD "acmilan18" 
#define NODE_NAME "bms2mqtt"

// WiFi
#define WIFI_SSID "Roque Google"
#define WIFI_PASSWORD "famroque21"

// watchdog timeout
#define WATCHDOG_TIMEOUT (BLE_TIMEOUT+10*1000) // go to sleep after x seconds





// ==== MAIN CODE ====
#include "datatypes.h" // for brevity the BMS stuff is in this file
#include <WiFi.h> // for WiFi
#include <BLEDevice.h> // for BLE
#include <MQTT.h> // for MQTT
#include <WiFiClient.h> // for MQTT
#include <WebServer.h> // for web server
#include <ESPmDNS.h> // for mDNS
#include <ArduinoJson.h> // for JSON handling

#include <driver/adc.h> // to read ESP battery voltage
#include <rom/rtc.h> // to get reset reason

#include "ble_globals.h"

// Web server configuration
WebServer server(80);
const char* hostname = "battery";

// Init BMS
static BLEUUID serviceUUID("0000ff00-0000-1000-8000-00805f9b34fb"); //xiaoxiang bms service
static BLEUUID charUUID_rx("0000ff01-0000-1000-8000-00805f9b34fb"); //xiaoxiang bms rx id
static BLEUUID charUUID_tx("0000ff02-0000-1000-8000-00805f9b34fb"); //xiaoxiang bms tx id

const byte cBasicInfo = 3; //datablock 3=basic info
const byte cCellInfo = 4;  //datablock 4=individual cell info
packBasicInfoStruct packBasicInfo;
packCellInfoStruct packCellInfo;
unsigned long bms_last_update_time=0;
bool bms_status;
#define BLE_PACKETSRECEIVED_BEFORE_STANDBY 0b11 // packets to gather before disconnecting


WiFiClient wificlient_mqtt;
MQTTClient mqttclient;


// Other stuff
float battery_voltage=0; // internal battery voltage
String debug_log_string="";
hw_timer_t * wd_timer = NULL;


void setup(){
  Serial.begin(115200);
  
  // Initialize BLE first
  bleStart();  // Initialize BLE stack
  
  // Try to get initial BMS data
  bool initial_success = false;
  int retry_count = 0;
  while (!initial_success && retry_count < 3) {
    handleBLE();  // This will scan, connect, and gather data
    if (true) {
      initial_success = true;
      Serial.println("Successfully got initial BMS data");
    } else {
      retry_count++;
      Serial.printf("Failed to get BMS data, attempt %d/3\n", retry_count);
      delay(1000);
    }
  }

  // Now start WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Attempting to connect to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nConnected");
  Serial.println("IP address: " + IPAddressString(WiFi.localIP()));

  // Initialize mDNS
  if (!MDNS.begin(hostname)) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    Serial.println("mDNS responder started");
    Serial.printf("You can now connect to http://%s.local\n", hostname);
  }

  // Setup web server routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data.json", HTTP_GET, handleJsonData);
  server.begin();
  Serial.println("HTTP server started");

  // Start MQTT
  mqttclient.begin(MQTTSERVER, wificlient_mqtt);
  mqttclient.onMessage(handleMQTTreceive);
  mqttclient.setKeepAlive(3);
  mqttclient.setTimeout(3000);
  MQTTconnect();
  
  // Send initial data if we have it
  if (true) {
    handleMQTT();
    MqttDebug("Initial BMS data sent");
  } else {
    MqttDebug("Warning: No initial BMS data available");
  }
}


// === Main stuff ====
void loop(){
  static unsigned long last_update = 0;
  static unsigned long last_mqtt_attempt = 0;
  const unsigned long UPDATE_INTERVAL = 2000; // Start with 10 seconds for testing
  const unsigned long MQTT_RETRY_INTERVAL = 30000; // Try to reconnect every 30 seconds
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.reconnect();
    delay(1000);
    return;
  }

  // Check MQTT connection with retry interval
  if (!mqttclient.connected() && (millis() - last_mqtt_attempt >= MQTT_RETRY_INTERVAL)) {
    last_mqtt_attempt = millis();
    MQTTconnect();
  }

  // Handle BLE and MQTT updates at regular intervals
  if (millis() - last_update >= UPDATE_INTERVAL) {
    Serial.println("Starting BMS update cycle...");
    
    // Reset BLE state and try to gather new data
    blePause();
    handleBLE();
    
    // If we got data, publish it
    if (true) {
      handleMQTT();
      MqttDebug("Successfully updated BMS data");
      Serial.println("BMS update successful");
    } else {
      MqttDebug("Failed to get BMS update");
      Serial.println("BMS update failed");
    }
    
    last_update = millis();
  }

  // Process any incoming MQTT messages
  if (mqttclient.connected()) {
    mqttclient.loop();
  }

  // Handle web server requests
  server.handleClient();
  
  delay(100); // Small delay to prevent tight looping
}


// enable watchdog timer -- a very ugly hack to overcome crashes of the BLE stack
// (desperate times ask for desperate measures)
void enableWatchdogTimer(){
  wd_timer = timerBegin(80);  
  timerAttachInterrupt(wd_timer, &WatchDogTimeoutHandler);
}


// WDT handler to put ESP in deep sleep after data has been obtained
void ARDUINO_ISR_ATTR WatchDogTimeoutHandler()
{ 
  // esp_sleep_enable_timer_wakeup((BMS_POLLING_INTERVAL - WATCHDOG_TIMEOUT) * 1e3); // standby period is in ms, function accepts us
  // esp_deep_sleep_start(); // sweet dreams
}


// read voltage of onboard battery
void getEspBatteryVoltage(void){
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_11);

  battery_voltage = ((float) 2*adc1_get_raw(ADC1_CHANNEL_7)*(3.3*1.06/4095));
}


// ===== Handles for MQTT =====
// handle connection and send messages at intervals
void handleMQTT(void){
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("WiFi disconnected. Skipping MQTT update.");
    return;
  }
  
  if (!mqttclient.connected()){
    MQTTconnect();
  }

  // Send MQTT message
  Serial.println("Sending MQTT update");

  mqttclient.publish(GetTopic("ip"), IPAddressString(WiFi.localIP()));
  mqttclient.publish(GetTopic("free-heap"), String(ESP.getFreeHeap()));
  mqttclient.publish(GetTopic("maxalloc-heap"), String(xPortGetMinimumEverFreeHeapSize()));
  mqttclient.publish(GetTopic("ssid"), WiFi.SSID());
  mqttclient.publish(GetTopic("rssi"), String(WiFi.RSSI()));
  
  mqttclient.publish(GetTopic("reset-reason"), String(GetResetReason(0)) + String(" | ") + String(GetResetReason(1)));
  
  mqttclient.publish(GetTopic("runtime"), String(millis()/1000));
  mqttclient.publish(GetTopic("battery-voltage"), String(battery_voltage,2));

  mqttclient.publish(GetTopic("bms-status"), String(bms_status));
  mqttclient.publish(GetTopic("bms-status-age"), String( (millis()-bms_last_update_time)/1000 ));

  if(true){
    // Cell information
    mqttclient.publish(GetTopic("number-of-cells"), String(packCellInfo.NumOfCells));
    mqttclient.publish(GetTopic("cell-diff"), String((float)packCellInfo.CellDiff, 0));
    
    // Basic voltage and current
    mqttclient.publish(GetTopic("current"), String((float)packBasicInfo.Amps / 1000, 2));
    mqttclient.publish(GetTopic("voltage"), String((float)packBasicInfo.Volts / 1000, 2));
    mqttclient.publish(GetTopic("power"), String((float)packBasicInfo.Watts, 2));
    
    // Average cell voltage
    if(packCellInfo.NumOfCells != 0){
      mqttclient.publish(GetTopic("cell-voltage"), String((float)packBasicInfo.Volts /(1000*packCellInfo.NumOfCells), 2));
    }
    
    // Capacity information
    mqttclient.publish(GetTopic("capacity-remain"), String((float)packBasicInfo.CapacityRemainAh, 2));
    mqttclient.publish(GetTopic("capacity-nominal"), String((float)packBasicInfo.NominalCapacityAh, 2));
    mqttclient.publish(GetTopic("soc"), String((float)packBasicInfo.CapacityRemainPercent, 1));
    
    // Temperature readings
    mqttclient.publish(GetTopic("temperature-1"), String((float)packBasicInfo.Temp1 / 10, 1));
    mqttclient.publish(GetTopic("temperature-2"), String((float)packBasicInfo.Temp2 / 10, 1));
    
    // Balance codes
    mqttclient.publish(GetTopic("balance-code-low"), String(packBasicInfo.BalanceCodeLow, HEX));
    mqttclient.publish(GetTopic("balance-code-high"), String(packBasicInfo.BalanceCodeHigh, HEX));
    
    // MOSFET status
    mqttclient.publish(GetTopic("mosfet-status"), String(packBasicInfo.MosfetStatus, HEX));
    
    // Remaining time information
    mqttclient.publish(GetTopic("time-to-discharge-hours"), String(packBasicInfo.RemainingTimeHoursToDischarge));
    mqttclient.publish(GetTopic("time-to-discharge-minutes"), String(packBasicInfo.RemainingTimeMinutesToDischarge));
    mqttclient.publish(GetTopic("time-to-charge-hours"), String(packBasicInfo.RemainingTimeHoursToCharge));
    mqttclient.publish(GetTopic("time-to-charge-minutes"), String(packBasicInfo.RemainingTimeMinutesToCharge));
  }

  MqttPublishDebug();
  
  mqttclient.loop();
}


// handler for incoming messages
void handleMQTTreceive(String &topic, String &payload) {
}


// ===== Helper functions =====
String GetResetReason(int core)
{
  return String("POWERON_RESET");
}

// Web server handlers
void handleRoot() {
  String html = "<html><head><title>BMS Data</title></head><body>";
  html += "<h1>BMS Data</h1>";
  html += "<p>Click <a href='/data.json'>here</a> to view data in JSON format</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleJsonData() {
  StaticJsonDocument<2048> doc;
  
  // System info
  doc["system"]["ip"] = IPAddressString(WiFi.localIP());
  doc["system"]["free_heap"] = ESP.getFreeHeap();
  doc["system"]["maxalloc_heap"] = xPortGetMinimumEverFreeHeapSize();
  doc["system"]["ssid"] = WiFi.SSID();
  doc["system"]["rssi"] = WiFi.RSSI();
  doc["system"]["runtime"] = millis()/1000;
  doc["system"]["battery_voltage"] = battery_voltage;
  
  // BMS status
  doc["bms"]["status"] = bms_status;
  doc["bms"]["last_update"] = (millis()-bms_last_update_time)/1000;
  
  if(true) {
    // Cell information
    doc["cells"]["count"] = packCellInfo.NumOfCells;
    doc["cells"]["voltage_diff"] = (float)packCellInfo.CellDiff;
    
    // Basic voltage and current
    doc["power"]["current"] = (float)packBasicInfo.Amps / 1000;
    doc["power"]["voltage"] = (float)packBasicInfo.Volts / 1000;
    doc["power"]["watts"] = (float)packBasicInfo.Watts;
    
    // Average cell voltage
    if(packCellInfo.NumOfCells != 0) {
      doc["cells"]["average_voltage"] = (float)packBasicInfo.Volts /(1000*packCellInfo.NumOfCells);
    }
    
    // Capacity information
    doc["capacity"]["remaining_ah"] = (float)packBasicInfo.CapacityRemainAh;
    doc["capacity"]["nominal_ah"] = (float)packBasicInfo.NominalCapacityAh;
    doc["capacity"]["percentage"] = (float)packBasicInfo.CapacityRemainPercent;
    
    // Temperature readings
    doc["temperature"]["sensor1"] = (float)packBasicInfo.Temp1 / 10;
    doc["temperature"]["sensor2"] = (float)packBasicInfo.Temp2 / 10;
    
    // Balance codes
    doc["balance"]["code_low"] = packBasicInfo.BalanceCodeLow;
    doc["balance"]["code_high"] = packBasicInfo.BalanceCodeHigh;
    
    // MOSFET status
    doc["mosfet"]["status"] = packBasicInfo.MosfetStatus;
    
    // Remaining time information
    doc["time"]["discharge_hours"] = packBasicInfo.RemainingTimeHoursToDischarge;
    doc["time"]["discharge_minutes"] = packBasicInfo.RemainingTimeMinutesToDischarge;
    doc["time"]["charge_hours"] = packBasicInfo.RemainingTimeHoursToCharge;
    doc["time"]["charge_minutes"] = packBasicInfo.RemainingTimeMinutesToCharge;
  }

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}
