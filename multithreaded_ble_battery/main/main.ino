/*
 * Multithreaded ESP32 BLE Battery Monitor
 * Based on original work from https://github.com/BeaverUI/ESP32-BluetoothBMS2MQTT
 * 
 * This implementation uses two threads:
 * 1. Web server thread - Always accessible at battery.local
 * 2. BLE communication thread - Handles BLE scanning, connecting and data retrieval
 *
 * Features:
 * - Supports multiple BLE battery devices (configurable)
 * - Real-time status updates via web interface
 * - JSON API for integration with other systems
 * - Connection status monitoring for both WiFi and BLE
 */

// ================ CONFIGURATION ================
// WIFI
#define WIFI_SSID "Roque Google"
#define WIFI_PASSWORD "famroque21"

// MQTT Configuration
#define MQTT_BROKER "164.92.75.55"
#define MQTT_PORT 1883
#define MQTT_TOPIC "bms/data"
#define MQTT_CLIENT_ID "ESP32-BMS"
#define MQTT_USERNAME "bms_mqtt"  // Add if your broker requires authentication
#define MQTT_PASSWORD "bms_mqtt_password"  // Add if your broker requires authentication

// Debug Settings
#define SERIAL_DEBUG true  // Set to true to enable debug messages over serial

// API Endpoint
#define API_ENDPOINT "http://164.92.75.55:4567/api"  // External API endpoint
#define API_UPDATE_INTERVAL 2000                      // API update interval (ms)

// BLE Device 1
#define BLE1_NAME "JUANLI2"               // BMS name
#define BLE1_ADDRESS "a5:c2:37:30:cb:3e" // BMS MAC address
#define BLE1_ENABLED true                // Set to false to disable this device

// BLE Device 2 (uncomment and configure if needed)
#define BLE2_NAME "JUANLI3"                 // BMS name
#define BLE2_ADDRESS "a5:c2:37:30:cc:48" // BMS MAC address
#define BLE2_ENABLED true               // Set to true to enable this device

// BLE Settings
#define BLE_SCAN_DURATION 1              // Duration of scan in seconds
#define BLE_MIN_RSSI -95                 // Minimum signal strength before connection is attempted
#define BLE_REQUEST_DELAY 500            // Package request delay after connecting (ms)
#define BLE_TIMEOUT 10*1000              // Timeout for BLE operations (ms)
#define BLE_UPDATE_INTERVAL 2000         // BLE data update interval (ms)

// Task Settings
#define BLE_TASK_STACK_SIZE 4096         // Stack size for BLE task
#define MQTT_TASK_STACK_SIZE 4096        // Stack size for MQTT task

// Battery data
#define BMS_MAX_CELLS 15                 // Maximum number of cells

// Add this near the top with other defines
#define MIN_HEAP_SIZE 16000  // Minimum heap size in bytes before restart
#define HEAP_CHECK_INTERVAL 5000  // Check heap every 5 seconds
#define MQTT_FAILURE_TIMEOUT 300000  // 5 minutes in milliseconds
#define MQTT_FAILURE_CHECK_INTERVAL 60000  // Check MQTT failure every minute

// Add this in the CONFIGURATION section
#define USE_STATIC_IP false  // Set to true to use static IP, false for DHCP

// MQTT Resilience Settings
#define MQTT_RECONNECT_DELAY_MIN 5000      // Start with 5 second delay
#define MQTT_RECONNECT_DELAY_MAX 60000     // Max 1 minute delay (60000ms)
#define MQTT_BUFFER_SIZE 10                // Number of messages to buffer
#define MQTT_WATCHDOG_TIMEOUT 600000       // 10 minutes without connection triggers restart
#define MQTT_PING_INTERVAL 30000           // Send ping every 30 seconds

// ================ INCLUDE LIBRARIES ================
#include <WiFi.h>
#include <HTTPClient.h>
#include <BLEDevice.h>
#include <ArduinoJson.h>
#include <driver/adc.h>
#include <MQTT.h>
#include <esp_wifi.h>

// ================ DATA TYPES ================
// BMS Packet Header
typedef struct {
    byte start;
    byte type;
    byte status;
    byte dataLen;
} bmsPacketHeaderStruct;

// BMS Protection Status
typedef struct {
    bool CellBlockOverVoltage;
    bool CellBlockUnderVoltage;
    bool BatteryOverVoltage;
    bool BatteryUnderVoltage;
    bool ChargingOverTemp;
    bool ChargingUnderTemp;
    bool DischargingOverTemp;
    bool DischargingUnderTemp;
    bool ChargingOverCurr;
    bool DischargingOverCurr;
    bool ShortCircuit;
    bool ICError;
    bool MOSLockIn;
} bmsProtectionStatusStruct;

// BMS Basic Info
typedef struct {
    uint16_t Volts;                      // unit 1mV
    int16_t Amps;                        // unit 1mA
    int32_t Watts;                       // unit 1W
    uint16_t CapacityRemainAh;
    uint16_t NominalCapacityAh;
    uint8_t CapacityRemainPercent;       // unit 1%
    uint16_t Temp1;                      // unit 0.1C
    uint16_t Temp2;                      // unit 0.1C
    uint16_t BalanceCodeLow;
    uint16_t BalanceCodeHigh;
    uint8_t MosfetStatus;
    uint16_t RemainingTimeHoursToDischarge;
    uint16_t RemainingTimeMinutesToDischarge;
    uint16_t RemainingTimeHoursToCharge;
    uint16_t RemainingTimeMinutesToCharge;
    // New fields from SmartBMSUtility
    uint16_t CycleLife;
    uint16_t ProductDate;
    bool BalanceCells[32];               // Balance status for each cell
    bmsProtectionStatusStruct Protection;
    uint8_t Version;
    uint8_t RSOC;
    uint8_t ControlStatus;
    bool ChargingPort;
    bool DischargingPort;
    uint8_t NumberOfCells;
    uint8_t NumberOfTempSensors;
    int16_t TemperatureReadings[4];      // Support up to 4 temperature sensors
} packBasicInfoStruct;

// BMS Cell Info
typedef struct {
    uint8_t NumOfCells;
    uint16_t CellVolt[BMS_MAX_CELLS];    // cell 1 has index 0
    uint16_t CellMax;
    uint16_t CellMin;
    uint16_t CellDiff;                   // difference between highest and lowest
    uint16_t CellAvg;
} packCellInfoStruct;

// BLE Device Structure
typedef struct {
    String name;                         // BMS name
    String address;                      // BMS MAC address
    bool enabled;                        // Whether this device is enabled
    BLEClient* client;                   // BLE client instance
    BLEAdvertisedDevice* device;         // Discovered BLE device
    BLERemoteService* service;           // BMS service
    BLERemoteCharacteristic* rxChar;     // Receive characteristic
    BLERemoteCharacteristic* txChar;     // Transmit characteristic
    bool connected;                      // Connection status
    unsigned long lastUpdate;            // Last update timestamp
    packBasicInfoStruct basicInfo;       // Basic info data
    packCellInfoStruct cellInfo;         // Cell info data
    uint8_t packetsReceived;             // Bit flag for received packets
} BleDeviceInfo;

// Status Message Structure
typedef struct {
    String message;
    unsigned long timestamp;
    bool isError;
} StatusMessage;

// ================ GLOBAL VARIABLES ================
// BLE UUIDs
static BLEUUID serviceUUID("0000ff00-0000-1000-8000-00805f9b34fb"); // Xiaoxiang BMS service
static BLEUUID charUUID_rx("0000ff01-0000-1000-8000-00805f9b34fb"); // Xiaoxiang BMS RX
static BLEUUID charUUID_tx("0000ff02-0000-1000-8000-00805f9b34fb"); // Xiaoxiang BMS TX

// BMS Command Codes
const byte cBasicInfo = 3; // Command code for basic info
const byte cCellInfo = 4;  // Command code for cell info

// Task Handles
TaskHandle_t bleTaskHandle = NULL;
TaskHandle_t mqttTaskHandle = NULL;

// Device info structs - support for up to 2 BLE devices
BleDeviceInfo bleDevices[2];

// Status messages
#define MAX_STATUS_MESSAGES 10
StatusMessage statusMessages[MAX_STATUS_MESSAGES];
int messageIndex = 0;
SemaphoreHandle_t statusMutex;

// Data sharing between tasks
SemaphoreHandle_t dataMutex;
bool newDataAvailable = false;

// ESP32 battery voltage
float espBatteryVoltage = 0;

// System status
bool wifiConnected = false;
String systemStartTime;
unsigned long bootTime;

// MQTT Client
WiFiClient wifiClient;
MQTTClient mqttClient;

// Add these at the top with other global variables
#define MAX_PACKET_SIZE 64
uint8_t packetBuffer[2][MAX_PACKET_SIZE];
uint8_t packetLength[2] = {0, 0};

// Add this with other global variables
unsigned long lastSuccessfulMqttPublish = 0;
unsigned long lastMqttFailureCheck = 0;

// MQTT Message Buffer
struct MqttMessage {
    String payload;
    unsigned long timestamp;
    bool valid;
};

MqttMessage mqttBuffer[MQTT_BUFFER_SIZE];
int mqttBufferIndex = 0;
unsigned long lastMqttPing = 0;
unsigned long lastConnectAttempt = 0;
int connectFailCount = 0;

// Function Prototypes
void bleTask(void *pvParameters);
void mqttTask(void *pvParameters);
void addStatusMessage(String message, bool isError = false);
bool connectToBleDevice(BleDeviceInfo &device);
bool sendBleCommand(BleDeviceInfo &device, uint8_t command);
bool processBmsPacket(uint8_t *packet, BleDeviceInfo &device);
bool processBasicInfo(packBasicInfoStruct *output, byte *data, unsigned int dataLen);
bool processCellInfo(packCellInfoStruct *output, byte *data, unsigned int dataLen);
bool isPacketValid(byte *packet);
int16_t twoIntsInto16(int highbyte, int lowbyte);
String getFormattedTime(unsigned long milliseconds);
void getEspBatteryVoltage();
String ipAddressToString(IPAddress address);
void sendDataToApi(BleDeviceInfo &device);
String createJsonPayload(BleDeviceInfo &device);
void sendDataToMqtt(BleDeviceInfo &device);
void connectMqtt();
bool checkBit(byte data, int pos);
bool hasProtectionOrPortStatus(BleDeviceInfo &device);
void addProtectionAndPortStatus(JsonObject &basicInfo, BleDeviceInfo &device);
void bufferMessage(const String& message);
void sendBufferedMessages();

// ================ BLE CALLBACKS ================
// BLE Advertisement Callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        // Check device 1
        if (bleDevices[0].enabled && 
            advertisedDevice.getName() == bleDevices[0].name && 
            advertisedDevice.getAddress().toString() == bleDevices[0].address &&
            advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(serviceUUID)) {
            
            if (advertisedDevice.getRSSI() >= BLE_MIN_RSSI) {
                // Store the device and stop scanning
                if (bleDevices[0].device != nullptr) {
                    delete bleDevices[0].device;
                }
                bleDevices[0].device = new BLEAdvertisedDevice(advertisedDevice);
                addStatusMessage("Found BLE device 1: " + bleDevices[0].name);
                Serial.println("Device 1 RSSI: " + String(advertisedDevice.getRSSI()));
                Serial.println("Device 1 Service UUID: " + advertisedDevice.getServiceUUID().toString());
            } else {
                addStatusMessage("BLE device 1 signal too weak: " + String(advertisedDevice.getRSSI()) + " dBm", true);
            }
        }
        
        // Check device 2
        if (bleDevices[1].enabled && 
            advertisedDevice.getName() == bleDevices[1].name && 
            advertisedDevice.getAddress().toString() == bleDevices[1].address &&
            advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(serviceUUID)) {
            
            if (advertisedDevice.getRSSI() >= BLE_MIN_RSSI) {
                // Store the device and stop scanning
                if (bleDevices[1].device != nullptr) {
                    delete bleDevices[1].device;
                }
                bleDevices[1].device = new BLEAdvertisedDevice(advertisedDevice);
                addStatusMessage("Found BLE device 2: " + bleDevices[1].name);
            } else {
                addStatusMessage("BLE device 2 signal too weak: " + String(advertisedDevice.getRSSI()) + " dBm", true);
            }
        }
    }
};

// BLE Client Callback
class MyClientCallback : public BLEClientCallbacks {
private:
    int deviceIndex;
    
public:
    MyClientCallback(int index) : deviceIndex(index) {}
    
    void onConnect(BLEClient* client) {
        addStatusMessage("Connected to " + bleDevices[deviceIndex].name);
        bleDevices[deviceIndex].connected = true;
    }
    
    void onDisconnect(BLEClient* client) {
        addStatusMessage("Disconnected from " + bleDevices[deviceIndex].name);
        bleDevices[deviceIndex].connected = false;
    }
};

// Data Notification callback - separate class for each device
class Device1NotifyCallback {
public:
    static void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
        Serial.println("Device 1 notification received");
        Serial.print("Data length: ");
        Serial.println(length);
        Serial.print("Data: ");
        for(int i = 0; i < length; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        // Check if this is the start of a new packet
        if (data[0] == 0xDD) {
            packetLength[0] = 0;  // Reset buffer
        }
        
        // Copy data to buffer
        if (packetLength[0] + length <= MAX_PACKET_SIZE) {
            memcpy(packetBuffer[0] + packetLength[0], data, length);
            packetLength[0] += length;
            
            // Check if we have a complete packet
            if (packetLength[0] >= 4 && packetBuffer[0][3] + 5 <= packetLength[0]) {
                // We have a complete packet, process it
                if (packetBuffer[0][0] == 0xDD && (packetBuffer[0][1] == 0x03 || packetBuffer[0][1] == 0x04)) {
                    processBmsPacket(packetBuffer[0], bleDevices[0]);
                }
                packetLength[0] = 0;  // Reset buffer for next packet
            }
        } else {
            // Buffer overflow, reset
            packetLength[0] = 0;
        }
    }
};

class Device2NotifyCallback {
public:
    static void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
        Serial.println("Device 2 notification received");
        Serial.print("Data length: ");
        Serial.println(length);
        Serial.print("Data: ");
        for(int i = 0; i < length; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
        // Check if this is the start of a new packet
        if (data[0] == 0xDD) {
            packetLength[1] = 0;  // Reset buffer
        }
        
        // Copy data to buffer
        if (packetLength[1] + length <= MAX_PACKET_SIZE) {
            memcpy(packetBuffer[1] + packetLength[1], data, length);
            packetLength[1] += length;
            
            // Check if we have a complete packet
            if (packetLength[1] >= 4 && packetBuffer[1][3] + 5 <= packetLength[1]) {
                // We have a complete packet, process it
                if (packetBuffer[1][0] == 0xDD && (packetBuffer[1][1] == 0x03 || packetBuffer[1][1] == 0x04)) {
                    processBmsPacket(packetBuffer[1], bleDevices[1]);
                }
                packetLength[1] = 0;  // Reset buffer for next packet
            }
        } else {
            // Buffer overflow, reset
            packetLength[1] = 0;
        }
    }
};

// ================ MAIN SETUP ================
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("\n\nStarting BLE Battery Monitor with API Forwarding");
    
    // Record boot time
    bootTime = millis();
    systemStartTime = getFormattedTime(0);
    
    // Print initial memory info
    Serial.print("Initial free heap: ");
    Serial.println(ESP.getFreeHeap());
    
    // Create mutexes
    statusMutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    
    // Initialize status messages
    addStatusMessage("System starting...");
    
    // Initialize device structures
    // Device 1
    bleDevices[0].name = BLE1_NAME;
    bleDevices[0].address = BLE1_ADDRESS;
    bleDevices[0].enabled = BLE1_ENABLED;
    bleDevices[0].client = nullptr;
    bleDevices[0].device = nullptr;
    bleDevices[0].service = nullptr;
    bleDevices[0].rxChar = nullptr;
    bleDevices[0].txChar = nullptr;
    bleDevices[0].connected = false;
    bleDevices[0].lastUpdate = 0;
    bleDevices[0].packetsReceived = 0;
    
    // Device 2
    bleDevices[1].name = BLE2_NAME;
    bleDevices[1].address = BLE2_ADDRESS;
    bleDevices[1].enabled = BLE2_ENABLED;
    bleDevices[1].client = nullptr;
    bleDevices[1].device = nullptr;
    bleDevices[1].service = nullptr;
    bleDevices[1].rxChar = nullptr;
    bleDevices[1].txChar = nullptr;
    bleDevices[1].connected = false;
    bleDevices[1].lastUpdate = 0;
    bleDevices[1].packetsReceived = 0;
    
    // Initialize BLE
    BLEDevice::init("");
    addStatusMessage("BLE initialized");
    
    // Initialize WiFi with more robust connection handling
    setupWiFi();  // Call our new WiFi setup function
    
    // Start BLE task on Core 1
    xTaskCreatePinnedToCore(
        bleTask,            // Task function
        "BLETask",          // Name of task
        BLE_TASK_STACK_SIZE,  // Stack size
        NULL,               // Parameter to pass
        1,                  // Priority
        &bleTaskHandle,     // Task handle
        1                   // Core (0 or 1)
    );
    
    // Start MQTT task on Core 0
    xTaskCreatePinnedToCore(
        mqttTask,           // Task function
        "MQTTTask",         // Name of task
        MQTT_TASK_STACK_SIZE, // Stack size
        NULL,               // Parameter to pass
        1,                  // Priority
        &mqttTaskHandle,    // Task handle
        0                   // Core (0 or 1)
    );
    
    // Read ESP32 battery voltage
    getEspBatteryVoltage();
}

void loop() {
    // Main loop is empty as all functionality is in tasks
    delay(1000);
}

// ================ BLE TASK ================
void bleTask(void *pvParameters) {
    addStatusMessage("BLE task started");
    
    // Create BLE scan object
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    
    while (true) {
        // Process each enabled device
        for (int i = 0; i < 2; i++) {
            if (!bleDevices[i].enabled) continue;
            
            // If we're not connected, scan for the device
            if (!bleDevices[i].connected) {
                Serial.println("Scanning for " + bleDevices[i].name);
                addStatusMessage("Scanning for " + bleDevices[i].name);
                pBLEScan->start(BLE_SCAN_DURATION, false);
                delay(1000); // Wait for scan to complete
                
                // If device was found during scan, try to connect
                if (bleDevices[i].device != nullptr) {
                    // Create client if needed
                    if (bleDevices[i].client == nullptr) {
                        bleDevices[i].client = BLEDevice::createClient();
                        bleDevices[i].client->setClientCallbacks(new MyClientCallback(i));
                    }
                    
                    // Connect to the device
                    if (connectToBleDevice(bleDevices[i])) {
                        addStatusMessage("Successfully connected to " + bleDevices[i].name);
                        bleDevices[i].lastUpdate = millis();
                        
                        // Send command to get basic info
                        if (sendBleCommand(bleDevices[i], cBasicInfo)) {
                            addStatusMessage("Requested basic info from " + bleDevices[i].name);
                            delay(1000); // Wait for response
                            
                            // Also request cell info
                            if (sendBleCommand(bleDevices[i], cCellInfo)) {
                                addStatusMessage("Requested cell info from " + bleDevices[i].name);
                            }
                        } else {
                            addStatusMessage("Failed to request data from " + bleDevices[i].name, true);
                        }
                    } else {
                        addStatusMessage("Failed to connect to " + bleDevices[i].name, true);
                    }
                }
            } 
            // If connected, check if it's time to update data
            else if (millis() - bleDevices[i].lastUpdate >= BLE_UPDATE_INTERVAL) {
                // Send command to get basic info
                if (sendBleCommand(bleDevices[i], cBasicInfo)) {
                    bleDevices[i].lastUpdate = millis();
                    delay(1000); // Wait for response
                    
                    // Also request cell info
                    if (sendBleCommand(bleDevices[i], cCellInfo)) {
                        addStatusMessage("Requested cell info from " + bleDevices[i].name);
                    }
                } else {
                    // Command failed, disconnect and try again later
                    if (bleDevices[i].client != nullptr && bleDevices[i].client->isConnected()) {
                        bleDevices[i].client->disconnect();
                    }
                    bleDevices[i].connected = false;
                    addStatusMessage("Lost connection to " + bleDevices[i].name, true);
                }
            }
        }
        
        // Small delay to avoid consuming all CPU
        delay(100);
    }
}

// Connect to a BLE device
bool connectToBleDevice(BleDeviceInfo &device) {
    // Ensure we have a client and device
    if (device.client == nullptr || device.device == nullptr) {
        Serial.println("Connect failed: No client or device");
        return false;
    }
    
    // Connect to the device
    if (!device.client->connect(device.device)) {
        Serial.println("Connect failed: Could not connect to device");
        return false;
    }
    
    // Get the service
    device.service = device.client->getService(serviceUUID);
    if (device.service == nullptr) {
        Serial.println("Connect failed: Could not get service");
        device.client->disconnect();
        return false;
    }
    
    // Get the characteristics
    device.rxChar = device.service->getCharacteristic(charUUID_rx);
    device.txChar = device.service->getCharacteristic(charUUID_tx);
    
    if (device.rxChar == nullptr || device.txChar == nullptr) {
        Serial.println("Connect failed: Could not get characteristics");
        device.client->disconnect();
        return false;
    }
    
    // Register notification handler
    if (device.rxChar->canNotify()) {
        Serial.println("Registering notification callback");
        if (&device == &bleDevices[0]) {
            device.rxChar->registerForNotify(Device1NotifyCallback::notifyCallback);
        } else {
            device.rxChar->registerForNotify(Device2NotifyCallback::notifyCallback);
        }
    } else {
        Serial.println("Connect failed: Cannot notify");
        device.client->disconnect();
        return false;
    }
    
    // Wait a bit for the connection to stabilize
    delay(BLE_REQUEST_DELAY);
    
    return true;
}

// Send a command to the BLE device
bool sendBleCommand(BleDeviceInfo &device, uint8_t command) {
    if (!device.connected || device.client == nullptr || !device.client->isConnected() || device.txChar == nullptr) {
        Serial.println("Send command failed: Device not ready");
        return false;
    }
    
    // Create command packet
    uint8_t length = 0x00;
    uint8_t dataByte = 0xFF;

    uint16_t sum = command + length;
    uint8_t checksum = (~sum + 1) & 0xFF;
    uint8_t data[7] = {0xdd, 0xa5, command, length, dataByte, checksum, 0x77};
    
    Serial.print("Sending command: ");
    for(int i = 0; i < 7; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    
    // Write to the device
    device.txChar->writeValue(data, sizeof(data), false);
    
    return true;
}

// ================ MQTT TASK ================
void mqttTask(void *pvParameters) {
    addStatusMessage("MQTT task started");
    mqttClient.begin(MQTT_BROKER, MQTT_PORT, wifiClient);
    
    unsigned long lastHeapCheck = 0;
    
    while (true) {
        // Current time
        unsigned long now = millis();
        
        // Check heap size
        if (now - lastHeapCheck >= HEAP_CHECK_INTERVAL) {
            uint32_t freeHeap = ESP.getFreeHeap();
            if (freeHeap < MIN_HEAP_SIZE) {
                ESP.restart();
            }
            lastHeapCheck = now;
        }
        
        // Check WiFi status and reconnect if needed
        if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
            setupWiFi();
            delay(1000);
            continue;
        }
        
        // MQTT Connection management
        if (!mqttClient.connected()) {
            if (now - lastSuccessfulMqttPublish >= MQTT_WATCHDOG_TIMEOUT) {
                Serial.println("MQTT watchdog timeout, restarting...");
                ESP.restart();
            }
            connectMqtt();
        } else {
            // Send periodic ping to keep connection alive
            if (now - lastMqttPing >= MQTT_PING_INTERVAL) {
                if (mqttClient.publish(MQTT_TOPIC "/ping", "ping")) {
                    lastMqttPing = now;
                }
            }
            
            // Process MQTT messages
            mqttClient.loop();
            
            // Try to send any buffered messages
            sendBufferedMessages();
        }
        
        // Process new data if available
        if (xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE) {
            if (newDataAvailable) {
                for (int i = 0; i < 2; i++) {
                    if (bleDevices[i].enabled && (bleDevices[i].packetsReceived & 0x01)) {
                        sendDataToMqtt(bleDevices[i]);
                    }
                }
                newDataAvailable = false;
            }
            xSemaphoreGive(dataMutex);
        }
        
        delay(100); // Prevent watchdog triggers
    }
}

// Connect to MQTT broker
bool connectMqtt() {
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected, can't establish MQTT connection");
        return false;
    }

    // Calculate backoff delay
    unsigned long currentDelay = min(
        MQTT_RECONNECT_DELAY_MIN * (1 << min(connectFailCount, 6)), // Exponential backoff
        MQTT_RECONNECT_DELAY_MAX
    );
    
    // Check if we should attempt reconnection
    if (millis() - lastConnectAttempt < currentDelay) {
        return false;
    }
    
    Serial.print("Attempting MQTT connection... ");
    lastConnectAttempt = millis();
    
    // Create a unique client ID
    String clientId = "ESP32-BMS-";
    clientId += String(random(0xffff), HEX);
    
    // Set MQTT client options
    mqttClient.setKeepAlive(60); // Keepalive timeout in seconds
    mqttClient.setConnectionTimeout(10); // Connection timeout in seconds
    
    if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.println("connected!");
        connectFailCount = 0;
        lastSuccessfulMqttPublish = millis(); // Reset the watchdog
        
        // After successful connection, try to send buffered messages
        sendBufferedMessages();
        return true;
    } else {
        connectFailCount++;
        Serial.print("failed, rc=");
        Serial.print(mqttClient.lastError());
        Serial.print(" try again in ");
        Serial.print(currentDelay / 1000);
        Serial.println(" seconds");
        return false;
    }
}

// Send data to MQTT broker
void sendDataToMqtt(BleDeviceInfo &device) {
    if (!wifiConnected) {
        Serial.println("WiFi not connected, buffering message");
        bufferMessage(createMqttPayload(device));
        return;
    }
    
    String payload = createMqttPayload(device);
    
    // Try to publish immediately
    if (mqttClient.connected() && mqttClient.publish(MQTT_TOPIC, payload)) {
        if (SERIAL_DEBUG) {
            Serial.println("MQTT message published successfully");
        }
        lastSuccessfulMqttPublish = millis();
    } else {
        // If publish fails, buffer the message
        Serial.println("MQTT publish failed, buffering message");
        bufferMessage(payload);
    }
}

// Helper function to create MQTT payload
String createMqttPayload(BleDeviceInfo &device) {
    StaticJsonDocument<1024> doc;
    
    // Add system info
    doc["system"]["uptime"] = getFormattedTime(millis());
    doc["system"]["ip"] = ipAddressToString(WiFi.localIP());
    doc["system"]["freeHeap"] = ESP.getFreeHeap();
    
    // Create devices array and add device data
    JsonArray devicesArray = doc.createNestedArray("devices");
    
    // Add data for each enabled device
    JsonObject deviceObj = devicesArray.createNestedObject();
    deviceObj["name"] = device.name;
    deviceObj["connected"] = device.connected;
    
    // Add basic info if available
    if (device.packetsReceived & 0x01) {
        JsonObject basicInfo = deviceObj.createNestedObject("basicInfo");
        basicInfo["v"] = (float)device.basicInfo.Volts / 100.0;  // voltage
        basicInfo["i"] = (float)device.basicInfo.Amps / 100.0;  // current
        basicInfo["p"] = (float)device.basicInfo.Watts;   // power
        basicInfo["soc"] = device.basicInfo.RSOC;
        basicInfo["cap"] = (float)device.basicInfo.CapacityRemainAh / 100.0;  // capacity
        basicInfo["ncells"] = device.basicInfo.NumberOfCells;
        basicInfo["ntemp"] = device.basicInfo.NumberOfTempSensors;
        
        // Add temperature readings (only if sensors exist)
        if (device.basicInfo.NumberOfTempSensors > 0) {
            JsonArray temps = basicInfo.createNestedArray("t");
            for (int j = 0; j < device.basicInfo.NumberOfTempSensors && j < 4; j++) {
                temps.add((float)device.basicInfo.TemperatureReadings[j] / 10.0);
            }
        }
        
        // Add protection status and ports status
        if (hasProtectionOrPortStatus(device)) {
            addProtectionAndPortStatus(basicInfo, device);
        }
    }
    
    // Serialize JSON to string
    String jsonString;
    serializeJson(doc, jsonString);
    
    return jsonString;
}

// Helper function to check if device has any protection or port status to report
bool hasProtectionOrPortStatus(BleDeviceInfo &device) {
    return (device.basicInfo.Protection.CellBlockOverVoltage ||
            device.basicInfo.Protection.CellBlockUnderVoltage ||
            device.basicInfo.Protection.BatteryOverVoltage ||
            device.basicInfo.Protection.BatteryUnderVoltage ||
            device.basicInfo.Protection.ChargingOverTemp ||
            device.basicInfo.Protection.ChargingUnderTemp ||
            device.basicInfo.Protection.DischargingOverTemp ||
            device.basicInfo.Protection.DischargingUnderTemp ||
            device.basicInfo.Protection.ChargingOverCurr ||
            device.basicInfo.Protection.DischargingOverCurr ||
            device.basicInfo.Protection.ShortCircuit ||
            device.basicInfo.Protection.ICError ||
            device.basicInfo.Protection.MOSLockIn ||
            device.basicInfo.ChargingPort ||
            device.basicInfo.DischargingPort);
}

// Helper function to add protection and port status to JSON
void addProtectionAndPortStatus(JsonObject &basicInfo, BleDeviceInfo &device) {
    JsonObject prot = basicInfo.createNestedObject("prot");
    prot["cov"] = device.basicInfo.Protection.CellBlockOverVoltage;
    prot["cuv"] = device.basicInfo.Protection.CellBlockUnderVoltage;
    prot["bov"] = device.basicInfo.Protection.BatteryOverVoltage;
    prot["buv"] = device.basicInfo.Protection.BatteryUnderVoltage;
    prot["cot"] = device.basicInfo.Protection.ChargingOverTemp;
    prot["cut"] = device.basicInfo.Protection.ChargingUnderTemp;
    prot["dot"] = device.basicInfo.Protection.DischargingOverTemp;
    prot["dut"] = device.basicInfo.Protection.DischargingUnderTemp;
    prot["coc"] = device.basicInfo.Protection.ChargingOverCurr;
    prot["doc"] = device.basicInfo.Protection.DischargingOverCurr;
    prot["sc"] = device.basicInfo.Protection.ShortCircuit;
    prot["ice"] = device.basicInfo.Protection.ICError;
    prot["mos"] = device.basicInfo.Protection.MOSLockIn;
    
    JsonObject ports = basicInfo.createNestedObject("ports");
    ports["chg"] = device.basicInfo.ChargingPort;
    ports["dis"] = device.basicInfo.DischargingPort;
}

// ================ BMS PACKET PROCESSING ================
// Helper function to check if a bit is set in a byte
bool checkBit(byte data, int pos) {
    return (data & (1 << pos)) != 0;
}

// Process a BMS packet
bool processBmsPacket(uint8_t *packet, BleDeviceInfo &device) {
    if (packet[0] != 0xDD) {
        Serial.println("Invalid packet: Wrong start byte");
        return false;
    }
    
    // Get data length from packet
    uint8_t dataLen = packet[3];
    Serial.print("Processing packet with data length: ");
    Serial.println(dataLen);
    
    // Process based on packet type
    switch (packet[1]) {
        case cBasicInfo: {
            // Process basic info packet according to SmartBMSUtility protocol
            device.basicInfo.Volts = ((uint16_t)packet[4] << 8) | packet[5];  // Total voltage in 0.01V units
            device.basicInfo.Amps = (int16_t)((packet[6] << 8) | packet[7]);  // Current in 0.01A units
            device.basicInfo.CapacityRemainAh = ((uint16_t)packet[8] << 8) | packet[9];  // Remaining capacity
            device.basicInfo.NominalCapacityAh = ((uint16_t)packet[10] << 8) | packet[11];  // Nominal capacity
            device.basicInfo.CycleLife = ((uint16_t)packet[12] << 8) | packet[13];
            device.basicInfo.ProductDate = ((uint16_t)packet[14] << 8) | packet[15];
            
            int cellIndex = 0;
            // Process balance cells (32 bits)
            for (int i = 15; i >= 0; i--) {
                device.basicInfo.BalanceCells[cellIndex] = checkBit(packet[16 + (i/8)], i % 8);
                cellIndex++;
            }
            for (int i = 31; i >= 16; i--) {
                device.basicInfo.BalanceCells[cellIndex] = checkBit(packet[16 + (i/8)], i % 8);
                cellIndex++;
            }
            
            // Process protection status
            device.basicInfo.Protection.CellBlockOverVoltage = checkBit(packet[21], 7);
            device.basicInfo.Protection.CellBlockUnderVoltage = checkBit(packet[21], 6);
            device.basicInfo.Protection.BatteryOverVoltage = checkBit(packet[21], 5);
            device.basicInfo.Protection.BatteryUnderVoltage = checkBit(packet[21], 4);
            device.basicInfo.Protection.ChargingOverTemp = checkBit(packet[21], 3);
            device.basicInfo.Protection.ChargingUnderTemp = checkBit(packet[21], 2);
            device.basicInfo.Protection.DischargingOverTemp = checkBit(packet[21], 1);
            device.basicInfo.Protection.DischargingUnderTemp = checkBit(packet[21], 0);
            
            device.basicInfo.Protection.ChargingOverCurr = checkBit(packet[20], 7);
            device.basicInfo.Protection.DischargingOverCurr = checkBit(packet[20], 6);
            device.basicInfo.Protection.ShortCircuit = checkBit(packet[20], 5);
            device.basicInfo.Protection.ICError = checkBit(packet[20], 4);
            device.basicInfo.Protection.MOSLockIn = checkBit(packet[20], 3);
            
            device.basicInfo.Version = packet[22];
            device.basicInfo.RSOC = packet[23];
            device.basicInfo.ControlStatus = packet[24];
            device.basicInfo.ChargingPort = checkBit(packet[24], 7);
            device.basicInfo.DischargingPort = checkBit(packet[24], 6);
            device.basicInfo.NumberOfCells = packet[25];
            device.basicInfo.NumberOfTempSensors = packet[26];
            
            // Process temperature readings
            if (device.basicInfo.NumberOfTempSensors > 0) {
                for (int i = 0; i < device.basicInfo.NumberOfTempSensors && i < 4; i++) {
                    device.basicInfo.TemperatureReadings[i] = ((int16_t)packet[27 + (i*2)] << 8) | packet[28 + (i*2)];
                }
            }
            
            // Calculate power
            device.basicInfo.Watts = ((int32_t)device.basicInfo.Volts * (int32_t)device.basicInfo.Amps) / 10000;  // Convert to watts
            
            device.packetsReceived |= 0x01;
            device.lastUpdate = millis();
            
            // Print detailed info to serial
            Serial.println("\n----- BLE DEVICE DATA -----");
            Serial.println("Device: " + device.name);
            Serial.println("Voltage: " + String((float)device.basicInfo.Volts/100, 2) + "V");
            Serial.println("Current: " + String((float)device.basicInfo.Amps/100, 2) + "A");
            Serial.println("Power: " + String((float)device.basicInfo.Watts, 2) + "W");
            Serial.println("Remaining Capacity: " + String((float)device.basicInfo.CapacityRemainAh/100, 2) + "Ah");
            Serial.println("Nominal Capacity: " + String((float)device.basicInfo.NominalCapacityAh/100, 2) + "Ah");
            Serial.println("Cycle Life: " + String(device.basicInfo.CycleLife));
            Serial.println("Product Date: " + String(device.basicInfo.ProductDate));
            Serial.println("Version: " + String(device.basicInfo.Version));
            Serial.println("RSOC: " + String(device.basicInfo.RSOC) + "%");
            Serial.println("Number of Cells: " + String(device.basicInfo.NumberOfCells));
            Serial.println("Number of Temp Sensors: " + String(device.basicInfo.NumberOfTempSensors));
            
            // Print protection status
            Serial.println("\nProtection Status:");
            Serial.println("Cell Block Over Voltage: " + String(device.basicInfo.Protection.CellBlockOverVoltage));
            Serial.println("Cell Block Under Voltage: " + String(device.basicInfo.Protection.CellBlockUnderVoltage));
            Serial.println("Battery Over Voltage: " + String(device.basicInfo.Protection.BatteryOverVoltage));
            Serial.println("Battery Under Voltage: " + String(device.basicInfo.Protection.BatteryUnderVoltage));
            Serial.println("Charging Over Temp: " + String(device.basicInfo.Protection.ChargingOverTemp));
            Serial.println("Charging Under Temp: " + String(device.basicInfo.Protection.ChargingUnderTemp));
            Serial.println("Discharging Over Temp: " + String(device.basicInfo.Protection.DischargingOverTemp));
            Serial.println("Discharging Under Temp: " + String(device.basicInfo.Protection.DischargingUnderTemp));
            Serial.println("Charging Over Current: " + String(device.basicInfo.Protection.ChargingOverCurr));
            Serial.println("Discharging Over Current: " + String(device.basicInfo.Protection.DischargingOverCurr));
            Serial.println("Short Circuit: " + String(device.basicInfo.Protection.ShortCircuit));
            Serial.println("IC Error: " + String(device.basicInfo.Protection.ICError));
            Serial.println("MOS Lock In: " + String(device.basicInfo.Protection.MOSLockIn));
            
            Serial.println("--------------------------\n");
            
            // Signal new data available to MQTT task
            if (xSemaphoreTake(dataMutex, (TickType_t)10) == pdTRUE) {
                newDataAvailable = true;
                xSemaphoreGive(dataMutex);
            }
            
            return true;
        }
            
        case cCellInfo:
            // Process cell info packet (unchanged)
            if (processCellInfo(&device.cellInfo, &packet[5], dataLen)) {
                device.packetsReceived |= 0x02;
                device.lastUpdate = millis();
                
                // Print cell info to serial
                Serial.println("\n----- CELL INFO -----");
                Serial.println("Number of Cells: " + String(device.cellInfo.NumOfCells));
                Serial.println("Min Cell: " + String((float)device.cellInfo.CellMin/1000, 3) + "V");
                Serial.println("Max Cell: " + String((float)device.cellInfo.CellMax/1000, 3) + "V");
                Serial.println("Avg Cell: " + String((float)device.cellInfo.CellAvg/1000, 3) + "V");
                Serial.println("Cell Diff: " + String(device.cellInfo.CellDiff) + "mV");
                Serial.println("---------------------\n");
                
                return true;
            }
            break;
            
        default:
            addStatusMessage("Unknown packet type: " + String(packet[1]), true);
            break;
    }
    
    return false;
}

// Process basic info data
bool processBasicInfo(packBasicInfoStruct *output, byte *data, unsigned int dataLen) {
    // Check expected data length
    if (dataLen < 0x22) {
        return false;
    }
    
    // Parse data fields
    output->Volts = ((uint32_t)twoIntsInto16(data[0], data[1])) * 10;       // 10mV -> mV
    output->Amps = ((int32_t)twoIntsInto16(data[2], data[3])) * 10;         // 10mA -> mA
    output->CapacityRemainAh = ((uint16_t)twoIntsInto16(data[4], data[5])) / 10;
    output->NominalCapacityAh = ((uint16_t)twoIntsInto16(data[6], data[7])) / 10;
    
    // Calculate SOC if nominal capacity is available, otherwise use provided value
    if (output->NominalCapacityAh > 0) {
        output->CapacityRemainPercent = (uint8_t)((float)output->CapacityRemainAh / output->NominalCapacityAh * 100);
    } else {
        output->CapacityRemainPercent = ((uint8_t)data[19]);
    }
    
    // Temperature data
    output->Temp1 = (((uint16_t)twoIntsInto16(data[23], data[24])) - 2731);
    output->Temp2 = (((uint16_t)twoIntsInto16(data[25], data[26])) - 2731);
    
    // Balance and MOSFET status
    output->BalanceCodeLow = (twoIntsInto16(data[12], data[13]));
    output->BalanceCodeHigh = (twoIntsInto16(data[14], data[15]));
    output->MosfetStatus = ((byte)data[20]);
    
    // Calculate time remaining
    float remainingTimeHours = 0.0;
    output->Watts = ((int32_t)output->Volts * (int32_t)output->Amps) / 1000000; // W
    
    if (output->Watts != 0) {
        // Convert capacity from Ah to Wh by multiplying with voltage
        float remainingWattHours = (float)output->CapacityRemainAh * ((float)output->Volts / 1000.0);
        
        if (output->Watts < 0) {
            // Discharging - calculate time until empty
            remainingTimeHours = remainingWattHours / (float)abs(output->Watts);
            
            // Store discharge time values
            output->RemainingTimeHoursToDischarge = (uint16_t)remainingTimeHours;
            output->RemainingTimeMinutesToDischarge = (uint16_t)((remainingTimeHours - output->RemainingTimeHoursToDischarge) * 60);
            output->RemainingTimeHoursToCharge = 0;
            output->RemainingTimeMinutesToCharge = 0;
        } else {
            // Charging - calculate time until full
            float fullCapacityWh = (float)output->NominalCapacityAh * ((float)output->Volts / 1000.0);
            float remainingToCharge = fullCapacityWh - remainingWattHours;
            remainingTimeHours = remainingToCharge / (float)output->Watts;
            
            // Store charging time values
            output->RemainingTimeHoursToCharge = (uint16_t)remainingTimeHours;
            output->RemainingTimeMinutesToCharge = (uint16_t)((remainingTimeHours - output->RemainingTimeHoursToCharge) * 60);
            output->RemainingTimeHoursToDischarge = 0;
            output->RemainingTimeMinutesToDischarge = 0;
        }
    } else {
        // No current flowing, set all times to zero
        output->RemainingTimeHoursToDischarge = 0;
        output->RemainingTimeMinutesToDischarge = 0;
        output->RemainingTimeHoursToCharge = 0;
        output->RemainingTimeMinutesToCharge = 0;
    }
    
    return true;
}

// Process cell info data
bool processCellInfo(packCellInfoStruct *output, byte *data, unsigned int dataLen) {
    // Each cell uses 2 bytes
    output->NumOfCells = dataLen / 2;
    if (output->NumOfCells > BMS_MAX_CELLS) {
        output->NumOfCells = BMS_MAX_CELLS;
    }

    uint32_t cellSum = 0;
    uint16_t cellMin = 65535;
    uint16_t cellMax = 0;

    for (byte i = 0; i < output->NumOfCells; i++) {
        uint16_t mv = ((uint16_t)data[i * 4] << 8) | data[i * 4 + 1];  // big-endian: high byte first
        output->CellVolt[i] = mv;
        cellSum += mv;

        if (mv > cellMax) cellMax = mv;
        if (mv < cellMin) cellMin = mv;
    }

    output->CellMin = cellMin;
    output->CellMax = cellMax;
    output->CellDiff = cellMax - cellMin;
    output->CellAvg = output->NumOfCells > 0 ? cellSum / output->NumOfCells : 0;

    return true;
}

// Validate a BMS packet
bool isPacketValid(byte *packet) {
    if (packet == nullptr) {
        return false;
    }
    
    // Check start bit
    if (packet[0] != 0xDD) {
        return false;
    }
    
    bmsPacketHeaderStruct *pHeader = (bmsPacketHeaderStruct *)packet;
    int checksumPos = pHeader->dataLen + 2; // status + data len + data
    int offset = 2; // header 0xDD and command type are not in data length
    
    // Check stop bit
    if (packet[offset + checksumPos + 2] != 0x77) {
        return false;
    }
    
    // Calculate and validate checksum
    byte checksum = 0;
    for (int i = 0; i < checksumPos; i++) {
        checksum += packet[offset + i];
    }
    checksum = ((checksum ^ 0xFF) + 1) & 0xFF;
    
    if (checksum != packet[offset + checksumPos + 1]) {
        return false;
    }
    
    return true;
}

// ================ UTILITY FUNCTIONS ================
// Add a status message to the circular buffer
void addStatusMessage(String message, bool isError) {
    // Get mutex to access status messages
    if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
        // Add message to the buffer
        statusMessages[messageIndex].message = message;
        statusMessages[messageIndex].timestamp = millis();
        statusMessages[messageIndex].isError = isError;
        
        // Move to next position in circular buffer
        messageIndex = (messageIndex + 1) % MAX_STATUS_MESSAGES;
        
        xSemaphoreGive(statusMutex);
    }
    
    // Also log to serial
    if (isError) {
        Serial.print("ERROR: ");
    }
    Serial.println(message);
}

// Get all status messages as a JSON array
void getAllStatusJson(JsonArray& status) {
    // Get status with mutex protection
    if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < MAX_STATUS_MESSAGES; i++) {
            int idx = (messageIndex - i - 1 + MAX_STATUS_MESSAGES) % MAX_STATUS_MESSAGES;
            if (statusMessages[idx].timestamp == 0) continue;
            
            JsonObject message = status.createNestedObject();
            message["time"] = getFormattedTime(statusMessages[idx].timestamp);
            message["message"] = statusMessages[idx].message;
            message["isError"] = statusMessages[idx].isError;
        }
        xSemaphoreGive(statusMutex);
    }
}

// Format milliseconds into a human-readable time string
String getFormattedTime(unsigned long milliseconds) {
    unsigned long seconds = milliseconds / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24;
    
    hours %= 24;
    minutes %= 60;
    seconds %= 60;
    
    char buffer[50];
    if (days > 0) {
        sprintf(buffer, "%lud %02lu:%02lu:%02lu", days, hours, minutes, seconds);
    } else {
        sprintf(buffer, "%02lu:%02lu:%02lu", hours, minutes, seconds);
    }
    
    return String(buffer);
}

// Read ESP32 battery voltage
void getEspBatteryVoltage() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    
    // Take multiple readings and average them
    const int numReadings = 10;
    int totalReading = 0;
    
    for (int i = 0; i < numReadings; i++) {
        totalReading += adc1_get_raw(ADC1_CHANNEL_7);
        delay(10);
    }
    
    float avgReading = (float)totalReading / numReadings;
    espBatteryVoltage = 2 * avgReading * (3.3 * 1.06 / 4095);
}

// Convert IP address to string
String ipAddressToString(IPAddress address) {
    return String(address[0]) + "." + String(address[1]) + "." + String(address[2]) + "." + String(address[3]);
}

// Convert two bytes into a 16-bit integer
int16_t twoIntsInto16(int highbyte, int lowbyte) {
    int16_t result = highbyte;
    result <<= 8;
    result |= lowbyte;
    return result;
}

// Print device info as JSON to Serial
void printDeviceJsonToSerial(BleDeviceInfo &device) {
    StaticJsonDocument<2048> doc;
    
    doc["name"] = device.name;
    doc["connected"] = device.connected;
    doc["timestamp"] = getFormattedTime(millis());
    
    // Basic info
    if (device.packetsReceived & 0x01) {
        JsonObject basicInfo = doc.createNestedObject("basicInfo");
        basicInfo["voltage"] = String((float)device.basicInfo.Volts / 1000, 2);
        basicInfo["current"] = String((float)device.basicInfo.Amps / 1000, 2);
        basicInfo["power"] = String((float)device.basicInfo.Watts, 2);
        basicInfo["soc"] = String((float)device.basicInfo.CapacityRemainPercent, 1);
        basicInfo["capacityRemain"] = String((float)device.basicInfo.CapacityRemainAh, 2);
        basicInfo["capacityNominal"] = String((float)device.basicInfo.NominalCapacityAh, 2);
        basicInfo["temp1"] = String((float)device.basicInfo.Temp1 / 10, 1);
        basicInfo["temp2"] = String((float)device.basicInfo.Temp2 / 10, 1);
    }
    
    // Cell info
    if (device.packetsReceived & 0x02) {
        JsonObject cellInfo = doc.createNestedObject("cellInfo");
        cellInfo["numCells"] = device.cellInfo.NumOfCells;
        cellInfo["minVoltage"] = String((float)device.cellInfo.CellMin / 1000, 3);
        cellInfo["maxVoltage"] = String((float)device.cellInfo.CellMax / 1000, 3);
        cellInfo["avgVoltage"] = String((float)device.cellInfo.CellAvg / 1000, 3);
        cellInfo["diff"] = device.cellInfo.CellDiff;
        
        JsonArray cells = cellInfo.createNestedArray("cells");
        for (int j = 0; j < device.cellInfo.NumOfCells; j++) {
            cells.add(String((float)device.cellInfo.CellVolt[j] / 1000, 3));
        }
    }
    
    Serial.println("\n----- BLE DEVICE DATA -----");
    serializeJsonPretty(doc, Serial);
    Serial.println("\n--------------------------");
}

// Initialize WiFi with more robust connection handling
void setupWiFi() {
    Serial.println("\nInitializing WiFi...");
    
    // Complete hardware reset of WiFi with better error handling
    WiFi.disconnect(true);  // Disconnect from any previous connections
    WiFi.mode(WIFI_OFF);    // Turn WiFi off completely
    delay(1000);
    
    // Start fresh with WiFi
    WiFi.mode(WIFI_STA);    // Set WiFi to station mode
    delay(100);
    
    WiFi.persistent(false);  // Disable persistent to prevent flash wear
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);    // Disable WiFi power saving
    
    // Set hostname before starting the connection
    WiFi.setHostname("ESP32-BMS");
    
    Serial.printf("Attempting to connect to %s\n", WIFI_SSID);
    
    // Start connection attempt
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // Wait for connection with simpler timeout handling
    int attempts = 0;
    const int maxAttempts = 30;  // Increased timeout
    
    while (attempts < maxAttempts) {
        wl_status_t status = WiFi.status();
        
        if (status == WL_CONNECTED) {
            Serial.println("\nWiFi Connected!");
            Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
            wifiConnected = true;
            return;
        }
        
        // Print detailed status for debugging
        switch (status) {
            case WL_IDLE_STATUS:
                Serial.println("WiFi IDLE");
                break;
            case WL_NO_SSID_AVAIL:
                Serial.println("SSID not available");
                break;
            case WL_SCAN_COMPLETED:
                Serial.println("Scan completed");
                break;
            case WL_CONNECT_FAILED:
                Serial.println("Connection failed");
                break;
            case WL_CONNECTION_LOST:
                Serial.println("Connection lost");
                break;
            case WL_DISCONNECTED:
                Serial.println("Disconnected");
                break;
            default:
                Serial.printf("Unknown status: %d\n", status);
                break;
        }
        
        attempts++;
        delay(1000);
        
        // Every 10 attempts, try restarting the connection
        if (attempts % 10 == 0) {
            Serial.println("Retrying connection...");
            WiFi.disconnect(true);
            delay(1000);
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    }
    
    Serial.println("\nWiFi connection failed after all attempts");
    addStatusMessage("WiFi connection failed. Will retry in the MQTT task.", true);
}

void bufferMessage(const String& message) {
    mqttBuffer[mqttBufferIndex].payload = message;
    mqttBuffer[mqttBufferIndex].timestamp = millis();
    mqttBuffer[mqttBufferIndex].valid = true;
    mqttBufferIndex = (mqttBufferIndex + 1) % MQTT_BUFFER_SIZE;
}

void sendBufferedMessages() {
    for (int i = 0; i < MQTT_BUFFER_SIZE; i++) {
        if (mqttBuffer[i].valid) {
            // Only send messages that are less than 1 hour old
            if (millis() - mqttBuffer[i].timestamp < 3600000) {
                if (mqttClient.publish(MQTT_TOPIC, mqttBuffer[i].payload)) {
                    mqttBuffer[i].valid = false; // Clear after successful send
                    lastSuccessfulMqttPublish = millis();
                    delay(100); // Small delay between messages
                } else {
                    // If sending fails, break and try again later
                    break;
                }
            } else {
                // Discard old messages
                mqttBuffer[i].valid = false;
            }
        }
    }
} 