//#include <esp_bt_device.h>     // ESP32 BLE
#include "BLEDevice.h"
#include "BLEScan.h"
#include <EEPROM.h>
//#include <string.h>
#include <Wire.h>
#include <WiFi.h>
//extern "C" {
//  #include "freertos/FreeRTOS.h"
//  #include "freertos/timers.h"
//}
//
#include "soc/timer_group_struct.h"      //   required for the resetWDT function
#include "soc/timer_group_reg.h"         //   required for the resetWDT function

#include <AsyncMqttClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "configInfo.h"

#define ssid     wifi_SSID
#define password wifi_Psswd

#define MQTT_HOST   IPAddress(192, 168, 1, 19) //e.g. 192.168.1.19
#define MQTT_PORT 1883
#define mqtt_user     mqtt_username
#define mqtt_password mqtt_psswd


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;


#define frontGardenTapCmd          "irrigation/frontGardenTap/IncomingCmd"
#define frontGardenTapState        "irrigation/frontGardenTap/State"
#define frontGardenTapMinutesLeft  "irrigation/frontGardenTap/MinutesLeft"
#define frontGardenTapConnection   "irrigation/frontGardenTap/Connection"
#define frontGardenTapConnectionSet   "irrigation/frontGardenTap/Connection/Set"
#define frontGardenTapBtryLvl      "irrigation/frontGardenTap/BatteryLvl"
#define frontGardenTapBtryFreq     "irrigation/frontGardenTap/BatteryFrequency"
#define frontGardenTapBtryFreqSet  "irrigation/frontGardenTap/BatteryFrequency/Set"
#define frontGardenTapRSSI         "irrigation/frontGardenTap/Rssi"
#define frontGardenTapLog         "irrigation/frontGardenTap/Log"

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long           interval       = 30000;

WiFiClient espClient;

IPAddress ip(192, 168, 1, 100);
IPAddress dns(192, 168, 1, 1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

// The remote service we wish to connect to.
static BLEUUID    serviceUUID("F3641400-00B0-4240-BA50-05CA45BF8ABC");
static BLEUUID    serviceUUID_battery("180F");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID("F3641401-00B0-4240-BA50-05CA45BF8ABC");
static BLEUUID    charUUID_battery("2A19");

static BLEUUID    serviceUUID_btry_freq("F3641500-00B0-4240-BA50-05CA45BF8ABC");
static BLEUUID    charUUID_btry_freq("F3641501-00B0-4240-BA50-05CA45BF8ABC");

//static BLEUUID    fullBatteryCharUUID("00002A19-0000-1000-8000-00805F9B34FB");

static uint8_t feedWDT = 0;
static boolean battery_notifications = false;
static boolean doConnect = false;
static boolean ble_connected = false;
static boolean mqtt_connected = false;
static boolean frequency_notify_enabled = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
//static BLERemoteCharacteristic* pRemoteBatteryCharacteristic;
static BLEAdvertisedDevice* myDevice;
static BLEClient*  pClient;
static BLERemoteService* pRemoteService;
static BLEScan* pBLEScan;
//static BLEAddress myDeviceAddress("d0:35:d2:81:92:a8");

std::string myDeviceName = "Front_Garden";
static int rssi = 0;

void connectToMqtt();


int getTapState(void) {
      obtainServiceHandle(serviceUUID);
      obtainCharacteristicHandle(charUUID);     
      int timeLeft = static_cast<uint32_t>(pRemoteCharacteristic->readUInt8());
      Serial.print("The read time left is ");
      Serial.println(timeLeft);
      mqttClient.publish(frontGardenTapMinutesLeft,0,false,String(timeLeft).c_str());

      return timeLeft;

}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {    
    Serial.println("We are Connected!");
    ble_connected = true;      
    mqttClient.publish(frontGardenTapConnection,0,false,"connected");
    rssi = myDevice->getRSSI();    
    mqttClient.publish(frontGardenTapRSSI,0,false,String(rssi).c_str());
  }
  void onDisconnect(BLEClient* pclient) {
    Serial.println("We have been disconnected!");   
    ble_connected = false;   
    mqttClient.publish(frontGardenTapConnection,0,true,"not connected");
    feedWDT = 0;
  }
};

bool connectToServer() {  
  if(myDevice){ 
    Serial.print("Forming a connection to our device: ");
    Serial.print(myDevice->getName().c_str());
    Serial.print(" (");
    Serial.print(myDevice->getAddress().toString().c_str());
    Serial.println(")");
    pClient->connect(myDevice);      
    return true;
  } else {
    return false;
  }
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks { 
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
    if (advertisedDevice.getName() == myDeviceName) {
      pBLEScan->stop();     

      if(myDevice == nullptr){
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        Serial.println("myDevice created");
      }
      if(pClient == nullptr){
        pClient  = BLEDevice::createClient();    
        pClient->setClientCallbacks(new MyClientCallback());
        Serial.println("my pClient created");
      }
      //connectToServer();
      //doConnect = true;
      return;
    } // onResult
  }
}; // MyAdvertisedDeviceCallbacks


void resetWDT(void){
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed=1;
  TIMERG0.wdt_wprotect=0;
}

void scanForDevice(void) {
  Serial.println("scanning for device");
  if(!pBLEScan){
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  }
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(100);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(1800, false);
}

bool obtainServiceHandle(BLEUUID serviceUUID) {
    // Obtain a reference to the service we are after in the remote BLE server.
    pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");
    return true;
}

bool obtainCharacteristicHandle(BLEUUID charUUID) {
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");
    return true;
}

std::string readCharacteristicValue(BLEUUID charUUID){
    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string val = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(val.c_str());
      return val;
      
    } else {
      Serial.println("The characteristic value could not be read!");
    }
}

void setTapState(uint8_t state) {
    if (ble_connected){                                 // If we are connected to a peer BLE Server, update the characteristic
      obtainServiceHandle(serviceUUID);
      obtainCharacteristicHandle(charUUID);      
      pRemoteCharacteristic->writeValue(state, true);
      //Serial.println("Tap is set to \"" + String(state) + "\"");
    }
}

void registerForNotifications(void) {
    if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println("The remote characteristic has notify and it has been enabled");
  }
}

void deregisterNotifications(void) {
    if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(NULL);
    Serial.println("The remote characteristic has notify and it has been disabled");
  }
}

void enableTapNotifications(void){
  if (ble_connected) {
    obtainServiceHandle(serviceUUID);
    obtainCharacteristicHandle(charUUID);
    registerForNotifications();
  }
}

void disableTapNotifications(void){
  if (ble_connected) {
    obtainServiceHandle(serviceUUID);
    obtainCharacteristicHandle(charUUID);
    deregisterNotifications();
  }
}

void enableBatteryNotifications(void){
  if (ble_connected) {
    obtainServiceHandle(serviceUUID_battery);
    obtainCharacteristicHandle(charUUID_battery);
    registerForNotifications();
  }
}

void disableBatteryNotifications(void){
  if (ble_connected) {
    obtainServiceHandle(serviceUUID_battery);
    obtainCharacteristicHandle(charUUID_battery);
    deregisterNotifications();
  }
}

void enableBatteryCheckFrequencyNotifications(void){
  if (ble_connected) {
    Serial.println("about to register for notifications");
    //delay(10);
    obtainServiceHandle(serviceUUID_btry_freq);
    obtainCharacteristicHandle(charUUID_btry_freq);
    registerForNotifications();
  }
}

void disableBatteryCheckFrequencyNotifications(void){
  if (ble_connected) {
    obtainServiceHandle(serviceUUID_btry_freq);
    obtainCharacteristicHandle(charUUID_btry_freq);
    deregisterNotifications();
  }
}

void changeBatteryReadFrequency(uint8_t frequency) {


    obtainServiceHandle(serviceUUID_btry_freq);
    obtainCharacteristicHandle(charUUID_btry_freq);
    
//    if(!frequency_notify_enabled){
//      enableBatteryCheckFrequencyNotifications();
//      frequency_notify_enabled = true;
//    }
    if(pClient->isConnected() && pRemoteService != nullptr){
      pRemoteCharacteristic->writeValue(frequency,true);
    }
}

static void notifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    //Serial.println((char*)pData);
    Serial.println(static_cast<uint32_t>(*pData));
      


  if(pBLERemoteCharacteristic->getUUID().equals(charUUID_battery)){
      int batteryPrcnt = static_cast<uint32_t>(*pData);
      Serial.println("The battery level is " + String(batteryPrcnt) + "%");
      mqttClient.publish(frontGardenTapBtryLvl,0,false,String(batteryPrcnt).c_str());
  }

 if(pBLERemoteCharacteristic->getUUID().equals(charUUID)){
    int tap = static_cast<uint32_t>(*pData);
    if(tap == 0){
      Serial.println("The tap will be turned off.");
      mqttClient.publish(frontGardenTapMinutesLeft,0,false,"0");
//      mqttClient.publish(frontGardenTap,0,false,"off"); 
    } else if (tap > 0x00){
      mqttClient.publish(frontGardenTapMinutesLeft,0,true, String(tap).c_str());
//      mqttClient.publish(frontGardenTapState,0,false,"on");

      Serial.print(tap);
      Serial.println(" minutes left before the tap is turned off.");
    }      
  }

  if(pBLERemoteCharacteristic->getUUID().equals(charUUID_btry_freq)){
    int frqncy = static_cast<uint32_t>(*pData);

    switch (frqncy){
      case 0:
            Serial.println("frequency changed to off");
            mqttClient.publish(frontGardenTapBtryFreq,0,true,"off");
        break;
      case 1:
            Serial.println("frequency changed to 1 minute");
            mqttClient.publish(frontGardenTapBtryFreq,0,true,String(frqncy).c_str());
        break;
      case 2:
          Serial.println("frequency changed to 10 minutes");
          mqttClient.publish(frontGardenTapBtryFreq,0,true,String(frqncy).c_str());
          break;
      case 3:
          Serial.println("frequency changed to 30 minutes");
          mqttClient.publish(frontGardenTapBtryFreq,0,true,String(frqncy).c_str());
          break;
      case 4:
          Serial.println("frequency changed to 1 hour");
          mqttClient.publish(frontGardenTapBtryFreq,0,true,String(frqncy).c_str());
          break;
      case 5:
          Serial.println("frequency changed to 6 hours");
          mqttClient.publish(frontGardenTapBtryFreq,0,true,String(frqncy).c_str());
          break;
      case 6:
          Serial.println("frequency changed to 12 hours");
          mqttClient.publish(frontGardenTapBtryFreq,0,true,String(frqncy).c_str());
          break;
      case 7:
          Serial.println("frequency changed to 24 hours");
          mqttClient.publish(frontGardenTapBtryFreq,0,true,String(frqncy).c_str());
          break;
      default:
        break;
     }      
  }
}

void setupWifi(void) {
  //connect to wifi
  WiFi.setHostname("Tap_Gateway_Esp");
  WiFi.config(ip, dns, gateway, subnet); 
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
  // ArduinoOTA.setPort(3232);

  ArduinoOTA.setHostname("ESP32_BLE_GATEWAY");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  delay(500);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  mqtt_connected  = true;
  uint16_t packetIdSub = mqttClient.subscribe(frontGardenTapCmd,0);
  delay(10);
  uint16_t packetIdSub2 = mqttClient.subscribe(frontGardenTapBtryFreqSet,0);
  delay(10);
  uint16_t packetIdSub3 = mqttClient.subscribe(frontGardenTapConnectionSet,0);
  delay(10);
  mqttClient.publish(frontGardenTapConnection,0,true,"not connected");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  mqtt_connected = false;

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
//  Serial.print("  packetId: ");
//  Serial.println(packetId);
//  Serial.print("  qos: ");
//  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
//  Serial.print("  packetId: ");
//  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    // handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");

  String prefix = "change_battery_frequency_to_";
  int prefixlngth = prefix.length();

  topic[size_t(topic)] = '\0';
  String t = String(topic);
  payload[len] = '\0';
  String s = String((char*)payload);
  Serial.print("the message is: " );
  Serial.println(s.substring(0,24));
    if (s == "off"){ 
      Serial.println("The tap will be turned off");
      delay(5);
      disableTapNotifications();
      delay(5);
      setTapState(0x00); 
      delay(5);       
    } else if (t == frontGardenTapConnectionSet && s == "connect" ) { //register notifications on battery service
      if(!ble_connected){
        feedWDT = 1;
        if(pClient != nullptr && pClient->connect(myDevice)){
          Serial.println("disabling feedWDT");
          feedWDT = 0;   
          int timeLeft = getTapState();
          if (timeLeft != 0) {
            enableTapNotifications();
          }
        }
      }
    } else if (t == frontGardenTapConnectionSet && s == "disconnect" ) { //register notifications on battery service
      //doConnect = false;
      if(ble_connected){
        pClient->disconnect();
      } else {
        mqttClient.publish(frontGardenTapConnection,0,true,"not connected");
      }
    } else if(s.toInt() > 0 && s.toInt() <= 240 && String(frontGardenTapCmd) == t)  {
      Serial.println("The tap will be turned on"); 
      delay(5);

      uint8_t msgAsChar = (char) s.toInt();
      setTapState(msgAsChar);
      delay(5);
      enableTapNotifications();
      delay(5);
     
    } else if (s == "enable_battery_updates" ) { //register notifications on battery service
      enableBatteryNotifications(); 
    } else if (s == "disable_battery_updates" ) {  // unregister notifications on battery service
      disableBatteryNotifications();  
    } else if (s == "enable_frequency_notify" ) { //register notifications on battery service
      enableBatteryCheckFrequencyNotifications(); 
      //delay(5); 
    } else if (s == "disable_frequency_notify" ) { //register notifications on battery service
      disableBatteryCheckFrequencyNotifications(); 
    } else if (String(frontGardenTapBtryFreqSet) == t) {
      if(s.toInt() ==  0){
        disableBatteryNotifications();
      } else {
        disableBatteryNotifications();
        changeBatteryReadFrequency((uint8_t) s.toInt());
        enableBatteryNotifications();        
      }           
    } else if (s == "reset_esp"){
      ESP.restart();
    } else if (s == "get_time_left"){
      getTapState();
    }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

//unsigned long previousMillis = 0; 

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(setupWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(mqtt_user,mqtt_password);

  setupWifi();
 
  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P9);
 
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());

  scanForDevice();
} // End of setup.

// This is the Arduino main loop function.
void loop() {
ArduinoOTA.handle();
//  unsigned long currentMillis = millis();
//
//  if (currentMillis - previousMillis >= 2000) {
//    // save the last time you blinked the LED
//    previousMillis = currentMillis;
//
//    Serial.println("still up");
//  }

if(feedWDT){    // This is enabled during connection to ble server as it take a long time to connect.
  resetWDT();
}

} // End of loop
