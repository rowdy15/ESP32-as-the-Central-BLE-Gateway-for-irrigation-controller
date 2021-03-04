// we need to use the code in the connecToServer() of the client example to enable notifications on all characteristics.

// specifically we need to get handles of the services and characteristics using

//  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);

//pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);










//#include <esp_bt_device.h>     // ESP32 BLE
#include <Arduino.h>
#include "BLEDevice.h"
// #include "BLEScan.h"
#include <EEPROM.h>
#include <string.h>
#include <iostream>
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

#ifndef CONFIG_ARDUINO_LOOP_STACK_SIZE
#define CONFIG_ARDUINO_LOOP_STACK_SIZE 16384
#endif

#define ssid          config_ssid
#define password      config_password
#define MQTT_HOST     IPAddress(192, 168, 1, 19) //e.g. 192.168.1.19
#define MQTT_PORT     1883
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
static BLEUUID    tapServiceUUID("F3641400-00B0-4240-BA50-05CA45BF8ABC");
static BLEUUID    batteryServiceUUID("180F");
// The characteristic of the remote service we are interested in.
static BLEUUID    tapCharacteristicUUID("F3641401-00B0-4240-BA50-05CA45BF8ABC");
static BLEUUID    batteryCharacteristicUUID("2A19");

static BLEUUID    frequencyServiceUUID("F3641500-00B0-4240-BA50-05CA45BF8ABC");
static BLEUUID    frequencyCharacteristicUUID("F3641501-00B0-4240-BA50-05CA45BF8ABC");

//static BLEUUID    fullBatteryCharUUID("00002A19-0000-1000-8000-00805F9B34FB");

static uint8_t feedWDT = 0;
// static boolean battery_notifications = false;
static boolean doConnect = false;
static boolean doScan = false;
static boolean ble_connected = false;
static boolean mqtt_connected = false;
// static boolean frequency_notify_enabled = false;

static BLEScan* pBLEScan;
static BLEAdvertisedDevice* myDevice;
static BLEClient*  pClient;
static BLERemoteService* pRemoteTapService;
static BLERemoteService* pRemoteBatteryService;
static BLERemoteService* pRemoteFrequencyService;
static BLERemoteCharacteristic* pRemoteTapCharacteristic;
static BLERemoteCharacteristic* pRemoteBatteryCharacteristic;
static BLERemoteCharacteristic* pRemoteFrequencyCharacteristic;

//static BLEAddress myDeviceAddress("d0:35:d2:81:92:a8");

std::string myDeviceName = "Front_Garden";
static int rssi = 0;

void connectToMqtt();
// bool obtainServiceHandle(BLEUUID serviceUUID);
bool obtainCharacteristicHandle(BLEUUID tapCharacteristicUUID);
static void notifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
void enableFrequencyNotifications(void);
void enableTapNotifications(void);
void enableBatteryNotifications(void);

void setupTapNotifications(void){
   // Obtain a reference to the service we are after in the remote BLE server.
    pRemoteTapService = pClient->getService(tapServiceUUID);
    if (pRemoteTapService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(tapServiceUUID.toString().c_str());
      pClient->disconnect();
    }
    Serial.println(" - Found our tap service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteTapCharacteristic = pRemoteTapService->getCharacteristic(tapCharacteristicUUID);
    if (pRemoteTapCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(tapCharacteristicUUID.toString().c_str());
      pClient->disconnect();
    }
    Serial.println(" - Found our tap characteristic");

    // Read the value of the characteristic.
    // if(pRemoteTapCharacteristic->canRead()) {
    //   std::string value = pRemoteTapCharacteristic->readValue();
    //   Serial.print("The Tap characteristic value was: ");
    //   Serial.println(value.c_str());
    // }

    enableTapNotifications();
}

void setupBatteryNotifications(void){
   // Obtain a reference to the service we are after in the remote BLE server.
    pRemoteBatteryService = pClient->getService(batteryServiceUUID);
    if (pRemoteBatteryService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(batteryServiceUUID.toString().c_str());
      pClient->disconnect();
    }
    Serial.println(" - Found our battery service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteBatteryCharacteristic = pRemoteBatteryService->getCharacteristic(batteryCharacteristicUUID);
    if (pRemoteBatteryCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(batteryCharacteristicUUID.toString().c_str());
    }
    Serial.println(" - Found our battery characteristic");

    //Read the value of the characteristic.
    // if(pRemoteBatteryCharacteristic->canRead()) {
    //   std::string value = pRemoteBatteryCharacteristic->readValue();
    //   Serial.print("The battery characteristic value was: ");
    //   Serial.println(value.c_str());
    // }

    enableBatteryNotifications();
}

void setupFrequencyNotifications(void){
   //Obtain a reference to the service we are after in the remote BLE server.
    pRemoteFrequencyService = pClient->getService(frequencyServiceUUID);
    if (pRemoteFrequencyService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(frequencyServiceUUID.toString().c_str());
      pClient->disconnect();
    }
    Serial.println(" - Found our frequency service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteFrequencyCharacteristic = pRemoteFrequencyService->getCharacteristic(frequencyCharacteristicUUID);
    if (pRemoteFrequencyCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(frequencyCharacteristicUUID.toString().c_str());
      pClient->disconnect();
    }
    Serial.println(" - Found our frequency characteristic");

    // //Read the value of the characteristic.
    // if(pRemoteFrequencyCharacteristic->canRead()) {
    //   std::string value = pRemoteFrequencyCharacteristic->readValue();
    //   Serial.print("The frequency characteristic value was: ");
    //   Serial.println(value.c_str());
    // }

    // enableFrequencyNotifications();  
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
  // if(myDevice){ 
    Serial.print("Forming a connection to our device: ");
    Serial.print(myDevice->getName().c_str());
    Serial.print(" (");
    Serial.print(myDevice->getAddress().toString().c_str());
    Serial.println(")");
    bool connection_result = pClient->connect(myDevice);   
    ble_connected = connection_result;     

    setupTapNotifications();
    setupBatteryNotifications(); 
    setupFrequencyNotifications();
    
    return connection_result;
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
    } // react to it being my device
  }// onResult
}; // MyAdvertisedDeviceCallbacks


// void resetWDT(void){
//   TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
//   TIMERG0.wdt_feed=1;
//   TIMERG0.wdt_wprotect=0;
// }

void scanForDevice(void) {
  Serial.println("scanning for device");
  if(!pBLEScan){
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  }
  pBLEScan->setInterval(1000);
  pBLEScan->setWindow(1000);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(1800, false);
}

void setTapState(uint8_t state) {
    if (ble_connected){                                 // If we are connected to a peer BLE Server, update the characteristic
      if(pRemoteTapCharacteristic->canWrite()){
        pRemoteTapCharacteristic->writeValue(state, true);
      }
      Serial.println("Tap is set to \"" + String(state) + "\"");
    }
}

int getTapState(void) {
      int timeLeft = static_cast<uint32_t>(pRemoteTapCharacteristic->readUInt8());
      Serial.print("The read time left is ");
      Serial.println(timeLeft);
      mqttClient.publish(frontGardenTapMinutesLeft,0,false,String(timeLeft).c_str());
      return timeLeft;
}

void enableTapNotifications(void) {
    if(pRemoteTapCharacteristic->canNotify()) {
    pRemoteTapCharacteristic->registerForNotify(notifyCallback);
    Serial.println("The remote characteristic has notify and it has been enabled");
  }
}

void disableTapNotifications(void) {
    if(pRemoteTapCharacteristic->canNotify()) {
    pRemoteTapCharacteristic->registerForNotify(NULL);
    Serial.println("The remote characteristic has notify and it has been disabled");
  }
}

void enableBatteryNotifications(void) {
    if(pRemoteBatteryCharacteristic->canNotify()) {
    pRemoteBatteryCharacteristic->registerForNotify(notifyCallback);
    Serial.println("The remote characteristic has notify and it has been enabled");
  }
}

void disableBatteryNotifications(void) {
    if(pRemoteBatteryCharacteristic->canNotify()) {
    pRemoteBatteryCharacteristic->registerForNotify(NULL);
    Serial.println("The remote characteristic has notify and it has been disabled");
  }
}

void enableFrequencyNotifications(void) {
    if(pRemoteFrequencyCharacteristic->canNotify()) {
    pRemoteFrequencyCharacteristic->registerForNotify(notifyCallback);
    Serial.println("The remote characteristic has notify and it has been enabled");
  }
}

void disableFrequencyNotifications(void) {
    if(pRemoteFrequencyCharacteristic->canNotify()) {
    pRemoteFrequencyCharacteristic->registerForNotify(NULL);
    Serial.println("The remote characteristic has notify and it has been disabled");
  }
}

void changeBatteryReadFrequency(uint8_t frequency) {
    if(pClient->isConnected() && pRemoteFrequencyService != nullptr){
      pRemoteFrequencyCharacteristic->writeValue(frequency,true);
    }
}

static void notifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.print("Notify callback for characteristic has arrived ");
    // Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    //Serial.println((char*)pData);
    Serial.println(static_cast<uint32_t>(*pData));

  if(pBLERemoteCharacteristic->getUUID().equals(batteryCharacteristicUUID)){
      int batteryPrcnt = static_cast<uint32_t>(*pData);
      Serial.println("The battery level is " + String(batteryPrcnt) + "%");
      mqttClient.publish(frontGardenTapBtryLvl,0,false,String(batteryPrcnt).c_str());
  }

 if(pBLERemoteCharacteristic->getUUID().equals(tapCharacteristicUUID)){
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

  if(pBLERemoteCharacteristic->getUUID().equals(frequencyCharacteristicUUID)){
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

  // ArduinoOTA.setHostname("ESP32_BLE_GATEWAY");

  // ArduinoOTA
  //   .onStart([]() {
  //     String type;
  //     if (ArduinoOTA.getCommand() == U_FLASH)
  //       type = "sketch";
  //     else // U_SPIFFS
  //       type = "filesystem";

  //     // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  //     Serial.println("Start updating " + type);
  //   })
  //   .onEnd([]() {
  //     Serial.println("\nEnd");
  //   })
  //   .onProgress([](unsigned int progress, unsigned int total) {
  //     Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  //   })
  //   .onError([](ota_error_t error) {
  //     Serial.printf("Error[%u]: ", error);
  //     if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  //     else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  //     else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  //     else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  //     else if (error == OTA_END_ERROR) Serial.println("End Failed");
  //   });

  // ArduinoOTA.begin();

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
  mqttClient.publish(frontGardenTapConnection,0,true,"connected");
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
 Serial.print("  packetId: ");
 Serial.println(packetId);
 Serial.print("  qos: ");
 Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
 Serial.print("  packetId: ");
 Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    // handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");

  // String prefix = "change_battery_frequency_to_";
  // int prefixlngth = prefix.length();

  topic[size_t(topic)] = '\0';
  String t = String(topic);
  payload[len] = '\0';
  String s = String((char*)payload);
  Serial.print("the message is: " );
  Serial.println(s.substring(0,24));
    if (s == "off"){ 
      Serial.println("The tap will be turned off");
      setTapState(0x00);     
    } else if (t == frontGardenTapConnectionSet && s == "connect" ) { //register notifications on battery service
      doConnect = true;
    } else if (t == frontGardenTapConnectionSet && s == "disconnect" ) { //register notifications on battery service
      doConnect = false;
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
      //delay(5);
      //enableTapNotifications();
      //delay(5);
     
    } else if (s == "enable_battery_updates" ) { //register notifications on battery service
      enableBatteryNotifications(); 
    } else if (s == "disable_battery_updates" ) {  // unregister notifications on battery service
      disableBatteryNotifications();  
    } else if (s == "enable_frequency_notify" ) { //register notifications on battery service
      enableFrequencyNotifications(); 
      //delay(5); 
    } else if (s == "disable_frequency_notify" ) { //register notifications on battery service
      disableFrequencyNotifications(); 
    } else if (String(frontGardenTapBtryFreqSet) == t) {
      changeBatteryReadFrequency((uint8_t) s.toInt());               
    } else if (s == "get_battery_level"){
      Serial.println("I am going to ask for the battery value.");
        int value = pRemoteBatteryCharacteristic->readUInt8();
        Serial.print("the battery value is: ");
        Serial.print((int)value);
        Serial.println("%");
        mqttClient.publish(frontGardenTapBtryLvl,0,false,String(value).c_str());
    }  else if (s == "reset_esp"){
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





//ArduinoOTA.handle();
  unsigned long currentMillis = millis();
//
 if (currentMillis - previousMillis >= 2000) {
   // save the last time you blinked the LED
   previousMillis = currentMillis;

     // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (ble_connected) {
    // String newValue = "Time since boot: " + String(millis()/1000);
    // Serial.println("Setting new characteristic value to \"" + newValue + "\"");
    
    // // Set the characteristic's value to be the array of bytes that is actually a string.
    // pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
 }

// if(feedWDT){    // This is enabled during connection to ble server as it take a long time to connect.
//   resetWDT();
// }

} // End of loop
