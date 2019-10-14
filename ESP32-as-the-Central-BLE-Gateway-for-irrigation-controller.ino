#include <esp_bt_device.h>     // ESP32 BLE
#include "BLEDevice.h"
#include "BLEScan.h"
#include <PubSubClient.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "configInfo.h"

#define ssid     wifi_SSID
#define password wifi_Psswd

#define mqtt_server   mqtt_server_ip //e.g. 192.168.1.19
#define mqtt_user     mqtt_username
#define mqtt_password mqtt_psswd

#define frontGardenTapCmd          "irrigation/frontGardenTapCmd"
#define frontGardenTapState        "irrigation/frontGardenTapState"
#define frontGardenTapMinutesLeft  "irrigation/frontGardenTapMinutesLeft"
#define frontGardenTapConnection   "irrigation/frontGardenConnection"
#define frontGardenTapBtryLvl      "irrigation/frontGardenBattery"
#define frontGardenTapBtryFreq     "irrigation/frontGardenBatteryFrequency"
#define frontGardenTapRSSI         "irrigation/frontGardenRSSI"

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long           interval       = 30000;

WiFiClient espClient;
PubSubClient client(espClient);

//SSD1306Wire  *display;
IPAddress ip(192, 168, 1, 99);
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

static boolean doConnect = false;
static boolean connected = false;
//static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
static BLEClient*  pClient;
static BLERemoteService* pRemoteService;
static BLEScan* pBLEScan;
//static BLEAddress myDeviceAddress("d0:35:d2:81:92:a8");

std::string myDeviceName = "Front_Garden";
static int rssi = 0;

void reconnectAndSend(String topic,String val);
bool connectToServer(void);

void publishConnection(bool isConnected) {
  String conn = "not connected";
  if(isConnected){
    conn = "connected";
  }
  
  if (!client.connected()){
    reconnectAndSend(frontGardenTapConnection, conn);
  } else{
    client.publish(frontGardenTapConnection,conn.c_str(),true);
  }
}

static void notifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
//    Serial.print("data: ");
//    Serial.println((char*)pData);

  client.publish(frontGardenTapConnection,"connected",true);

  if(pBLERemoteCharacteristic->getUUID().equals(charUUID_btry_freq)){
      int frqncy = static_cast<uint32_t>(*pData);
      //Serial.println("Response to frequency raw " + String((char)*pData));
      Serial.println("Response to frequency change " + String(frqncy));
      if (!client.connected()){
        reconnectAndSend(frontGardenTapBtryFreq,String(frqncy).c_str());
      } else{
        client.publish(frontGardenTapBtryFreq,String(frqncy).c_str(),true);;
      }
  }

  if(pBLERemoteCharacteristic->getUUID().equals(charUUID_battery)){
      int batteryPrcnt = static_cast<uint32_t>(*pData);
      Serial.println("A battery level is " + String(batteryPrcnt));
      client.publish(frontGardenTapBtryLvl,String(batteryPrcnt).c_str(),true);
  }

 if(pBLERemoteCharacteristic->getUUID().equals(charUUID)){
  int tap = static_cast<uint32_t>(*pData);
    if(tap == 0){      
      // let mqtt know the tap is off
      client.publish(frontGardenTapMinutesLeft,"0");
      client.publish(frontGardenTapState,"off");
      //displayMsg("tap off");       
    } else if (tap > 0x00){
      Serial.print("There is ");
      Serial.print(tap);
      Serial.println(" minutes left before the tap is turned off.");
      client.publish(frontGardenTapMinutesLeft, String(tap).c_str());
      client.publish(frontGardenTapState,"on");
      //displayMsg("tap on");
    }
      
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {    
    Serial.println("We are Connected!");
    connected = true;    
  }
    void onDisconnect(BLEClient* pclient) {
    Serial.println("We have been disconnected!"); 
   // esp_ble_remove_bond_device(myDevice->getAddress());   
    connected = false;    
  }
};

bool connectToServer() {
    doConnect = false;
    Serial.print("Forming a connection to ");
    if(myDevice){
      Serial.println(myDevice->getAddress().toString().c_str());
      
      if(!pClient){
        pClient  = BLEDevice::createClient();    
        pClient->setClientCallbacks(new MyClientCallback());
        Serial.println(" - Created client");
      }
      // Connect to the remove BLE Server.
      pClient->connect(myDevice);
      if(pClient->isConnected()){
        Serial.println(" - Connected to server");
        connected = true;
  
        rssi = myDevice->getRSSI();
  
        if(rssi != 0){
          if (!client.connected()){
            reconnectAndSend(frontGardenTapRSSI,String(rssi).c_str());
          } else{
            client.publish(frontGardenTapRSSI,String(rssi).c_str(),true);;
          }
          rssi = 0;
        }
  
        publishConnection(connected);
        return true;
      } 
    }
    return false;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
 //   if (advertisedDevice.getAddress().equals(myDeviceAddress)) {
    if (advertisedDevice.getName() == myDeviceName) {
      Serial.println("We found our device!!!!");
      pBLEScan->stop();
      
      if(!myDevice){
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        Serial.println("Setting new device as myDevice");
        //connectToServer();
      }
      doConnect = true; 
    } else {// Found our server
        doConnect = false; 
    }// Our server was not found
  } // onResult
}; // MyAdvertisedDeviceCallbacks







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
  if (doConnect == true){
    if (connected == true){
      obtainServiceHandle(serviceUUID);
      obtainCharacteristicHandle(charUUID);
      // If we are connected to a peer BLE Server, update the characteristic
      
      if(pRemoteCharacteristic->canWrite()) {
        pRemoteCharacteristic->writeValue(state, true);
        Serial.println("Tap is set to \"" + String(state) + "\"");
      } else {
        Serial.println("Tap characteristic could not be written to");
      }
    } else if (connected == false) {
      if (connectToServer()) {
        obtainServiceHandle(serviceUUID);
        obtainCharacteristicHandle(charUUID);
        // If we are connected to a peer BLE Server, update the characteristic
        
        if(pRemoteCharacteristic->canWrite()) {
          pRemoteCharacteristic->writeValue(state, true);
          Serial.println("Tap is set to \"" + String(state) + "\"");
          
          
        } else {
          Serial.println("Tap characteristic could not be written to");
        }  
      }       
    }
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
  if (connected == true) {
    obtainServiceHandle(serviceUUID);
    obtainCharacteristicHandle(charUUID);
    registerForNotifications();
  }
}

void disableTapNotifications(void){
  if (connected == true) {
    obtainServiceHandle(serviceUUID);
    obtainCharacteristicHandle(charUUID);
    deregisterNotifications();
  }
}

void enableBatteryNotifications(void){
  if (connected == true) {
    obtainServiceHandle(serviceUUID_battery);
    obtainCharacteristicHandle(charUUID_battery);
    registerForNotifications();
  }
}

void disableBatteryNotifications(void){
  if (connected == true) {
    obtainServiceHandle(serviceUUID_battery);
    obtainCharacteristicHandle(charUUID_battery);
    deregisterNotifications();
  }
}

void getTapState(void) {
  if (connected == true){
    obtainServiceHandle(serviceUUID);
    obtainCharacteristicHandle(charUUID);
    std::string val = readCharacteristicValue(charUUID);
    Serial.print("tap state: ");
    Serial.println(val.c_str());
    client.publish(frontGardenTapState,val.c_str());      
  }
}

void scanForDevice(void) {
  Serial.println("scanning for device");
  if(!pBLEScan){
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  }
  pBLEScan->setInterval(50);
  pBLEScan->setWindow(4000);
  pBLEScan->setActiveScan(false);
  pBLEScan->start(1800, false);
}



void changeBatteryReadFrequency(String i) {
  Serial.println(i);
  
  obtainServiceHandle(serviceUUID_btry_freq);
  obtainCharacteristicHandle(charUUID_btry_freq);
  uint8_t frequency = 0x01;

  if (i=="10 Min"){
    frequency = 0x01;
    
  } else if (i=="30 Min") {
    frequency = 0x02;
    
  } else if (i=="1 Hour") {
    frequency = 0x03;
    
  } else if (i=="6 Hour") {
    frequency = 0x04;
    
  } else if (i=="12 Hour") {
    frequency = 0x05;
  } else if (i=="24 Hour") {
    frequency = 0x06;
  }

  pRemoteCharacteristic->writeValue(frequency,true);

}

void reconnect() {
  // Loop until we're reconnected
  // Wait 5 seconds before retrying
  //unsigned long currentMillis = millis();
  //if (currentMillis - previousMillis >= 5000) {
  //  previousMillis = currentMillis;
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      if (client.connect("BleEsp",mqtt_user,mqtt_password)) {
        Serial.println("reconnected to mqtt broker");
        client.subscribe(frontGardenTapCmd);
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println("try again in 1 second");
        delay(1000);
      } 
    }
}

void reconnectAndSend(String topic,String val) {
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      if (client.connect("BleEsp",mqtt_user,mqtt_password)) {
        Serial.println("reconnected to mqtt broker");
        client.subscribe(frontGardenTapCmd);
        client.publish(topic.c_str(),val.c_str(),true);
        return;
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println("try again in 1 second");
        delay(1000);
        reconnectAndSend(topic,val);
      } 
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");

  String prefix = "change_battery_frequency_to_";
  int prefixlngth = prefix.length();

  payload[length] = '\0';
  String s = String((char*)payload);
  Serial.print("the message is: " );
  Serial.println(s.substring(0,24));
    if (s == "off"){ 
      //the below three lines disable notifications in custom service.
//      uint8_t val[] = {0x00, 0x00};
//      BLERemoteDescriptor* desc = pBLERemoteCharacteristic->getDescriptor((uint16_t)0x2902);
//      desc->writeValue(val, 2);
      Serial.println("deregistering tap notifications");
      disableTapNotifications();
      Serial.println("The tap will be turned off");
      setTapState(0x00);   
      //getTapState();         
    } else if(s.toInt() > 0 && s.toInt() <= 240)  {
      Serial.println("The tap will be turned on"); 
      uint8_t msgAsChar = (char) s.toInt();
      setTapState(msgAsChar);
      registerForNotifications();     
    } else if (s == "enable_battery_updates" ) { //register notifications on battery service
      enableBatteryNotifications();  
    }
    else if (s == "disable_battery_updates" ) {  // unregister notifications on battery service
      disableBatteryNotifications();   
    }
    else if (s.substring(0,24) == "change_battery_frequency" ) {
        changeBatteryReadFrequency(s.substring(prefixlngth));   
    } else if (s == "restart_esp32" ) {
      ESP.restart();      
    }
    else if (s == "get_tap_state" ) {
      getTapState();   
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


  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

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

void setupPubSubClient(void) {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.subscribe(frontGardenTapCmd);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  setupWifi();
  setupPubSubClient();

  if (!client.connected()){
    reconnectAndSend(frontGardenTapConnection,"not connected");
  } else{
    client.publish(frontGardenTapConnection,"not connected");
  }
  
  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P9);
 
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());

  scanForDevice();
  
} // End of setup.

// This is the Arduino main loop function.
void loop() {
  ArduinoOTA.handle();
  if(connected){
  if (!client.connected()) { reconnect(); } // check if pubsubclient is connected to broker before entering loop method
  client.loop();// pubsubclient loopchecking method
  } else {
    if (doConnect) {
      connectToServer();
    } else {
      scanForDevice();    
    }
  }
} // End of loop
