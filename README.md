# Battery Powered BLE Irrigation Controller Project With the ESP32 as the Central Gateway Device

Battery powered irrigation project where the ESP32 is the interface between the nRF52832 (BLE) microcontroller (see [this project](https://github.com/rowdy15/battery-powered-ble-irrigation-controller-project-with-the-nRF52832)) and an MQTT Broker. For more information about the project, please see the nRF52832 repository.

In my particular use case, I am using the smart home automation software *Home Assistant* to communicate with the MQTT broker to send and recieve messages to and from the nRF52832 microcontroller via the ESP32 microcontroller.

If you need help with setting up an MQTT broker and integrating it with Home Assistant, check out [this](https://www.youtube.com/watch?v=AsDHEDbyLfg) link.

If you need help with setting up Home Assistant, check out [this](https://www.youtube.com/watch?v=qnCRcGTznXs) video. NOTE: You will need a Raspberry Pi 3 for the setup in this video.  (But Home Assistant can be installed and setup on just about anything if you want to get started without a raspberry pi).
