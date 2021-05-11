"ESP8266_FGM Speake magnetometer sensor over WiFi connection" 
This application runs on the ESP8266 wireless device and captures and measures t9he square wave output from the magnetometer to directly measure the local field intensity. For use in Aurora sensing and geo magnetometry. 
Intention is to cause this to report via a MQTT serviec to a node-red dashboard which provides alerting functions in case of impending aurora. 
Requires Arduino 1.6, ESP8266 V2.., Arduino MQTT
Access by - Serial port provides monitor, wifi is used for MQTT reporting only. 
