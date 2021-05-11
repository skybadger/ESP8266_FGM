<h3>ESP8266_FGM Speake magnetometer sensor over WiFi connection</h3>
<p>This application measures the local earth mangnetic field using a Speake instruments fluxgate Magnetometer. <br/>
  This application runs on the ESP8266 wireless device and captures and measures the period of the square wave output from the magnetometer to directly measure the local field intensity. For use in Aurora sensing and geo-magnetometry. <br/>
This device reports its measurements and its health status to a local Node-red MQTT service hosting a system dashboard which provides alerting functions in case of impending aurora. <br/>
  
<h3> dependencies</h3>
<ul>
  <li>Arduino 1.6+  </li>
  <li>ESP8266 V2.2+</li>
  <li>ESP8266Pubsub MQTT client for ESP8266</li>
  <li>ArduinoJSON v5 JSON parser and printer. </li>
  <li>Remotedebug telnet remote debug service </li>
  </ul>
  
<h3>Use</h3>
<p>Access by - Serial port provides monitor,   wifi is used for MQTT reporting only. </p>
