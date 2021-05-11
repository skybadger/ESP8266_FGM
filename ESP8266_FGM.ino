/*
This app measures the signal output from the Speake instruments fluxgate magnetometer.
This produces a 5v TTL square wave ouput whose period is proportional to the magnetic field. 
The sensor is sensitive enough to be used as an earth field sensor and detect or predict aurora.
The data is collected every measurement period, averaged and cached and these values used to calculate moving K values

The data  is relayed to a MQTT server to which a logging client should be subscribed.
 
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 The current state is stored in EEPROM and restored on bootup
*/
#include <DebugSerial.h>
#include <ESP.h>
#include <ESP8266WiFi.h>
#include <Wire.h>         //https://playground.arduino.cc/Main/WireLibraryDetailedReference
#include <Time.h>         //Look at https://github.com/PaulStoffregen/Time for a more useful internal timebase library
#include <esp8266_peri.h> //register map and access
#include <ArduinoJson.h>
#include <PubSubClient.h> //https://pubsubclient.knolleary.net/api.html
#define MAX_TIME_INACTIVE 0 //to turn off the de-activation of a telnet session
#include "RemoteDebug.h"  //https://github.com/JoaoLopesF/RemoteDebug

//Create a remote debug object
RemoteDebug Debug;

#include <ESP8266WebServer.h> //Needed for common_funcs. Web server will be needed later. 
#include <ESP8266HTTPUpdateServer.h>
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer updater;

//Ntp dependencies - available from v2.4
#include <time.h>
#include <sys/time.h>
#include <coredecls.h>
#define TZ              0       // (utc+) TZ in hours
#define DST_MN          60      // use 60mn for summer time in some countries
#define TZ_MN           ((TZ)*60)
#define TZ_SEC          ((TZ)*3600)
#define DST_SEC         ((DST_MN)*60)
time_t now; //use as 'gmtime(&now);'

//Hardware device system functions - reset/restart etc
#ifdef ESP8266_blah
extern "C" {
#include "ets_sys.h" //Base timer and interrupt handling 
#include "osapi.h"   //re-cast ets_sys names  - might be missing hw_timer_t - cast to uint32_t  or _ETSTIMER_ ?
#include "user_interface.h"
}
#endif

//Used by pulseIn
#include <limits.h>
#include "wiring_private.h"
#include "pins_arduino.h"

//Hardware device system functions - reset/restart etc
EspClass device;
long int nowTime, startTime, indexTime;

//Local Strings
const char* myHostname = "espFGM01";
const char* thisID = myHostname;

WiFiClient espClient;
PubSubClient client(espClient);
//Required by reconnectNB in common_funcs
volatile int timerSet = false;
volatile boolean timeoutFlag = false;
volatile boolean callbackFlag = false;

ETSTimer timeoutTimer, fineTimer, coarseTimer;
void onTimeoutTimer( void* ptr );
void onCoarseTimer ( void* ptr );
void onFineTimer   ( void* ptr );
volatile boolean coarseTimerFlag = false;
volatile boolean fineTimerFlag = false;

#include <SkybadgerStrings.h>
#include <Skybadger_common_funcs.h>

//Interrupt handler variables
const byte PulseCounterPin = 14;//Pin 14 on ESP-12E is D5 
const int MAXDATA = 1000; //max period is 25us - means 25ms to acquire. 
volatile uint32_t dataCache[MAXDATA];
volatile uint32_t lastClockValue = 0;
volatile uint32_t newClockValue = 0;
volatile bool newDataFlag = false;
//volatile int signalDirection = 0; 
  
//Acquired data storage and processing 
static int dataCount = 0;
static int dataIndex = 0;
static unsigned long avgSum = 0UL;
static float runningVar = 0.0F, varSum = 0.0F;
static float runningAvg = 0.0F;
static uint32_t datum = 0;
static uint32_t lastDatum = 0;

uint32_t inline ICACHE_RAM_ATTR myGetCycleCount()
{
    uint32_t ccount;
    __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
    return ccount;
}

/*
  Interrupt handler function captures clock counts at rising and falling edges on input pin.
  Sets flag for main loop to calculate pule width. 
  Check for potential issue of loop() taking multiple pulse count lengths. 
  Address using countHandled flag ?
  https://github.com/esp8266/esp8266-wiki/wiki/gpio-registers
  Measured on esP8266-12 to take 300K 160Mhz clocks per loop iteration  - 20ms. 
*/
/*
 * This variant tries to capture all the data in the ISR for post-processing in the loop handler. 
 * Only register for falling edge processing - handle entire period. 
 */
void inline ICACHE_RAM_ATTR pulseCounter(void)
{
  static volatile int dataCtr = 0;
  static volatile uint32_t localDatum = 0;
  
  if( !newDataFlag ) //loop handler has signalled post-process is complete - get more. 
  {
    if( dataCtr < MAXDATA ) //we haven't finished collecting it yet.
    {
      newClockValue =  myGetCycleCount();//device.getCycleCount();
      localDatum = newClockValue - lastClockValue; 
      lastClockValue = newClockValue;
      
      if ( localDatum > 1100 && localDatum < 5000 ) //sanity check on range
      {
        dataCache[dataCtr] = localDatum;
        dataCtr++;    
      }
    }
    else //we are done collecting 
    {
      //set flag for post-processing when the buffer is full.
      newDataFlag = true;
      dataCtr = 0;      
    }
  }
  else //flag is still high
  {
    //do nothing until loop handler handles the data 
    ;
  }
}

//Original - uses rises and falling flags on interrupt attach call. 
//void inline ICACHE_RAM_ATTR pulseCounter(void)
//{
//  //if edge rising, reset counter
//  signalDirection = GPIP(PulseCounterPin); //READ_PERI_REG( 0x60000318, PulseCounterPin );
//  if ( signalDirection == 1 ) 
//  {
//    newClockValue =  myGetCycleCount();//device.getCycleCount();
//  }
//  else //this must be a falling edge, so read clock counter to determine time since last rising edge
//  {
//    lastClockValue = newClockValue;
//    newClockValue = myGetCycleCount();//device.getCycleCount();
//   
//    //set flag.
//    newDataFlag = 1;
//  }
//}
//digitalRead(pin)
#define WAIT_FOR_PIN_STATE(state) \
    while ( GPIP(pin) != (state)) { \
        if (myGetCycleCount() - start_cycle_count > timeout_cycles) { \
            return 0; \
        } \
        optimistic_yield(5000); \
    }
    
// max timeout is 27 seconds at 160MHz clock and 54 seconds at 80MHz clock
uint32_t myPulseIn( uint8_t pin, uint8_t state, unsigned long timeout)
{
    const uint32_t max_timeout_us = clockCyclesToMicroseconds(UINT_MAX);
    if (timeout > max_timeout_us) 
    {
        timeout = max_timeout_us;
    }
    const uint32_t timeout_cycles = microsecondsToClockCycles(timeout);
    const uint32_t start_cycle_count = myGetCycleCount();
    
    //sync with external edge
    WAIT_FOR_PIN_STATE( !state );
    
    //get time when edge changes to desired state
    WAIT_FOR_PIN_STATE(state);
    const uint32_t pulse_start_cycle_count = myGetCycleCount();
    
    //Wait for next edge change to isolate just the up or down part of a rectangular wave - ie phase 
    WAIT_FOR_PIN_STATE(!state);
    //Or wait until next same state detected - ie entire period. 
    WAIT_FOR_PIN_STATE(state);
    
    //return time measured by system clocks. 
    return (myGetCycleCount() - pulse_start_cycle_count);
}

void loop() 
{
  static boolean dataProcessed = false;

  debugV("Cycle: %u \n", ESP.getCycleCount() );
  
  if( coarseTimerFlag )
  {
    //Use the timer to cause new data to be collected. 
    if( dataProcessed ) 
    {
      dataProcessed  = false;
      if( newDataFlag )
        newDataFlag = false; 
    }
    
    coarseTimerFlag = false;
  }
  
  if( newDataFlag && !dataProcessed )
  {
    dataIndex = 0;
    runningAvg = 0.0F;
    runningVar = 0.0F;
    avgSum = 0;
    varSum = 0.0F;
    //calc the mean
    for ( dataIndex = 0; dataIndex < MAXDATA; dataIndex++ )
    {  
       avgSum += dataCache[dataIndex];
    }
    runningAvg =  avgSum/((float)MAXDATA);     
    //calc the variance
    for ( dataIndex = 0; dataIndex < MAXDATA; dataIndex++ )
    {  
       varSum += (float) pow( ( (float) dataCache[dataIndex] - runningAvg), 2.0);
    }
    runningVar = varSum/((float)MAXDATA);       
    debugI( "New datum - mean: %f, var: %f \n", runningAvg, runningVar );

// Old code for updating live values per reading.     
//    else //buffer is full and operating as a circular buffer
//    {
//       //calc new average - take off oldest point in buffer and add in the new one.
//       lastDatum = (dataIndex == 0 ) ? dataCache[MAXDATA-1]: dataCache[dataIndex -1]; 
//       
//       avgSum -= lastDatum;
//       varSum -= (float) pow( ((float) lastDatum - runningAvg), 2.0 );
//
//       avgSum += datum;
//       runningAvg = avgSum/((float)dataCount);
//       
//       varSum += (float) pow( ( (float) datum - runningAvg), 2.0);
//       runningVar = varSum/((float)dataCount);
//
//       Serial.printf("New old: %lu, new:%lu, mean: %f, var: %f \n", lastDatum, datum, runningAvg, runningVar );
//    }  

    dataProcessed = true;
  }

  uint32_t length = myPulseIn( PulseCounterPin, 1, 8000/*us*/ );
  debugD( "Length: %u \n", length );
  
  if ( !client.connected() )
  {
    debugW( "Waiting on client sync reconnection ");
    //reconnect();
    reconnectNB();
    debugI( "MQTT reconnected  ");
    client.setCallback( callback );
    client.subscribe(inTopic);
  }
  else
  {
    //Service MQTT keep-alives
    client.loop();
    if (callbackFlag ) 
    {
        //publish results
        publishHealth();
        publishData();
        callbackFlag = false;
    }
  }

  //If there are any web client connections - handle them.
  server.handleClient();

  // Remote debug over WiFi
  Debug.handle();
  // Or
  //debugHandle(); // Equal to SerialDebug  

  //Show welcome message
 
  Debug.setSerialEnabled(false);
}

void onFineTimer( void* pArg )
{
  //Read command list and apply. 
  fineTimerFlag = true;
}

void onCoarseTimer( void* pArg )
{
  //Read command list and apply. 
  coarseTimerFlag = true;
}

//Used to complete timeout actions. 
void onTimeoutTimer( void* pArg )
{
   ;;//fn moved to Shutter controller  - delete later if not used. 
   timeoutFlag = true;
}

void setup() 
{
  Serial.begin( 115200 );
  //Serial.begin( 115200, SERIAL_8N1, SERIAL_TX_ONLY);
  Serial.println("connecting wifi");

  //Start NTP client
  configTime(TZ_SEC, DST_SEC, timeServer1, timeServer2, timeServer3 );
  Serial.println( "Time Services setup");

  // Connect to wifi 
  setupWifi();                   

  //Debugging over telnet setup
  // Initialize the server (telnet or web socket) of RemoteDebug
  //Debug.begin(HOST_NAME, startingDebugLevel );
  Debug.begin( WiFi.hostname().c_str(), Debug.ERROR ); 
  Debug.setSerialEnabled(true);//until set false 
  // Options
  // Debug.setResetCmdEnabled(true); // Enable the reset command
  // Debug.showProfiler(true); // To show profiler - time between messages of Debug
  //In practice still need to use serial commands until debugger is up and running.. 
  debugE("Remote debugger enabled and operating");

  // Device is an ESP8266-01 which has only 4 output pins, two are used to do serial i/f and remaining two take part in programming 
  // so we are careful in choice of pins and direction/states
  
  //Pins mode and direction setup
  pinMode( PulseCounterPin, INPUT);           // set pin to input
  //digitalWrite( PulseCounterPin, HIGH);       // turn on pullup resistors
  
  //Open a connection to MQTT
  DEBUGS1("Configuring MQTT connection to :");DEBUGSL1( mqtt_server );
  client.setServer( mqtt_server, 1883 );
  Serial.printf(" MQTT settings id: %s user: %s pwd: %s\n", thisID, pubsubUserID, pubsubUserPwd );
  client.connect( thisID, pubsubUserID, pubsubUserPwd ); 
  //Create a timer-based callback that causes this device to read the local i2C bus devices for data to publish.
  client.setCallback( callback );
  client.subscribe( inTopic );
  publishHealth();
  client.loop();
  DEBUGSL1("Configured MQTT connection");
 
  //Using an input on change detector interrupt
  //try just one edge - will count the envelope frequency
  //then try both edges and set for falling followed by rising. 
  //Use same handler for both. 
  //attachInterrupt( digitalPinToInterrupt(PulseCounterPin), pulseCounter, RISING );  
  attachInterrupt( digitalPinToInterrupt(PulseCounterPin), pulseCounter, FALLING );  

  //Start web server
  updater.setup( &server );
  server.begin();
  Serial.println( "webserver setup complete");

  //setup interrupt-based 'soft' alarm handler for dome state update and async commands
  ets_timer_setfn( &fineTimer,    onFineTimer,    NULL ); 
  ets_timer_setfn( &coarseTimer,  onCoarseTimer,  NULL ); 
  ets_timer_setfn( &timeoutTimer, onTimeoutTimer, NULL ); //MQTT non-blocking handler

  //ets_timer_arm_new( &fineTimer,   1000,      1/*repeat*/, 1);//millis
  ets_timer_arm_new( &coarseTimer, 5000,     1/*repeat*/, 1);//millis
  //ets_timer_arm_new( &timeoutTimer, 2500, 0/*one-shot*/, 1);//set by MQTT NB handler
  
  debugV("Setup: %u\n", ESP.getCycleCount() );

  //It would be really nice to find a way to feed a HW counter/timer directly 
  //Or we can use a variation on the pulseIn function (core_esp8266_wiring_pulse.cpp)
  //unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
}

void setupWifi(void)
{
  //Setup Wifi
  int zz = 00;
  WiFi.mode(WIFI_STA);
  WiFi.hostname( myHostname );

  scanNet( );
 
  WiFi.begin( ssid4, password4 );  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);//This delay is essentially for the DHCP response. Shouldn't be required for static config.
    Serial.print(".");
    if ( zz++ > 200 )
    {
      Serial.println("Restarting to try to find a connection");
      device.restart();
    }
  }
  Serial.println("WiFi connected");
  Serial.printf("SSID: %s, Signal strength %i dBm \n\r", WiFi.SSID().c_str(), WiFi.RSSI() );
  Serial.printf("Hostname: %s\n\r",       WiFi.hostname().c_str() );
  Serial.printf("IP address: %s\n\r",     WiFi.localIP().toString().c_str() );
  Serial.printf("DNS address 0: %s\n\r",  WiFi.dnsIP(0).toString().c_str() );
  Serial.printf("DNS address 1: %s\n\r",  WiFi.dnsIP(1).toString().c_str() );

  //Setup sleep parameters
  // LIGHT_S
  //wifi_set_sleep_type(LIGHT_SLEEP_T);
  wifi_set_sleep_type(NONE_SLEEP_T);
  delay(5000);

  Serial.println("WiFi setup complete & connected");
}

void publishHealth( void )
 {
  String outTopic;
  String output;
  String timestamp;
  
  //publish to our device topic(s)
  DynamicJsonBuffer jsonBuffer(256);  
  JsonObject& root = jsonBuffer.createObject();

  getTimeAsString2( timestamp );
  root["time"] = timestamp;

  // Once connected, publish an announcement...
  root["hostname"] = myHostname;
  root["message"] = "FGM Magnetometer operating";
  root.printTo( output );
  
  outTopic = outHealthTopic;
  outTopic.concat( myHostname );
  
  if( client.publish( outTopic.c_str(), output.c_str(), true ) )  
  {
    debugI( "Topic published: %s ", output.c_str() ); 
  }
  else
  {
    debugW( "Topic failed to publish: %s ", output.c_str() );   
  }
 }

/* MQTT callback for subscription and topic.
 * Only respond to valid states ""
 */
void publishData(void)
{
  String output;
  String outTopic;
  String timestamp = "";
  
  DynamicJsonBuffer jsonBuffer(256);  
  getTimeAsString( timestamp );

  //publish to our device topic(s)
  JsonObject& root = jsonBuffer.createObject();
  root["time"] = timestamp;
  root["Bz"] = runningAvg;
  root["Bz_err"] = sqrt(runningVar);
  root["Hostname"] = myHostname;
  root["sensor"] = "FGM-3h";
  root.printTo( output );
  
  outTopic = outSenseTopic ;
  outTopic.concat("magnetometer/");
  outTopic.concat(myHostname);
  client.publish( outTopic.c_str(), output.c_str() );
}

/* MQTT callback for subscription and topic.
 * Only respond to valid states ""
 * Publish under ~/skybadger/sensors/<sensor type>/<host>
 * Note that messages have an maximum length limit of 18 bytes - set in the MQTT header file. 
 */
void callback(char* topic, byte* payload, unsigned int length) 
{  
  //set callback flag
  callbackFlag = true;  
 }
