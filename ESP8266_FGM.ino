/*
https://www.bakke.online/index.php/2017/05/22/reducing-wifi-power-consumption-on-esp8266-part-3/
This app measures the signal output from the Speake instruments fluxgate magnetometer.
This produces a 5v TTL square wave ouput whose period is proportional to the magnetic field. 
The sensor is sensitive enough to be used as an earth field sensor and detect or predict aurora.
The data is collected every measurement period, averaged and cached and these values used to calculate moving K values

The data  is relayed to a MQTT server to which a logging client should be subscribed.
 
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 The current state is stored in EEPROM and restored on bootup
 Frequency of ESP clock is either 80b MHz or 160MhZ Hence one clock is 12.5nS or 6.25 nS
 Frequency of FGM varies between 160 and 25 KHz 
 Hence longest count can be is .. 40 microsecs @12.5ns = 3200 or @6.25nS = 6400
 Shortest valid count can be is .. 500 or 1000
 
*/
#define ESP8266_01 //Set the cpu model
//#define LOW_POWER_OPS - not unless using pin7 attached to /RST on ESP8266-1 otherwise no return from DEEP SLEEP
#if defined LOW_POWER_OPS
#define DEBUG_DISABLED - Disable for low power operations. 
#endif 
#define DEBUG_ESP_MH //Enable debug output 
#define MAX_TIME_INACTIVE 0 //to turn off the de-activation of a telnet session
//#define DEBUG_DISABLED //if disabled, all debug output goes to serial port. if all debugging is disabled, no output

#include "RemoteDebug.h"  //https://github.com/JoaoLopesF/RemoteDebug && https://github.com/karol-brejna-i/RemoteDebug
#include <DebugSerial.h>
#include <SkybadgerStrings.h>

#include <ESP.h>
#include <ESP8266WiFi.h>
#include <Wire.h>         //https://playground.arduino.cc/Main/WireLibraryDetailedReference
#include <Time.h>         //Look at https://github.com/PaulStoffregen/Time for a more useful internal timebase library
#include <esp8266_peri.h> //register map and access
#include <ArduinoJson.h>
#include <PubSubClient.h> //https://pubsubclient.knolleary.net/api.html

//Create a remote debug object
#if !defined DEBUG_DISABLED
RemoteDebug Debug;
#endif 

#include <ESP8266WebServer.h> //Needed for common_funcs. Web server will be needed later. 
#include <ESP8266HTTPUpdateServer.h>
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer updater;

//Ntp dependencies - available from v2.4
#include <time.h>
#include <sys/time.h>
#include <coredecls.h>
#define TZ              0       // (utc+) TZ in hours
#define DST_MN          00      // use 60mn for summer time in some countries
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
#define SLEEPTIME 60000000 //usecs

#include <Skybadger_common_funcs.h>

//Interrupt handler variables
#if defined ESP8266_01
const byte PulseCounterPin = 3; //Rx , pin 1, GPIO 3
//const byte PulseCounterPin = 0; //pin2 GPIO0
//const byte PulseCounterPin = 0; //Tx pin8 GPIO1
//const byte PulseCounterPin = 0; //pin3 GPIO2
#else
const byte PulseCounterPin = 14;//Pin 14 on ESP-12E is D5 
#endif

//Acquired data 
//max period is 40us - means 40 milliseconds to acquire. Should aim at something that provides the same figure whether fast or slow. 
const int MAXDATA = 1000; 
volatile uint32_t dataCache[MAXDATA];
volatile uint32_t lastClockValue = 0;
volatile uint32_t newClockValue = 0;
volatile bool newDataFlag = true;
volatile int signalDirection = 0; 
  
//Acquired data storage and processing 
int dataCount = 0;
int dataIndex = 0;
unsigned long avgSum = 0UL;
float runningAvg = 0.0F;
float runningVar = 0.0F;
float varSum = 0.0F;
uint32_t datum = 0;
uint32_t lastDatum = 0;

//uint32_t inline ICACHE_RAM_ATTR myGetCycleCount()
uint32_t inline IRAM_ATTR myGetCycleCount()
{
    uint32_t ccount;
    __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
    return ccount;
}

/*
  Interrupt handler function captures clock counts at rising and falling edges on input pin.
  Sets flag for main loop to calculate pulse width. 
  Check for potential issue of loop() taking multiple pulse count lengths. 
  Address using countHandled flag ?
  https://github.com/esp8266/esp8266-wiki/wiki/gpio-registers
  Measured on esP8266-12 to take 300K 160Mhz clocks per loop iteration  - 20ms. 
*/

/*
 * @function pulsecounter
 * @Description This variant tries to capture all the data in the ISR for post-processing in the loop handler. 
 * Only register for falling edge processing - measure entire period. 
 * clock counts are 12.5ns/clock @80MHz and 6.25ns at 160 MHz
 * https://sub.nanona.fi/esp8266/timing-and-ticks.html
 */
//void inline ICACHE_RAM_ATTR pulseCounter(void)
void inline IRAM_ATTR pulseCounter(void)
{
  static volatile int dataCtr = 0;
  static volatile uint32_t localDatum = 0;
  
  if( !newDataFlag ) //when loop handler signals post-processing is complete get more data. 
  {
    if( dataCtr < MAXDATA ) //we haven't finished collecting it yet.
    {
      newClockValue =  myGetCycleCount();//device.getCycleCount();
      localDatum = newClockValue - lastClockValue; 
      lastClockValue = newClockValue;
      
      if ( localDatum > 500 && localDatum < 6400 ) //sanity check on range
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
}

//Original - uses rising and falling flags on interrupt attach call. 
//void inline ICACHE_RAM_ATTR pulseCounterComplex(void)
void inline IRAM_ATTR pulseCounterBipolar(void)
{
  //Need to ensure we measure rising edge first. 
  static volatile bool risingEdgeDetected = false;
  static volatile int dataCtr = 0;
  static volatile uint32_t localDatum = 0;

  signalDirection = GPIP(PulseCounterPin); //READ_PERI_REG( 0x60000318, PulseCounterPin );
  if ( signalDirection == 1 && !risingEdgeDetected )
    risingEdgeDetected = true;
  else 
    return; 

  if( !newDataFlag ) //when loop handler signals post-processing is complete get more data. 
  {  
    //if edge rising, reset counter
    if ( signalDirection == 1 ) 
    {
      lastClockValue =  myGetCycleCount();//device.getCycleCount();
    }
    else //this must be a falling edge, so read clock counter to determine time since last rising edge
    {
      newClockValue = myGetCycleCount();//device.getCycleCount();   
      //sanity check on range
      if( dataCtr < MAXDATA ) //we haven't finished collecting it yet.
      {
        localDatum  = newClockValue - lastClockValue;
        //Do this for each pair of calls in case we get the jitters. Otherwise could just do for every start of acquisition.
        risingEdgeDetected = false;
        if ( localDatum > 500 && localDatum < 6400 ) 
        {
          dataCache[dataCtr] = localDatum;
          dataCtr++;    
        }
      }
      else //set flag.
        newDataFlag = 1;
    }
  }
}

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
    //WAIT_FOR_PIN_STATE(state);
    
    //return time measured by system clocks. 
    return ( myGetCycleCount() - pulse_start_cycle_count);
}

/*
Wake up the processing loop on a timed basis. 
collect the data and then connect to wifi
Send the data via MQTT and then disconnect again. 
*/
void loop() 
{
  static boolean dataProcessed = false;
  debugV("Cycle: %u \n", ESP.getCycleCount() );
  delay(10);
  
  //Coarse timer must be longer than MAXDATA periods of the longest period signal or expect to be called multiple times over an acquisition time. 
  if ( coarseTimerFlag ) 
  {  
    newDataFlag = false;//Start interrupt handler while we reconnect. 
    setupWifi();

    //Use the timer to cause new data to be collected and to know when it is not. 
    //newDataFlag is set when interrupts have completed collecting a set of data 
    while( !newDataFlag ) //Wait for handler completion
      ;
    if ( newDataFlag )
    {
      //Show what the current length of a pulse is - moved to here to avoid interference with interrupt-based acquisition 
      uint32_t length = myPulseIn( PulseCounterPin, 1, 250 /*us*/ );
      debugD( "Length: %u \n", length );
    }
      
    if( newDataFlag )
    {
      dataIndex = 0;
      float Avg = 0.0F;
      float Var = 0.0F;
      avgSum = 0;
      varSum = 0.0F;
      
      //calc the mean
      for ( dataIndex = 0; dataIndex < MAXDATA; dataIndex++ )
      {  
         avgSum += dataCache[dataIndex];
      }
      Avg = avgSum/((float)MAXDATA);     
      
      //calc the variance
      for ( dataIndex = 0; dataIndex < MAXDATA; dataIndex++ )
      {  
         varSum += (float) pow( ( (float) dataCache[dataIndex] - Avg), 2.0);
      }
      Var = varSum/((float)MAXDATA);       
      debugI( "New datum - mean: %f, var: %f \n", Avg, Var );
  
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
    }
 
    if ( !client.connected() )
    {
      debugW( "Waiting on client sync reconnection ");
      //reconnect();
      reconnectNB();
      debugI( "MQTT reconnected  ");
      client.setCallback( callback );
      client.subscribe(inTopic);
    }
  
    //Service MQTT keep-alives
    client.loop();
    //publish results
    publishHealth();
    publishData();
    callbackFlag = false;
    client.disconnect();
  
    //If there are any web client connections - handle them.
    server.handleClient();
  }

  // Remote debug over WiFi
  Debug.handle();
  // Or
  //debugHandle(); // Equal to SerialDebug  

  //Update flags 
  coarseTimerFlag = false;
  
  //if we have data and have sent it, we can go back to sleep, otherwise loop again until we have. Leave newdataflag true to cause interrupt to wait. 
  //Turn off the Wifi connection again for low power operation
  Serial.println("Entering deep sleep now.");
  WiFi.disconnect( true );
  delay( 1 );
#if defined LOW_POWER_OPS  
  // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up, can't use this without connecting pin 7 to  /rst on ESP8266-1
  //Much easier on -12 and ESP32
  ESP.deepSleep( SLEEPTIME, WAKE_RF_DISABLED );
#endif
  //Leave newDataflag high until next coarseTimerFlag
}

void onFineTimer( void* pArg )
{
  //
  fineTimerFlag = true;
}
//This handler sets up the thread processing time. 
void onCoarseTimer( void* pArg )
{
  //
  coarseTimerFlag = true;
}

//Used to complete timeout actions. 
void onTimeoutTimer( void* pArg )
{
   //
   timeoutFlag = true;
}

volatile static bool firstBootSetup = true;

//One time setup after wake up of device. 
void setup() 
{
  //Do this to ensure wifi stays off until its required per loop transmit. 
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay( 1 );
  
#if !defined LOW_POWER_OPS
  setupWifi();
#endif
  
  //Serial.begin( 115200 );
  Serial.begin( 115200, SERIAL_8N1, SERIAL_TX_ONLY);
  Serial.println("connecting wifi");
 
#if !defined DEBUG_DISABLED
  //Debugging over telnet setup
  // Initialize the server (telnet or web socket) of RemoteDebug
  Debug.begin( WiFi.hostname().c_str(), Debug.ERROR ); 
  Debug.setSerialEnabled(true);//until set false 
  // Options
  // Debug.setResetCmdEnabled(true); // Enable the reset command
  // Debug.showProfiler(true); // To show profiler - time between messages of Debug
  //In practice still need to use serial commands until debugger is up and running.. 
#endif 
  debugE("Remote debugger enabled and operating");

  // Target Device is an ESP8266-01 which has only 4 output pins, 
  // two are used to do serial i/f and remaining two take part in programming 
  // so we are careful in choice of pins and direction/states
  
  //Pins mode and direction setup
  pinMode( PulseCounterPin, INPUT);           // set pin to input
  //digitalWrite( PulseCounterPin, HIGH);     // turn on pullup resistors
  
  //Configure the connection to MQTT
  DEBUGS1("Configuring MQTT connection to :");DEBUGSL1( mqtt_server );
  client.setServer( mqtt_server, 1883 );
  Serial.printf(" MQTT settings id: %s user: %s pwd: %s\n", thisID, pubsubUserID, pubsubUserPwd );
  
  //Create a timer-based callback that causes this device to read the data to publish.
  //Not using this in low power mode 
  //client.setCallback( callback );
  client.connect( thisID, pubsubUserID, pubsubUserPwd ); 
  //Not subscribing in low power mode 
  //client.subscribe( inTopic );  
  //publishHealth();
  //client.loop();
  client.disconnect( ); //valid ? 
  
  DEBUGSL1("Configured MQTT connection");
 
  //Start update web server
  //updater.setup( &server );
  //server.begin();
  //Serial.println( "webserver setup complete");
  
  //Using an input on change detector interrupt
  //try just one edge - will count the envelope frequency
  //then try both edges and set for falling followed by rising. 
  //Use same handler for both. 
  attachInterrupt( digitalPinToInterrupt(PulseCounterPin), pulseCounterBipolar, RISING );  
  attachInterrupt( digitalPinToInterrupt(PulseCounterPin), pulseCounterBipolar, FALLING );  

  //setup interrupt-based 'soft' alarm handler for state update and async commands
  ets_timer_setfn( &fineTimer,    onFineTimer,    NULL ); 
  ets_timer_setfn( &coarseTimer,  onCoarseTimer,  NULL ); 
  ets_timer_setfn( &timeoutTimer, onTimeoutTimer, NULL ); //MQTT non-blocking handler

  //ets_timer_arm_new( &fineTimer,   10000,      1/*repeat*/, 1);//millis
  ets_timer_arm_new( &coarseTimer,   60000,     1/*repeat*/, 1);//millis - 60 secs
  //ets_timer_arm_new( &timeoutTimer, 2500, 0/*one-shot*/, 1);//set by MQTT NB handler

  //Keep high to halt acquisition
  newDataFlag = true; 
}

void setupWifi(void)
{
  //Setup Wifi
  int zz = 00;

  Serial.println ( "Enabling WiFi" );

  WiFi.forceSleepWake();
  delay( 1 );
  if ( firstBootSetup ) 
  {
	  WiFi.persistent( true );
	  firstBootSetup = false;
  }
  else
  {
    // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
	  WiFi.persistent( false );
  }

	WiFi.mode( WIFI_STA );
	WiFi.hostname( myHostname );
	WiFi.begin( ssid2, password2 );  
	Serial.println("Connecting");

  //scanNet( );
 
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);//This delay is essentially for the DHCP response. Shouldn't be required for static config.
    Serial.print(".");
    if ( zz++ == 300 )
    {
      WiFi.disconnect();
  		delay( 10 );
  		WiFi.forceSleepBegin();
  		delay( 10 );
  		WiFi.forceSleepWake();
  		delay( 10 );
  		WiFi.begin( ssid2, password2);
    }
  	if ( zz >= 310 ) 
  	{
  		//restart
  		device.restart();
  	}
  }
  Serial.println("WiFi connected");
  Serial.printf("SSID: %s, Signal strength %i dBm \n\r", WiFi.SSID().c_str(), WiFi.RSSI() );
  Serial.printf("Hostname: %s\n\r",       WiFi.hostname().c_str() );
  Serial.printf("IP address: %s\n\r",     WiFi.localIP().toString().c_str() );
  Serial.printf("DNS address 0: %s\n\r",  WiFi.dnsIP(0).toString().c_str() );
  Serial.printf("DNS address 1: %s\n\r",  WiFi.dnsIP(1).toString().c_str() );

  //Start NTP client
  configTime(TZ_SEC, DST_SEC, timeServer1, timeServer2, timeServer3 );
  Serial.println( "Time Services setup");

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
