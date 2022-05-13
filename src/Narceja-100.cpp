/*
 __    _  _______  ______    _______  _______      ___  _______ 
|  |  | ||   _   ||    _ |  |       ||       |    |   ||   _   |
|   |_| ||  |_|  ||   | ||  |       ||    ___|    |   ||  |_|  |
|       ||       ||   |_||_ |       ||   |___     |   ||       |
|  _    ||       ||    __  ||      _||    ___| ___|   ||       |
| | |   ||   _   ||   |  | ||     |_ |   |___ |       ||   _   |
|_|  |__||__| |__||___|  |_||_______||_______||_______||__| |__|

 Name:     Narceja 
 Date:     MAR 2022
 Author:   Flavio L Puhl Jr <flavio_puhl@hotmail.com> 
 GIT:      
 About:    
 
Update comments                                      
+-----------------------------------------------------+------------------+---------------+
|               Feature added                         |     Version      |      Date     |
+-----------------------------------------------------+------------------+---------------+
| Initial Release                                     |      1.0.0       |     MAR/22    |
|                                                     |                  |               |
|                                                     |                  |               |
+-----------------------------------------------------+------------------+---------------+


Library versions                                       
+-----------------------------------------+------------------+-------------------------- +
|       Library                           |     Version      |          Creator          |
+-----------------------------------------+------------------+-------------------------- +
|	PubSubClient                            |     @^2.8        |      knolleary            |
|	ArduinoJson                             |     @^6.18.5     |      bblanchon            |
|	NTPClient                               |     @^3.1.0      |      arduino-libraries    |
+-----------------------------------------+------------------+-------------------------- +

Upload settings 
+----------------------------------------------------------------------------------------+
| PLATFORM: Espressif 32 (3.3.0) > WeMos D1 MINI ESP32                                   |
| HARDWARE: ESP32 240MHz, 320KB RAM, 4MB Flash                                           |
| PACKAGES:                                                                              |
|  - framework-arduinoespressif32 3.10006.210326 (1.0.6)                                 |
|  - tool-esptoolpy 1.30100.210531 (3.1.0)                                               |
|  - toolchain-xtensa32 2.50200.97 (5.2.0)                                               |
|                                                                                        |
| RAM:   [=         ]  12.9% (used 42324 bytes from 327680 bytes)                        |
| Flash: [======    ]  64.0% (used 839394 bytes from 1310720 bytes)                      |
+----------------------------------------------------------------------------------------+

*/

/*+--------------------------------------------------------------------------------------+
 *| Libraries                                                                            |
 *+--------------------------------------------------------------------------------------+ */

// Libraries built into IDE

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <EEPROM.h>                                         // read and write from flash memory

#include "esp_wifi.h"
#include "esp_bt_main.h"                                    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_bt_main.html#_CPPv421esp_bluedroid_disablev
#include "esp_bt.h"


/*+--------------------------------------------------------------------------------------+
 *| Constants declaration                                                                |
 *+--------------------------------------------------------------------------------------+ */

int LED_BOARD = 2;
String swversion = __FILE__;

const char *ssid =  "CasaDoTheodoro1";                         // name of your WiFi network
const char *password =  "09012011";                            // password of the WiFi network

const char *ID = "NarcejaDev";                                 // Name of our device, must be unique
const char *TOPIC = "Narceja/data";                            // Topic to subcribe to
const char* BROKER_MQTT = "broker.hivemq.com";                 // MQTT Cloud Broker URL
//const char* BROKER_MQTT = "mqtt.eclipseprojects.io";          // MQTT Cloud Broker URL

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

WiFiClient wclient;
PubSubClient client(wclient);                                 // Setup MQTT client

#define uS_TO_S_FACTOR 1000000                                // Conversion factor for micro seconds to seconds 
#define TIME_TO_SLEEP  60                                     // Time ESP32 will go to sleep (in seconds)
#define Threshold 40                                          // Greater the value, more the sensitivity 

RTC_DATA_ATTR int bootCount = 0;

String wakeup_msg;

int meas_counter = 0;

int batt_level_pin = 33;                                       // Voltage is connected to GPIO 33 (Analog ADC1_CH5)
float RatioFactor = 2;                                         // Voltage divider Factor
float batt_level_volts;

/*+--------------------------------------------------------------------------------------+
 *| Method to print the reason by which ESP32 has been awaken from sleep                 |
 *+--------------------------------------------------------------------------------------+ */

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : 
      log_i("Wakeup caused by external signal using RTC_IO");
      wakeup_msg = "WAKEUP_EXT0";
        break;
    case ESP_SLEEP_WAKEUP_EXT1 : 
      log_i("Wakeup caused by external signal using RTC_CNTL");
      wakeup_msg = "WAKEUP_EXT1";
        break;
    case ESP_SLEEP_WAKEUP_TIMER : 
      log_i("Wakeup caused by timer");
      wakeup_msg = "WAKEUP_TIMER";
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :
      log_i("Wakeup caused by touchpad");
      wakeup_msg = "WAKEUP_TOUCHPAD";
        break;
    case ESP_SLEEP_WAKEUP_ULP : 
      log_i("Wakeup caused by ULP program");
      wakeup_msg = "WAKEUP_ULP";
        break;
    default : 
      log_i("Wakeup was not caused by deep sleep");
      wakeup_msg = "WAKEUP_NOT_BY_SLEEP";
        break;
  }
}

/*+--------------------------------------------------------------------------------------+
 *| Prepare and go to deep sleep                                                         |
 *+--------------------------------------------------------------------------------------+ */

void prepareAndgoDeepSleep() {

  delay(500);
  //Serial.flush();                                           // Waits for the transmission of outgoing serial data to complete. 

  if(esp_bluedroid_disable()){                                // Disable bluetooth
    log_i("Bluetooth disabled");
  } else {
    log_i("FAILED to disable bluetooth");
  }
  
  if(esp_bt_controller_disable()){                            // Disable BT controller
    log_i("Bluetooth controller disabled");
  } else {
    log_i("FAILED to disable bluetooth controller");
  }
                                   
  if(esp_wifi_disconnect() == ESP_OK){                        // Stop Wifi
    log_i("WiFi disconnected");
  } else {
    log_i("FAILED to disconnect wifi");
  }

  if(esp_wifi_stop() == ESP_OK){                              // Stop Wifi
    esp_wifi_deinit();
    log_i("WiFi stopped");
  } else {
    log_i("FAILED to stop wifi");
  }

  if(esp_wifi_deinit() == ESP_OK){                            // Stop Wifi
    log_i("WiFi de-initiated");
  } else {
    log_i("FAILED to de-initiate wifi");
  }
  
  
  log_i("Going to sleep now");
  esp_deep_sleep_start();
  //Serial.println("This will never be printed");

}

/*+--------------------------------------------------------------------------------------+
 *| Connect to WiFi network                                                              |
 *+--------------------------------------------------------------------------------------+ */

void setup_wifi() {

  log_i("Connecting to %s",ssid);
    WiFi.mode(WIFI_STA);                                     // Setup ESP in client mode
    
    WiFi.begin(ssid, password);                              // Connect to network

    int wait_passes = 0;
    while (WiFi.status() != WL_CONNECTED) {                  // Wait for connection
      delay(500);
      log_i(".");
      if (++wait_passes >= 20) { ESP.restart(); }            // Restart in case of no wifi connection   
    }

  log_i("WiFi connected");
  

}

 
/*+--------------------------------------------------------------------------------------+
 *| Reconnect to MQTT client                                                             |
 *+--------------------------------------------------------------------------------------+ */
 
void reconnect() {
  
  while (!client.connected()) {                               // Loop until we're reconnected 
    log_i("Attempting MQTT connection...");
    if (client.connect(ID)) {
      log_i("MQTT broker connected");
      log_i("Publishing to: %s",TOPIC);
    } else {
      log_i(" try again in 5 seconds");
      delay(5000);
      setup_wifi();
    }
  }
}

/*+--------------------------------------------------------------------------------------+
 *| Get Date & Time                                                                      |
 *+--------------------------------------------------------------------------------------+ */
 
String DateAndTime(){

    timeClient.setTimeOffset(-10800);                       // Set offset time in seconds to adjust for your timezone, for example:
                                                            // GMT +1 = 3600
                                                            // GMT +8 = 28800
                                                            // GMT -1 = -3600
                                                            // GMT 0 = 0
    while(!timeClient.update()) {
      timeClient.forceUpdate();
    }

  time_t epochTime = timeClient.getEpochTime();              // The time_t type is just an integer. 
                                                             // It is the number of seconds since the Epoch.
  struct tm * tm = localtime(&epochTime);
  char dts[22];
    strftime(dts, sizeof(dts), "%d%b%Y %H-%M-%S", tm);       // https://www.cplusplus.com/reference/ctime/strftime/
  
  return dts;
 
}


/*+--------------------------------------------------------------------------------------+
 *| Measurement counter update                                                           |
 *+--------------------------------------------------------------------------------------+ */
 
 void meas_cnt_update() {

  EEPROM.get(8,meas_counter);                               // Recover counter from EEPROM
    log_i("Current counter : %i",meas_counter);
   
      meas_counter++;                                                    
        EEPROM.put(8,meas_counter);                                        
                                                                        
        EEPROM.commit();                                                                                                                                        

  EEPROM.get(8,meas_counter);                               // Recover new  counter 
    log_i("New counter : %i",meas_counter);

 }


/*+--------------------------------------------------------------------------------------+
 *| Battery voltage measurement                                                          |
 *+--------------------------------------------------------------------------------------+ */
 
 float batt_level() {

  batt_level_volts = (3.3 / 4096) * analogRead(batt_level_pin) * RatioFactor;
    log_i("Battery voltage level : %f", batt_level_volts);
 
  return batt_level_volts;

 }

 /*+--------------------------------------------------------------------------------------+
 *| Measurement counter read                                                           |
 *+--------------------------------------------------------------------------------------+ */
 
 int meas_cnt_read() {

  int meas_counter_now;
  EEPROM.get(8,meas_counter_now);                               // Recover counter from EEPROM
    log_i("Current counter : %i",meas_counter_now);
 
  return meas_counter_now;
 }

/*+--------------------------------------------------------------------------------------+
 *| Serialize JSON and publish MQTT                                                      |
 *+--------------------------------------------------------------------------------------+ */

void SerializeAndPublish() {

  if (!client.connected())                            // Reconnect if connection to MQTT is lost 
  {    reconnect();      }

  client.loop();                                      // MQTT 

  char buffer[256];                                   // JSON serialization 
  
    StaticJsonDocument<256> doc;                      // See ArduinoJson Assistant V6 
    
      doc["Device"] = "Narceja";
      doc["Version"] = swversion;
      doc["RSSI (db)"] = WiFi.RSSI();
      doc["IP"] = WiFi.localIP();
      doc["LastRoll"] = DateAndTime();
      doc["UpTime (boots)"] = bootCount;
      doc["WakeUpReason"] = wakeup_msg;
      doc["MeasCounter"] = meas_cnt_read();
      doc["BattVoltLvl"] = batt_level();
    
    serializeJson(doc, buffer);
      log_i("\nJSON Payload:");
      Serial.printf("\n");
    serializeJsonPretty(doc, Serial);                 // Print JSON payload on Serial port        
      Serial.printf("\n");                  
      log_i("Sending message to MQTT topic");
    client.publish(TOPIC, buffer);                    // Publish data to MQTT Broker 

}

/*+--------------------------------------------------------------------------------------+
 *| Dummy callback method                                                                |
 *+--------------------------------------------------------------------------------------+ */

void callback() {
  //placeholder callback function
}

/*+--------------------------------------------------------------------------------------+
 *| Setup                                                                                |
 *+--------------------------------------------------------------------------------------+ */

void setup() {
  
  pinMode(LED_BOARD, OUTPUT);                                 // initialize digital pin LED_BUILTIN as an output.

  Serial.begin(115200);                                       // Start serial communication at 115200 baud 
    delay(1000); 
  
  EEPROM.begin(512);
  
  swversion = (swversion.substring((swversion.indexOf(".")), (swversion.lastIndexOf("\\")) + 1))+" "+__DATE__+" "+__TIME__;   
   log_i("SW version: %s", swversion);

     
  bootCount++;                                                //Increment boot number and print it every reboot
    log_i("Boot number: %i", bootCount);

  /*
  // Initial setup
   log_i("Initial setup Started...");
  int initial_value = 1192482;
  EEPROM.put(8,initial_value);                               // Only for initial counter save  
    log_i("Save initial value...");                                                                                                      
    EEPROM.commit();
      log_i("Commit...");
  EEPROM.get(8,initial_value);                               
    log_i("Initial meas value : %i",initial_value);    
  */


  print_wakeup_reason();                                      //Print the wakeup reason for ESP32

  touchAttachInterrupt(T0, callback, Threshold);              // Setup interrupt on Touch Pad 0 (GPIO4)
  esp_sleep_enable_touchpad_wakeup();                         // Configure Touchpad as wakeup source

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);                // Configure GPIO as wakeup source
                                                              // 1 = High, 0 = Low

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000);     // Configure timer as wakeup source
    log_i("Setup sleep for every %i Seconds", TIME_TO_SLEEP);

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    log_i("Configured all RTC Peripherals to be powered down in sleep");

  if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0){
  //if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TOUCHPAD){  // only for testing purpose
    digitalWrite(LED_BOARD, HIGH);
      meas_cnt_update();
    delay(500);
    digitalWrite(LED_BOARD, LOW);
  }  

  setup_wifi();

  log_i("Broker MQTT setting server... ");
    client.setServer(BROKER_MQTT, 1883);                      // MQTT port, unsecure

  log_i("Starting timeclient server... "); 	
    timeClient.begin();                                       // Initialize a NTPClient to get time 

  meas_cnt_read();

  SerializeAndPublish();

  prepareAndgoDeepSleep();
  
  
}


/*+--------------------------------------------------------------------------------------+
 *| Main                                                                                 |
 *+--------------------------------------------------------------------------------------+ */

// the loop function runs over and over again forever
void loop() {

// nothing here
  
}
