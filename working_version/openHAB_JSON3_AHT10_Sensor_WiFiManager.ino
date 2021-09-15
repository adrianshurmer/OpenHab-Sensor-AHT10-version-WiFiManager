/*
 * Version 4 - Sensor app for MQTT communication with OpenHab on running on Raspberry Pi
 * Version 4 adds WiFi management to previous version.
 * WiFi management enables the sensor to start in Access Point mode to allow connection to a local
 * WiFi network and save the credentials to the EEPROM for retrieval each time the sensor comes our of 
 * deep sleep
 * This program does not use the void loop() feature as everything is done in the setup routine and then it's put into  
 * deep sleep from where it wakes after a pre-defined period to do the whole thing again.
 */

// Import required Libaries
#include "Arduino.h" // used by deep sleep
#include <ESP8266WiFi.h>
#include <PubSubClient.h> // used by MQTT
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> // used by WiFiManager

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <ArduinoJson.h>

ADC_MODE(ADC_VCC) // setup availability to read votage from ADC pin

// Define some constants
const int MEASUREMENT_PERIOD = 1000000*10; // Time period between measurements in seconds

// WiFi Broker Constants
// const char* NETWORK_SSID =
// const char* NETWORK_PASSWORD = 

// MQTT Broker Constants
const char* MQTT_SERVER = "192.168.0.99"; // MQTT Broker IP Address
const char* MQTT_PORT = "1833"; // MQTT Port Number
const char* MQTT_CLIENT = "MQTT client name here"; // This name needs to be unique
const char* PUBLISH_TOPIC_1 = "test/sensor1"; // Topic to Publish Data to

// GPIO Pin Constants
const int ONBOARD_LED_PIN = 13; // just for testing the onboard LED during commands sent to sensor board
const int SENSOR_PWR_PIN = 15; // powers the sensor chip (GPIO15 - D8)

int BOARD_RESET=12;

// Create Objects
Adafruit_AHTX0 aht; // Create the Adafruit AHT10 object

WiFiClient ESP_CLIENT;
PubSubClient client(ESP_CLIENT);


/*
-------------------------------------
Setup stuff below
-------------------------------------
*/

void setup() {

  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the built in LED as an output to indicate resetting procedure has started
  pinMode(BOARD_RESET, INPUT); // Initialize pin to receive comand to reset board
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  pinMode(SENSOR_PWR_PIN, OUTPUT);
 
  // Set outputs to HIGH (off)
  digitalWrite(LED_BUILTIN, HIGH);
  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // If board is being reset flash the onboard LED at 2s intervals
  if (digitalRead(BOARD_RESET)==HIGH){
    
    Serial.println("Reset button pressed");
    delay(500);
    int x = 0;
    while(x < 10){
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    x++;
    }
    

    wifiManager.resetSettings(); // Initiate the reset routine
    Serial.println("");
    Serial.println("Resetting Credentials");
    delay(500);
  }

  // Uncomment to set custom ip for portal
  // wifiManager.setAPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  /* fetches ssid and pass from eeprom and tries to connect
   * if it does not connect it starts an access point with the specified name
   * here  "AutoConnectAP"
   * and goes into a blocking loop awaiting configuration*/
  wifiManager.autoConnect("AutoConnectAP");
  /* or use this for auto generated name ESP + ChipID*/
  // wifiManager.autoConnect();
  
  // if you get here you have connected to the WiFi
  Serial.println("Connected.");
  
  // Call the function to connect to the MQTT Server
  delay(500);
  connect_to_MQTT();

  // Call the function to set up the AHT10 Sensor
  digitalWrite(SENSOR_PWR_PIN, HIGH);
  setupAHTSensor();

  // If the MQTT Client can not connect to the Server then report this in serial monitor and ry to reconnect
  if (! client.connected()) {
    Serial.print("[");           // Echo status of connection to MQTT server
    Serial.print(MQTT_CLIENT); 
    Serial.println("] Not connected to MQTT Server..... Trying to reconnect..");
    connect_to_MQTT();  // Try to reconnect to the MQTT Server
  }


  // Call the createJSON function to parse data into the JSON format
  createJSON();

  client.loop();
  
  Serial.println("I'm awake, but I'm going into deep sleep mode for 15 mins");
  digitalWrite(SENSOR_PWR_PIN, LOW);
  ESP.deepSleep(MEASUREMENT_PERIOD);

}

/*
------------------------------------------------
 * Define all the finctions below this line *
------------------------------------------------
*/

/*
------------------------------------------------------------------------------------------------------------------
 * Function to connect to the MQTT Server and publish/subscribe to any topics defined at top of this application *
 -----------------------------------------------------------------------------------------------------------------
 */
void connect_to_MQTT() {

  client.setServer(MQTT_SERVER, 1883);  // Set up according to the MQTT server details defined at top and to port 1833
  

  if (client.connect(MQTT_CLIENT)) {

    Serial.print("[");           // Echo that we are connected to the server
    Serial.print(MQTT_CLIENT); 
    Serial.println("] Connected to MQTT Server!");
    
    //client.subscribe(SUBSCRIBE_TOPIC_1);  // THE TOPICS WE ARE SUBSCRIBING TO - MORE CAN BE ADDED AS REQUIRED
    
     }
     else {

    Serial.print("[");           // Echo that we don't have a connection
    Serial.print(MQTT_CLIENT); 
    Serial.println("] Could not connect to MQTT Server!");
    
  }


}

/*
--------------------------------------------------------------------------------
* Function to setup AHT10 sensor or report that a connection could not be made *
--------------------------------------------------------------------------------
*/
void setupAHTSensor(){
   // SETUP THE AHT10 SENSOR
  if (! aht.begin()) {
    Serial.println("Could not find AHT10 Sensor! Check wiring");
    while (1) delay(10);
  }
}

/*
 -------------------------------------------------------------------
 * Function to create the JSon file to parse data into JSON format *
--------------------------------------------------------------------
 */
void createJSON(){

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

  StaticJsonBuffer<300> JSONbuffer;
  JsonObject& JSONencoder = JSONbuffer.createObject();
 
  JSONencoder["test"] = "sensor2";
  JsonArray& temps = JSONencoder.createNestedArray("temperature");
      temps.add(temp.temperature);
    
  JsonArray& hums = JSONencoder.createNestedArray("humidity");
       hums.add(humidity.relative_humidity);
    
  JsonArray& rssi = JSONencoder.createNestedArray("rssi");
       rssi.add(getConnectionData());

  JsonArray& volts = JSONencoder.createNestedArray("voltage");
       volts.add(getVoltage());

  char JSONmessageBuffer[300];
  JSONencoder.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
 
  client.publish(PUBLISH_TOPIC_1,JSONmessageBuffer);
 
  Serial.println(JSONmessageBuffer);
}

/*
-----------------------------------------
 * Funtion to measure voltage on A0 pin *
-----------------------------------------
*/
float getVoltage(){
  // read the input on analog pin 0:
  float voltage = float voltage = (ESP.getVcc()/1000.00);
    // print out the value you read:
  Serial.println(voltage);
  return voltage;
  }

/* 
-------------------------------------------------------  
 * Function to get signal strength of WiFi connection *
-------------------------------------------------------
*/
float getConnectionData(){
  
  float rssi = (WiFi.RSSI());
  return rssi; 
  
}



void loop() {
  // put your main code here, to run repeatedly:

}
