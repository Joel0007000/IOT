#include <ESP8266WiFi.h>
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macro
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define ON_Board_LED 2  //--> Defining an On Board LED (GPIO2 = D4), used for indicators when the process of connecting to a wifi router and when there is a client request.

#define DHTPIN 13  
// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

#define WIFI_SSID "nepal123"
#define WIFI_PASSWORD "nepal123"


unsigned long myChannelNumber = 1312630;
const char * myWriteAPIKey = "XEOV7D7MJ3YB68DC";

WiFiClient client;


unsigned long previousMillisGetHR = 0; //--> will store the last time Millis (to get Heartbeat) was updated.
unsigned long previousMillisHR = 0; //--> will store the last time Millis (to get BPM) was updated.

const long intervalGetHR = 10; //--> Interval for reading heart rate (Heartbeat) = 10ms.
const long intervalHR = 10000; //--> Interval for obtaining the BPM value based on the sample is 10 seconds.

const int PulseSensorHRWire = A0; //--> PulseSensor connected to ANALOG PIN 0 (A0 / ADC 0).
const int LED_D1 = D1; //--> LED to detect when the heart is beating. The LED is connected to PIN D1 (GPIO5) on the NodeMCU ESP12E.
int Threshold = 540; //--> Determine which Signal to "count as a beat" and which to ignore.

int cntHB = 0; //--> Variable for counting the number of heartbeats.
boolean ThresholdStat = true; //--> Variable for triggers in calculating heartbeats.
int BPMval = 0; //--> Variable to hold the result of heartbeats calculation.




void GetHeartRate() {
  //----------------------------------------Process of reading heart rate.
  unsigned long currentMillisGetHR = millis();

  if (currentMillisGetHR - previousMillisGetHR >= intervalGetHR) {
    previousMillisGetHR = currentMillisGetHR;

    int PulseSensorHRVal = analogRead(PulseSensorHRWire);
    Serial.println("pulseval  ");
    Serial.println(PulseSensorHRVal);

    if (PulseSensorHRVal > Threshold && ThresholdStat == true) {
      cntHB++;
      ThresholdStat = false;
      digitalWrite(LED_D1,HIGH);
    }

    if (PulseSensorHRVal < Threshold) {
      ThresholdStat = true;
      digitalWrite(LED_D1,LOW);
    }
  }
  //----------------------------------------

  //----------------------------------------The process for getting the BPM value.
  unsigned long currentMillisHR = millis();

  if (currentMillisHR - previousMillisHR >= intervalHR) {
    previousMillisHR = currentMillisHR;

    BPMval = cntHB * 6; //--> The taken heart rate is for 10 seconds. So to get the BPM value, the total heart rate in 10 seconds x 6.
    Serial.print("BPM : ");
    Serial.println(BPMval);

    sensors_event_t event;
  dht.temperature().getEvent(&event);

   ThingSpeak.setField(1, BPMval);
   ThingSpeak.setField(2, event.temperature);
   Serial.print(event.temperature);
     int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
    
    cntHB = 0;
  }
  //----------------------------------------
}
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------void setup()
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // we agree to talk fast!
 


WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  delay(500);
    
  pinMode(ON_Board_LED,OUTPUT); //--> On Board LED port Direction output
  digitalWrite(ON_Board_LED, HIGH); //--> Turn off Led On Board

  pinMode(LED_D1,OUTPUT); //--> Set LED_3 PIN as Output.

    // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  
   ThingSpeak.begin(client);

}
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------void loop()
void loop() {
  GetHeartRate(); //--> Calling the GetHeartRate() subroutine
}
