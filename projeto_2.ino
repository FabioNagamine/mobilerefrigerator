/*------header------

    SENAI Technology College "Mariano Ferraz"
    Sao Paulo, 03/24/2020
    Postgraduate - Internet of Things
    Sensors and Identification and Tracking Systems

    Names of postgraduate students: 
    Lecturer: Fábio Takashi Nagamine
              Felipe Thadeu Noro Affonso
              Leonardo Tarrou

    Goals: use an ESP8266 with a DHT11 and a GPS modules to monitoring the temperature and geolocation 
    of a pharmaceutical products

    Hardware: ESP8266 D1 mini, DHT11 module, GPS module and Relay module


    Reviews: 
    R0 - begin

*/


#include "ThingSpeak.h"                                 //ThingSpeak Library
#include <ESP8266WiFi.h>                                //ESP8266WiFi Library
#include "DHTesp.h"                                     //DHTesp Library         
#include <TinyGPS++.h>                                  // Tiny GPS Plus Library
#include <SoftwareSerial.h>                             // Software Serial Library so we can use other Pins for communication with the GPS module

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif


char ssid[] = "teste"; // your network SSID (name) 
char pass[] = "realtime"; // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
bool Geladeira;
static const int RXPin = 12, TXPin = 13;                // Ublox 6m GPS module to pins 12 and 13
static const uint32_t GPSBaud = 9600;                   // Ublox GPS default Baud Rate is 9600
unsigned long myChannelNumber = 897554;                 //Number of the channel on ThingSpeak API
const char * myWriteAPIKey = "HZIIZBXADOVL8QTO";        //ThingSpeak API key
int rele=14, DHTpin=5;                     //rele and DHT pins                        
              
WiFiClient  client;
DHTesp dht;
TinyGPSPlus gps;                                        // Create an Instance of the TinyGPS++ object called gps
SoftwareSerial ss(RXPin, TXPin);                        // The serial connection to the GPS device


void setup() {
 
  Serial.begin(115200);  // Initialize serial
  WiFi.mode(WIFI_STA); 
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard); //Print model


  dht.setup(DHTpin, DHTesp::DHT11); // Connect DHT sensor to GPIO 13
  pinMode(rele, OUTPUT);
  Serial.println(TinyGPSPlus::libraryVersion());
  ss.begin(GPSBaud);
  
}

void loop() {
  
  //Initialize the DHT11 module and its variables then print in serial
  delay(dht.getMinimumSamplingPeriod());
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.println(temperature, 1);

  //Control the refrigerator
  if(temperature<2.0){
    Geladeira=1;
    digitalWrite(rele,HIGH);
  }else{
    Geladeira=0;
    digitalWrite(rele,LOW);
  }

  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }
  //Get position from Satelite
  float latitude= gps.location.lat();
  float longitude= gps.location.lng();  
  delay(2000);

  // set the fields with the values
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);
  ThingSpeak.setField(3, Geladeira);
  ThingSpeak.setField(4, latitude);
  ThingSpeak.setField(5, longitude);
 
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }  
 

  // change the values
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  
  smartDelay(500);                                      // Run Procedure smartDelay

  if (millis() > 5000 && gps.charsProcessed() < 10){
    Serial.println(F("No GPS data received: check wiring"));
  }    
 ESP.deepSleep(1 * 60000000);// Put the esp8266 in deep sleep mode for 1 minute to save battery


  
}

  static void smartDelay(unsigned long ms)                // This custom version of delay() ensures that the gps object is being "fed".
  {
    unsigned long start = millis();
    do 
    {
      while (ss.available())
        gps.encode(ss.read());
    } while (millis() - start < ms);
  }
