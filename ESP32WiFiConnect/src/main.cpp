/*#include <WiFi.h>
#include <WiFiClient.h>
#include <Arduino.h>
const char* ssid = "portablematteo";
const char* password = "1234567890";
const char* serverIP = "172.18.79.251";
const int serverPort = 8080;

WiFiClient client;

void setup() {
  Serial.begin(921600);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  Serial.print("Connecting to server: ");
  Serial.print(serverIP);
  Serial.print(":");
  Serial.println(serverPort);

  if (client.connect(serverIP, serverPort)) {
    Serial.println("Connected to server");
    client.println("Hello from ESP32!");
    client.println(); // Send an empty line to indicate the end of the request
  }
  else {
    Serial.println("Connection to server failed");
  }
}

void loop() {
  if (client.available()) {
    String response = client.readStringUntil('\n');
    Serial.println(response);
  }

  if (!client.connected()) {
    Serial.println("Server disconnected");
    client.stop();
    while (true) {
      // Handle reconnection or other actions
    }
  }
}
*/
/*
#include <Arduino.h>
#include <WiFiMulti.h>

#define WIFI_SSID "portablematteo"
#define WIFI_PASSWORD "1234567890"
// put function declarations here:

WiFiMulti wifiMulti;

void setup() {
  // put your setup code here, to run once:
Serial.begin(921600);

wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
while(wifiMulti.run() !=WL_CONNECTED)
{
  delay(100);
}
Serial.println("Connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}

void loop() {

  // put your main code here, to run repeatedly:
}*/

/*
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#define WIFI_SSID "portablematteo"
#define WIFI_PASSWORD "1234567890"
const char* serverIP = "127.0.0.1";
const int serverPort = 8080;

WiFiClient client;


void setup() {
Serial.begin(921600);
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
Serial.println("Connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
    Serial.print("Connecting to server: ");
  Serial.print(serverIP);
  Serial.print(":");
  Serial.println(serverPort);
  
  if (client.connect(serverIP, serverPort)) {
    Serial.println("Connected to server");
    client.println("Hello from ESP32!");
    client.println(); // Send an empty line to indicate the end of the request
  }
  else {
    Serial.println("Connection to server failed");
  }
}

void loop() {
  if (client.available()) {
    String response = client.readStringUntil('\n');
    Serial.println(response);
  }

  if (!client.connected()) {
    Serial.println("Server disconnected");
    client.stop();
    while (true) {
    }
  }
}*/
/*
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

const char* ssid = "portablematteo";
const char* password = "1234567890";
WiFiServer server(80);

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  server.begin();
  Serial.println("Server started");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("New client connected");
    
    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\r');
        Serial.println(request);
        client.println("Hello from ESP32!");
        delay(10);
        client.stop();
        Serial.println("Client disconnected");
      }
    }
  }
}
*/

/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-client-server-wi-fi/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/


// Import required libraries
#include "WiFi.h"
#include "ESPAsyncWebServer.h"

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <iostream>
#include <string>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//lib pour l'altitude
#include <SPI.h>
#include <Adafruit_BMP280.h>



#define RXD0 16
#define TXD0 17

#define DHTPIN 33
#define DHTTYPE DHT11

HardwareSerial SerialPort(2);
DHT_Unified dht(DHTPIN, DHTTYPE);



// Set your access point network credentials
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";
int test=0,test2=100,test3=200, test4=300,test5=400;
String temperatureHes,humidityHes,latitudeHes,longitudeHes, altitudeHes, volumeHes;
Adafruit_BMP280 Altimeter;

//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String tester() {
  return String(test);
  //return String(1.8 * bme.readTemperature() + 32);
}
String altitudeHesString() {
  return String(altitudeHes);
  //return String(1.8 * bme.readTemperature() + 32);
}
String longitudeHesString() {
  return String(longitudeHes);
  //return String(1.8 * bme.readTemperature() + 32);
}
String latitudeHesString() {
  return String(latitudeHes);
  //return String(1.8 * bme.readTemperature() + 32);
}
String humidityHesString() {
  return String(humidityHes);
  //return String(1.8 * bme.readTemperature() + 32);
}
String temperatureHesString() {
  return String(temperatureHes);
  //return String(1.8 * bme.readTemperature() + 32);
}
String volumeHesString() {
  return String(volumeHes);
  //return String(1.8 * bme.readTemperature() + 32);
}


float convert_latitude_longitude(String to_convert){ 
    //conversion du string en tableau de char  
    char* coordinate = new char[to_convert.length()];
    strcpy(coordinate, to_convert.c_str());
    //conversion de tableau de char en float
    float flt_coordinate = atof(coordinate);

    int firstdigits = ((int)flt_coordinate)/100;
    float lastdigits = flt_coordinate - (float)(firstdigits*100);
    float converted = ((float)(firstdigits + lastdigits/60.00));
    return converted;
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  Serial.println("test");
    SerialPort.begin(9600, SERIAL_8N1, RXD0, TXD0);
 
  Altimeter.begin();

    dht.begin();

  // Setting the ESP as an access point
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();
      server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain",tester().c_str());
  });
      server.on("/altitude", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain",altitudeHesString().c_str());
  });
      server.on("/longitude", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain",longitudeHesString().c_str());
  });
      server.on("/latitude", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain",latitudeHesString().c_str());
  });
        server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain",humidityHesString().c_str());
  });
        server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain",temperatureHesString().c_str());
  });
          server.on("/volume", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain",volumeHesString().c_str());
  });
}

float altitude (){
  Serial.print("Atitude barométrique :");
  float alt;
  alt = Altimeter.readAltitude(1015);
  Serial.print(alt);
  return alt;
}

void temperature_humidity_read(){
  
  uint32_t delayMS;
  sensor_t sensor;
  delayMS = sensor.min_delay / 500;  
  //delay(delayMS);

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    temperatureHes=event.temperature;
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    humidityHes=event.relative_humidity;
  }
}

void gps_read(){
  
  String gps_data;
  String gps_time;
  String Latitude;
  char Latitude_NS;
  String Longitude;
  char Longitude_WE; 
  String Altitude;

  gps_data = SerialPort.readStringUntil(13);
  gps_data.trim();
  if (gps_data.startsWith("$GPGGA")){
    //Serial.print("raw:" + gps_data + "\n");

    int Pos = gps_data.indexOf(',');
    gps_data.remove(0,Pos+1);
    Serial.print("Data:" + gps_data + "\n\n");

    //Heure UTC
    Pos = gps_data.indexOf(',');
    gps_time = gps_data;
    gps_time.remove(Pos, 80);
    gps_data.remove(0, Pos+1);
    Serial.print("Heure UTC: " + gps_time + "\n");

    //Latitude
    Pos = gps_data.indexOf(',');
    Latitude = gps_data;
    Latitude.remove(Pos, 80);
    gps_data.remove(0, Pos+1);
    Serial.print("Latitude: ");
    Serial.println(convert_latitude_longitude(Latitude), 6);
    Serial.print(" ");
    latitudeHes= convert_latitude_longitude(Latitude);

    //Latitude Nord ou Sud
    Pos = gps_data.indexOf(',');
    Latitude_NS = gps_data.charAt(Pos-1);
    gps_data.remove(0, Pos+1);
    Serial.print(Latitude_NS);
    Serial.print("\n");

    //Longitude
    Pos = gps_data.indexOf(',');
    Longitude = gps_data;
    Longitude.remove(Pos, 80);
    gps_data.remove(0, Pos+1);
    Serial.print("Longitude: ");
    Serial.println(convert_latitude_longitude(Longitude), 6);
    Serial.print(" ");
    longitudeHes=convert_latitude_longitude(Longitude);

    //Longitude Ouest ou Est
    Pos = gps_data.indexOf(',');
    Longitude_WE = gps_data.charAt(Pos-1);
    gps_data.remove(0, Pos+1);
    Serial.print(Longitude_WE);
    Serial.print("\n");

    //Altitude 
    Pos = gps_data.indexOf(',');
    gps_data.remove(0, Pos+1);
    Pos = gps_data.indexOf(',');
    gps_data.remove(0, Pos+1);
    Pos = gps_data.indexOf(',');
    gps_data.remove(0, Pos+1);
    Altitude = gps_data;
    Altitude.remove(Pos, 80);
    
    Serial.print("Altitude: " + Altitude + " m");
    Serial.print("\n");
    altitudeHes=Altitude;
  }
}

int volume (){
  int val;
  for (int i = 0; i < 200; i++)
  {
    if(val<analogRead(32))val = analogRead(32);
  }
  return val;
}

void loop(){
test=test+1;
temperature_humidity_read();
gps_read();
altitudeHes=altitude ();

volumeHes=volume();

Serial.print(volumeHes);

delay(1000);

}


/*
void gps_read(){

  
  String gps_data;
  String gps_time;
  String Latitude;
  char Latitude_NS;
  String Longitude;
  char Longitude_WE; 
  String Altitude;

  gps_data = SerialPort.readStringUntil(13);
  gps_data.trim();
  if (gps_data.startsWith("$GPGGA")){
    //Serial.print("raw:" + gps_data + "\n");

    int Pos = gps_data.indexOf(',');
    gps_data.remove(0,Pos+1);
    Serial.print("Data:" + gps_data + "\n\n");

    //Heure UTC
    Pos = gps_data.indexOf(',');
    gps_time = gps_data;
    gps_time.remove(Pos, 80);
    gps_data.remove(0, Pos+1);
    Serial.print("Heure UTC: " + gps_time + "\n");

    //Latitude
    Pos = gps_data.indexOf(',');
    Latitude = gps_data;
    Latitude.remove(Pos, 80);
    gps_data.remove(0, Pos+1);
    Serial.print("Latitude: " + Latitude + " ");
    latitudeHes=Latitude;
    //Latitude Nord ou Sud
    Pos = gps_data.indexOf(',');
    Latitude_NS = gps_data.charAt(Pos-1);
    gps_data.remove(0, Pos+1);
    Serial.print(Latitude_NS);
    Serial.print("\n");

    //Longitude
    Pos = gps_data.indexOf(',');
    Longitude = gps_data;
    Longitude.remove(Pos, 80);
    gps_data.remove(0, Pos+1);
    Serial.print("Longitude: " + Longitude + " ");
    longitudeHes=Longitude;

    //Longitude Ouest ou Est
    Pos = gps_data.indexOf(',');
    Longitude_WE = gps_data.charAt(Pos-1);
    gps_data.remove(0, Pos+1);
    Serial.print(Longitude_WE);
    Serial.print("\n");

    //Altitude 
    Pos = gps_data.indexOf(',');
    gps_data.remove(0, Pos+1);
    Pos = gps_data.indexOf(',');
    gps_data.remove(0, Pos+1);
    Pos = gps_data.indexOf(',');
    gps_data.remove(0, Pos+1);
    Altitude = gps_data;
    Altitude.remove(Pos, 80);
    
    Serial.print("Altitude: " + Altitude + " m");
    Serial.print("\n");
    altitudeHes=Altitude;

    
    //traitement des coordonnées
    Serial.print("\n\n\n");
    //double lat = strto(Latitude, NULL);
    char* c_lat = new char[Latitude.length()];
    strcpy(c_lat, Latitude.c_str());

    float lat_deg = atof(c_lat);
    int firstdigits = ((int)lat_deg)/100;
    float nexttwodigits = lat_deg - (float)(firstdigits*100);
    float converted = ((float)(firstdigits + nexttwodigits/60.00))*1000;
    int lastdigits = (nexttwodigits/60)*100000;
    Serial.print(converted);
    Serial.print("\n");
    Serial.print(lastdigits);
    Serial.print("\n");
    Serial.print(firstdigits);
    Serial.print(".");
    Serial.print(lastdigits);
    Serial.print("\n");
    //Latitude = to_string(firstdigits) + ".";
    Serial.print("la nouvelle latitude est: "+Latitude);
    Serial.print("\n");
  }
}
*/

/*
#include "WiFi.h"
#include "ESPAsyncWebServer.h"

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <iostream>
#include <string>
#include <WiFiAP.h>
//#include <WebServer.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define RXD0 16
#define TXD0 17

#define DHTPIN 2
#define DHTTYPE DHT11

HardwareSerial SerialPort(2);
DHT_Unified dht(DHTPIN, DHTTYPE);

// Set your access point network credentials
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";
int test = 0, test2 = 100, test3 = 200, test4 = 300, test5 = 400;
String temperatureHes, humidityHes, latitudeHes, longitudeHes, altitudeHes;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String tester() {
  return String(test);
}

String altitudeHesString() {
  return String(altitudeHes);
}

String longitudeHesString() {
  return String(longitudeHes);
}

String latitudeHesString() {
  return String(latitudeHes);
}

String humidityHesString() {
  return String(humidityHes);
}

String temperatureHesString() {
  return String(temperatureHes);
}

float convert_latitude_longitude(String to_convert) {
  //conversion du string en tableau de char
  char* coordinate = new char[to_convert.length()];
  strcpy(coordinate, to_convert.c_str());
  //conversion de tableau de char en float
  float flt_coordinate = atof(coordinate);

  int firstdigits = ((int)flt_coordinate) / 100;
  float lastdigits = flt_coordinate - (float)(firstdigits * 100);
  float converted = ((float)(firstdigits + lastdigits / 60.00));
  return converted;
}

void setup() {
  // Serial port for debugging purposes
  Serial.begin(921600);
  Serial.println("test");
  SerialPort.begin(9600, SERIAL_8N1, RXD0, TXD0);
  dht.begin();

  // Connect to Wi-Fi network
  WiFi.begin("pcmatteo", "1234567890");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Print ESP32 IP address after successful connection
  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());

  // Setting the ESP as an access point
  Serial.print("Setting AP (Access Point)...");
  // Remove the password parameter if you want the AP to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin();
  server.on("/test", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/plain", tester().c_str());
  });
  server.on("/altitude", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/plain", altitudeHesString().c_str());
  });
  server.on("/longitude", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/plain", longitudeHesString().c_str());
    });
    server.on("/latitude", HTTP_GET, [](AsyncWebServerRequest* request) {
      request->send_P(200, "text/plain", latitudeHesString().c_str());
    });
    server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest* request) {
      request->send_P(200, "text/plain", humidityHesString().c_str());
    });
    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest* request) {
      request->send_P(200, "text/plain", temperatureHesString().c_str());
    });
}

void temperature_humidity_read() {

  uint32_t delayMS;
  sensor_t sensor;
  delayMS = sensor.min_delay / 500;
  //delay(delayMS);

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    temperatureHes = String(event.temperature);
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    humidityHes = String(event.relative_humidity);
  }
}

void gps_read() {

  String gps_data;
  String gps_time;
  String Latitude;
  char Latitude_NS;
  String Longitude;
  char Longitude_WE;
  String Altitude;

  gps_data = SerialPort.readStringUntil(13);
  gps_data.trim();
  if (gps_data.startsWith("$GPGGA")) {
    //Serial.print("raw:" + gps_data + "\n");

    int Pos = gps_data.indexOf(',');
    gps_data.remove(0, Pos + 1);
    Serial.print("Data:" + gps_data + "\n\n");

    //Heure UTC
    Pos = gps_data.indexOf(',');
    gps_time = gps_data;
    gps_time.remove(Pos, 80);
    gps_data.remove(0, Pos + 1);
    Serial.print("Heure UTC: " + gps_time + "\n");

    //Latitude
    Pos = gps_data.indexOf(',');
    Latitude = gps_data;
    Latitude.remove(Pos, 80);
    gps_data.remove(0, Pos + 1);
    Serial.print("Latitude: ");
    Serial.println(convert_latitude_longitude(Latitude), 6);
    Serial.print(" ");
    latitudeHes = String(convert_latitude_longitude(Latitude));

    //Latitude Nord ou Sud
    Pos = gps_data.indexOf(',');
    Latitude_NS = gps_data.charAt(Pos - 1);
    gps_data.remove(0, Pos + 1);
    Serial.print(Latitude_NS);
    Serial.print("\n");

    //Longitude
    Pos = gps_data.indexOf(',');
    Longitude = gps_data;
    Longitude.remove(Pos, 80);
    gps_data.remove(0, Pos + 1);
    Serial.print("Longitude: ");
    Serial.println(convert_latitude_longitude(Longitude), 6);
    Serial.print(" ");
    longitudeHes = String(convert_latitude_longitude(Longitude));

    //Longitude Ouest ou Est
    Pos = gps_data.indexOf(',');
    Longitude_WE = gps_data.charAt(Pos - 1);
    gps_data.remove(0, Pos + 1);
    Serial.print(Longitude_WE);
    Serial.print("\n");

    //Altitude 
    Pos = gps_data.indexOf(',');
    gps_data.remove(0, Pos + 1);
    Pos = gps_data.indexOf(',');
    Altitude = gps_data;
    Altitude.remove(Pos, 80);
    Serial.print("Altitude: ");
    Serial.println(Altitude);
    Serial.print(" ");
    altitudeHes = String(Altitude);
  }
}

void loop() {
  //temperature_humidity_read();
  //gps_read();
  test+=1;
  
  delay(1000);
}*/


