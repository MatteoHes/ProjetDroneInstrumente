/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-client-server-wi-fi/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

//Bibliotheque de gestion de la connection Wifi
#include "WiFi.h"
//Bibliotheque de gestion du serveur
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

const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";
int test=0,i=0;
String temperatureHes,humidityHes,latitudeHes,longitudeHes, altitudeHes, volumeHes, vitesseHes, tempsHes,lumHes;
Adafruit_BMP280 Altimeter;
void  course_and_speed(void);
float convert_latitude_longitude(String to_convert);

//Création du serveur Web Asynchrone sur le port 80
AsyncWebServer server(80);

//Convertissage des String en fonction String
//pour la fonction server.on
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
String volumeHesString() {
  return String(volumeHes);
}
String vitesseHesString() {
  return String(vitesseHes);
}
String tempsHesString() {
  return String(tempsHes);
}
String lumHesString() {
  return String(lumHes);
}

void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
  
  Serial.println("test");
    SerialPort.begin(9600, SERIAL_8N1, RXD0, TXD0);
 
  Altimeter.begin();
    dht.begin();

  Serial.print("Setting AP (Access Point)…");

//On règle l'ESP32 en mode Access Point
//On lui donne un identifiant et un mot de passe
  WiFi.softAP(ssid, password);

//On récupère l'adresse IP de la carte
  IPAddress IP = WiFi.softAPIP();
  Serial.println(IP);

  Serial.print("AP IP address: ");
  
  //Préparation des réponses aux requêtes HTTP_GET
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
            server.on("/vitesse", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain",vitesseHesString().c_str());
  });
              server.on("/luminosity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain",lumHesString().c_str());
  });
}

//Fonction récupérant les données de l'altimètre
float altitude (){
  Serial.print("Atitude barométrique :");
  float alt;
  alt = Altimeter.readAltitude(1028);
  Serial.print(alt);
  return alt;
}

//Fonction récupérant les données de la température
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

//Fonction récupérant les coordonnées GPS
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
    tempsHes = gps_time;

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
  }
}

//Fonction récupérant le volume sonore
int volume (){
  int val;

//On échantillone 50 valeurs en une seconde, et on prend la plus élevée
  for (int i = 0; i < 50; i++)
  {
    if(val<analogRead(32))val = analogRead(32);
    delay(20);
  }
  return val;
}

//Fonction convertissant les données GPS en nombres utilisables
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

//Fonction récupérant les données de la vitesse grâce au GPS
void course_and_speed(){
  String gps_data;
  String gps_time;
  String speed;
  String course;
  
  while(gps_data.startsWith("$GPRMC") == false){
    gps_data = SerialPort.readStringUntil(13);
    gps_data.trim();
  }

  int Pos;
  for (int i = 0; i < 7; i++)
  {
    Pos = gps_data.indexOf(',');
    gps_data.remove(0,Pos+1);
  }
  Pos = gps_data.indexOf(',');
  speed = gps_data;
  speed.remove(Pos, 80);
  gps_data.remove(0,Pos+1);
  Pos = gps_data.indexOf(',');
  course = gps_data;
  course.remove(Pos, 80);

  char* chr_speed = new char[speed.length()];
  strcpy(chr_speed, speed.c_str());
  //conversion de tableau de char en float
  float speed_kph = atof(chr_speed);
  speed_kph = speed_kph *1.852;
  vitesseHes=speed_kph;
  Serial.print("Vitesse: ");
  Serial.print(speed_kph,2);
  Serial.print(" km/h\n");
  Serial.print("Direction:" + course + "°\n");
}

//Fonction récupérant la luminosité
int lumiere(void)
{
  int lumiere=0;
  float sensor;
  lumiere=analogRead(35);
  lumiere = map(lumiere,0,2500,0,100);
  return lumiere;
}


void loop(){
//Appel des fonctions
temperature_humidity_read();
lumHes=lumiere();

gps_read();
course_and_speed();
altitudeHes=altitude();

//Le delay est dans la fonction volume
//pour avoir un échantillonage rapide du son
volumeHes=volume();
}

