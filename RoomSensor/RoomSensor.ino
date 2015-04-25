#include <SPI.h>      // Required to use Ethernet
#include <Ethernet.h> // The Ethernet library includes the client
#include <Progmem.h>  // Allows us to sacrifice flash for DRAM
#include "DHT.h"      // Required to use the DTH22 sensor
#include<stdlib.h>    // Required for the dtostrf (float to string) method
#include <OneWire.h>  
#include <DallasTemperature.h>
#include <Wire.h>    //BH1750 IIC Mode 
#include <math.h>
#include "keys.h"

///////////////////////
// Ethernet Settings //
///////////////////////
// Enter a MAC address for your controller below.
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(54,86,132,254);  // numeric IP for data.sparkfun.com
char server[] = "data.sparkfun.com";    // name address for data.sparkfun (using DNS)
// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192,168,0,177);

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 80 is default for HTTP):
EthernetClient client;

/////////////////
// Phant Stuff //
/////////////////
const String publicKey = phantPublicKey;
const String privateKey = phantPrivateKey;
const byte NUM_FIELDS = 8;
const String fieldNames[NUM_FIELDS] = {"temperature", "humidity", "noise", "ldr", "dust", "radiatortemp", "ambientlight", "barometer"};
String fieldData[NUM_FIELDS];

//////////////////////
// Input Pins, Misc //
//////////////////////
const int noisePin = A0;
const int lightPin = A2;
const int dustMeasurePin = A3;
const int dustLightPin = 3;
const int indicatorLightPin = 7;
String name = "Ether-anon";
boolean newName = true;
boolean indicatorOn = true;

//////////////////////
// DS18B20          //
//////////////////////
// Data wire is plugged into pin 5 on the Arduino (changed from 2 in example)
#define ONE_WIRE_BUS 5
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//////////////////////
// DTH22 Temp & Hum //
//////////////////////
#define DHTPIN 2
#define DHTTYPE DHT22   // DHT 22  (AM2302)
// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);
float voMeasured;
float calcVoltage;
float dustDensity;

//////////////////////
// BH1750           //
//////////////////////
/*
  SCL(analog pin 5)
  SDA(analog pin 4)
 */
int BH1750address = 0x23; //setting i2c address
byte buff[2];

//////////////////////
// Timer            //
//////////////////////
long previousMillis = 0;
long interval = 60000; // interval in milliseconds 60000

void setup()
{
  Serial.begin(115200);

  // Setup Input Pins:
  pinMode(noisePin, INPUT_PULLUP);
  pinMode(lightPin, INPUT_PULLUP);
  pinMode(dustLightPin, OUTPUT);
  pinMode(indicatorLightPin, OUTPUT);

  // Set up Ethernet:
  setupEthernet();
  
  // Set up DHTH
  dht.begin();
  
  // Set up BH1750
  Wire.begin();
  BH1750_Init(BH1750address);
  
  // Set up the DallasTemperature library
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
}

void loop()
{
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    postSensorData();
  }
}

void postSensorData() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  
  char tmp[10];
  dtostrf(t, 1, 2, tmp);
  fieldData[0] = tmp;
  
  dtostrf(h, 1, 2, tmp);
  fieldData[1] = tmp;
  
  //dtostrf(dustDensity, 1, 2, tmp);
  dtostrf(readDustSensor(), 1, 2, tmp);
  fieldData[4] = tmp;
  
  fieldData[2] = String(analogRead(noisePin));
  fieldData[3] = String(analogRead(lightPin));
  
  // Read the DS18B20
  sensors.requestTemperatures(); // Send the command to get temperatures
  fieldData[5] = String(sensors.getTempCByIndex(0));
  
  // Read the BH1750
  uint16_t val=0;
  if(2==BH1750_Read(BH1750address))
  {
    val=((buff[0]<<8)|buff[1])/1.2;
    fieldData[6] = String(val);
  }

  postData();
}

float readDustSensor() {
  digitalWrite(dustLightPin,LOW); // power on the LED
  delayMicroseconds(280);
  voMeasured = analogRead(dustMeasurePin); // read the dust value
  delayMicroseconds(40);
  digitalWrite(dustLightPin,HIGH); // turn the LED off
  delayMicroseconds(9680);
 
  // 0 - 5V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured * (5.0 / 1024.0);
  
  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = 0.17 * calcVoltage - 0.1;
  /*
  Serial.print("Raw Signal Value (0-1023): ");
  Serial.print(voMeasured);
  
  Serial.print(" - Voltage: ");
  Serial.print(calcVoltage);
  
  Serial.print(" - Dust Density: ");
  Serial.print(dustDensity * 1000); // 這裡將數值呈現改成較常用的單位( ug/m3 )
  Serial.println(" ug/m3 ");
  */
  return dustDensity * 1000;
}

float BH1750_Read(int address) //
{
  float i = -1;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()) //
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission();  
  return i;
}
 
void BH1750_Init(int address) 
{
  Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();
}

void postData()
{
  // Make a TCP connection to remote host
  if (client.connect(server, 80))
  {
    // Post the data! Request should look a little something like:
    // GET /input/publicKey?private_key=privateKey&light=1024&switch=0&name=Jim HTTP/1.1\n
    // Host: data.sparkfun.com\n
    // Connection: close\n
    // \n
    client.print("GET /input/");
    client.print(publicKey);
    client.print("?private_key=");
    client.print(privateKey);
    for (int i=0; i<NUM_FIELDS; i++)
    {
      client.print("&");
      client.print(fieldNames[i]);
      client.print("=");
      client.print(fieldData[i]);
    }
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(server);
    client.println("Connection: close");
    client.println();
  }
  else
  {
    Serial.println(F("Connection failed"));
  } 

  // Check for a response from the server, and route it
  // out the serial port.
  while (client.connected())
  {
    if ( client.available() )
    {
      char c = client.read();
      //Serial.print(c);
    }      
  }
  //Serial.println();
  client.stop();
}

void setupEthernet()
{
  Serial.println("Setting up Ethernet...");
  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed to configure Ethernet using DHCP"));
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  }
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  // give the Ethernet shield a second to initialize:
  delay(1000);
}

