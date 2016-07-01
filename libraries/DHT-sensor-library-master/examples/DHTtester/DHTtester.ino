// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

#include "DHT.h"

#define DHTPIN 8     // what pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("DHTxx test!");

  setupCommandParser();

  dht.begin();
}

void loop() {
  
  //parse incoming serial data
  parseIncomingSerialData();

  // Wait a few seconds between measurements.
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);

  Serial.print("{ \"humidity\": \"");
  Serial.print(h);
  Serial.print("%\"");
  Serial.print(",\"temperature\": \"");
  Serial.print(f);
  Serial.print("*F\",\"");
  Serial.print(",\"heat_index\": \"");
  Serial.print(hif);
  Serial.println("*F\"}");
}






String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void parseIncomingSerialData()
{
  // print the string when a newline arrives:
  if (stringComplete) {
    Serial.println("recieved:" + inputString); 
    processCommand( inputString );
    // clear the string:
    inputString = "";
    stringComplete = false;
  } 
}
void processCommand( String cmd )
{
  Serial.println("processing command: "+ cmd ); 

  if( cmd == "reset\n" )
  {
    Serial.println("RESETTING...");
    setup(); 
  } 
  else if( cmd == "demo\n" )
  {
    Serial.println("DEMO MODE ACTIVATED"); 
  }
}
void setupCommandParser()
{
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);  
}
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}
