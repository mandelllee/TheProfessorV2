//      _                 __   _     _  __ _
//     | |               / _| | |   (_)/ _| |
//     | |     ___  __ _| |_  | |    _| |_| |_
//     | |    / _ \/ _` |  _| | |   | |  _| __|
//     | |___|  __/ (_| | |   | |___| | | | |_
//     \_____/\___|\__,_|_|   \_____/_|_|  \__|
//
//     Arduino-Core
//
//          I2C    I2C   1WIRE
//          SDA    SCL   Temp   DHT                           Rx2   Tx2   Rx0   Tx0
//    16  : 5    : 4   : 0    : 2   : 3v  : GND : 14  : 12  : 13  : 15  : 3   : 1   : GND : 3v  :
//  |--------------------------------------------------------------------------------------------|
//  |                                                                                            |
//  |                                                                                            |
//  |__________________                                                                          |
//  |___               |                                                                         |-----
//  |__   |            |                                                                         |     |
//   __|  | ESP8266    |   Node MCU v1.0                                                         | USB |
//  |__|  |            |                                                                         |     |
//   __|__|            |                                                                         |     |
//  |__________________|                                                                         |-----
//  |                                                                                            |
//  |                                                                                            |
//  |                                                                                            |
//  |--------------------------------------------------------------------------------------------|
//    A   : n/a  : n/a : 10  : 9   : MOSI : CS  : MISO : SCLK : GND : 3v : EN : RST : GND : Vin
//                                   SPI    SPI   SPI    SPI
//

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BMP085.h>
#include <OneWire.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_STATUS_REQUEST
#include <TaskScheduler.h>



int oneWirePin = 0;
OneWire  ds(oneWirePin);  //a 2.2K resistor is necessary for 3.3v on the signal line, 4.7k for 5v


Scheduler ts;
Scheduler sensorScheduler;

#include "leaflift_crypto.h"

#include "Adafruit_MCP23017.h"

char API_HOST[] = "api-quadroponic.rhcloud.com";
int API_PORT = 80;
//char API_HOST[] = "10.5.1.25";
//int API_PORT = 3000;

bool SEND_DATA_TO_API = true;
int SEND_DATA_INTERVAL = 10000;

bool _soilSensorEnabled = false;

double temp_c = 0.00;
double ph_value_double = 0.00;

String BOARD_ID = "";

String VERSION = "0.0-skye";

bool TEST_MODE = false;

String chip_id = "";
String _hostname = "";

int i2c_devices[10];
int i2c_device_count = 0;
String currentDisplayText = "";

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
bool _renderDisplayEnabled = true;

bool _enableOTAUpdate = true;
bool _phSensorEnabled = false;
bool _uptime_display = true;
String uptime_string = "";



#include <SoftwareSerial.h>
SoftwareSerial BT(14, 12);
// creates a "virtual" serial port/UART
// connect BT module TX to D10
// connect BT module RX to D11
// connect BT Vcc to 5V, GND to GND

Adafruit_MCP23017 mcp;


#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

const char* ssid     = "gbsx";
const char* password = "OrlandoNakazawa!";
String ipAddressString = "";
ESP8266WebServer server(80);

const char WiFiAPPSK[] = "password";

const char *ap_name      = "Enterprise-D";
const char *ap_passwd    = "1234567890";

void setupWiFi()
{
  //    WiFi.mode(WIFI_AP);
  //    WiFi.softAP(ap_name, ap_passwd);

  WiFi.mode(WIFI_STA);
  connectWiFi();
}

bool promptVisible = false;
int promptTimeout = 0;
String promptTitle;
String promptText;
double promptLength = 0.00;

unsigned long _tickCount = 0;
Scheduler ticker;

Task tTicker( 1000, TASK_FOREVER, &TickCallback, &ticker, true);

void TickCallback() {
  _tickCount++;
  // if we have counted 60 seconds
  if ( _tickCount % 60 == 0 ) {
  }
}

//const uint16_t aport = 8266;
//WiFiServer TelnetServer(aport);
//WiFiClient Telnet;
//
//
//void printDebug(const char* c){
////Prints to telnet if connected
//  Serial.println(c);
//
//  if (Telnet && Telnet.connected()){
//    Telnet.println(c);
//  } //- See more at: http://www.esp8266.com/viewtopic.php?f=8&t=5597#sthash.ITMN1A9x.dpuf
//}

#include "leaflift_api.h"
#include "leaflift_OTA.h"
#include "dht_sensor.h"


void __setupWiFi() {
  //  WiFi.softAP( "Wi-Fi_" + _hostname.c_str(), "secure" );
  WiFi.mode(WIFI_AP);

  //WiFi.mode(WIFI_STA);

  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "ESP8266 Thing " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i = 0; i < AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);

  //connectWiFi();
}

void connectWiFi() {

  Serial.println("Connecting to wiFi ssid: " + String( ssid ) + "...");

  displayTextOnDisplay("Connecting to:\n   " + String(ssid) + "...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    addTextToDisplay( "." );
  }
  saveIP();
}


void saveIP() {

  IPAddress myAddr = WiFi.localIP();
  byte first_octet = myAddr[0];
  byte second_octet = myAddr[1];
  byte third_octet = myAddr[2];
  byte fourth_octet = myAddr[3];

  ipAddressString = String( String(first_octet) + "." + String(second_octet) + "." + String(third_octet) + "." + String(fourth_octet) );

  Serial.println("IP: " + ipAddressString );

}

int _lastLUXReading;



Adafruit_BMP085 bmp;



void setupBMP085Sensor() {
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
}
int _lastTempC;
int _lastTempF;
double _currentFarenheightEnv = 0.00;
void readBMP085Sensor() {

  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  _lastTempC = bmp.readTemperature();

  int newVal = (_lastTempC * 1.8) + 32;
  if ( newVal != _lastTempF ) {
    _lastTempF = newVal;
    recordValue( "environment", "bmp180_temp_f", String(_lastTempF), _hostname );
  }

  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(101500));
  Serial.println(" meters");

}
#include "leaflift_soil.h"

void setupIO() {
  mcp.begin();      // use default address 0

  //  mcp.pinMode(7, INPUT);
  //  mcp.pullUp(7, HIGH);  // turn on a 100K pullup internally

  //mcp.pinMode(6, OUTPUT);

  mcp.pinMode(0, OUTPUT);
  mcp.pinMode(1, OUTPUT);
  mcp.pinMode(2, OUTPUT);
  mcp.pinMode(3, OUTPUT);
  mcp.digitalWrite(0, HIGH);
  mcp.digitalWrite(1, HIGH);
  mcp.digitalWrite(2, HIGH);
  mcp.digitalWrite(3, HIGH);

  mcp.pinMode(4, OUTPUT);
  mcp.pinMode(5, OUTPUT);
  mcp.pinMode(6, OUTPUT);
  mcp.pinMode(7, OUTPUT);

  mcp.digitalWrite(4, HIGH);
  mcp.digitalWrite(5, HIGH);
  mcp.digitalWrite(6, HIGH);
  mcp.digitalWrite(7, HIGH);


  mcp.pinMode(8, INPUT);
  mcp.pinMode(9, INPUT);
  mcp.pinMode(10, INPUT);
  mcp.pinMode(11, INPUT);
  //  mcp.pullUp(8, HIGH);
  //  mcp.pullUp(9, HIGH);
  //  mcp.pullUp(10, HIGH);
  //  mcp.pullUp(11, HIGH);

  if ( TEST_MODE ) testSwitches();

  //  delay(200);
  //  mcp.digitalWrite(1, LOW);
  //  delay(2000);
  //  mcp.digitalWrite(1, HIGH);
  //  delay(200);
  //  mcp.digitalWrite(1, LOW);
  //  delay(200);
  //  mcp.digitalWrite(1, HIGH);
  //  delay(200);
  //  mcp.digitalWrite(1, LOW);
  //  delay(200);
  //  mcp.digitalWrite(1, HIGH);
  //  delay(200);
  //  mcp.digitalWrite(1, LOW);
  // delay(200);
  //  mcp.digitalWrite(1, HIGH);

}

void testSwitches() {

  currentDisplayText = "Testing Switches...\n";
  updateDisplay();

  for (int n = 0; n < 8; n++) {
    mcp.digitalWrite( n, HIGH );
  }

  for (int n = 0; n < 8; n++) {
    addTextToDisplay( "MCP[" + String(n) + "]\n" );
    mcp.digitalWrite( n, LOW );
    delay( 2000 );
    mcp.digitalWrite( n, HIGH );
    //delay( 2000 );
  }
}
#include "leaflift_bluetooth.h"

void toggleAllSwitches() {

  delay(250);
  if ( switch_state == 0) {
    switch_state = 1;
    mcp.digitalWrite( 0, LOW );
    mcp.digitalWrite( 1, LOW );
    mcp.digitalWrite( 2, LOW );
    mcp.digitalWrite( 3, LOW );

    mcp.digitalWrite( 4, LOW );
    mcp.digitalWrite( 5, LOW );
    mcp.digitalWrite( 6, LOW );
    mcp.digitalWrite( 7, LOW );

  } else {
    switch_state = 0;
    mcp.digitalWrite( 0, HIGH );
    mcp.digitalWrite( 1, HIGH );
    mcp.digitalWrite( 2, HIGH );
    mcp.digitalWrite( 3, HIGH );
    mcp.digitalWrite( 4, HIGH );
    mcp.digitalWrite( 5, HIGH );
    mcp.digitalWrite( 6, HIGH );
    mcp.digitalWrite( 7, HIGH );
  }
}
void sendStatusJSON( String msg ) {
  Serial.println(msg);
  server.send(200, "text/plain", getJSONStatus( msg ) );
}


#include "leaflift_server.h"


String get_i2cString() {
  String i2c_string = "";
  for (int n = 0; n < 10; n++) {
    if ( i2c_devices[n] > 0 ) {
      if (i2c_string.length() > 0) i2c_string += ",";
      i2c_string += "" + String( i2c_devices[n] ) + "";
    }
  }
  return i2c_string;
}


String getJSONStatus( ) {
  return getJSONStatus("");
}

String getJSONStatus( String msg )
{
  String i2c_string = get_i2cString();
  String data = "{\n  \"board_id\":\"" + BOARD_ID + "\",\n  \"chip_id\":\"" + chip_id + "\",\n  \"uptime\":\"" + uptime_string + "\",\n  \"core_version\":\"" + VERSION + "\",\n  \"ip\":\"" + ipAddressString + "\",\n  \"hostname\":\"" + _hostname + "\",\n  \"i2c\":[" + i2c_string + "]";

  data += ",\n  \"api_enabled\": \"" + String(SEND_DATA_TO_API ? "1" : "0") + "\"";
  data += ",\n  \"api_interval\": \"" + String(SEND_DATA_INTERVAL) + "\"";
  data += ",\n  \"api_host\": \"" + String(API_HOST) + "\"";
  data += ",\n  \"ph_sensor\": \"" + String(_phSensorEnabled ? "1" : "0") + "\"";
  data += ",\n  \"temp_probes\": \"" + String(_enableTempProbes ? "1" : "0") + "\"";
  data += ",\n  \"uptime_display\": \"" + String(_uptime_display ? "1" : "0") + "\"";
  data += ",\n  \"bluetooth\": \"" + String(bluetoothAvailable ? "1" : "0") + "\"";
  data += ",\n  \"ota\": \"" + String(_enableOTAUpdate ? "1" : "0") + "\"";
  data += ",\n  \"dht_sensor\": \"" + String( _dhtSensorEnabled ? "1" : "0") + "\"";
  data += ",\n  \"soil\": \"" + String( _soilSensorEnabled ? "1" : "0") + "\"";
  data += ",\n  \"soil_sensor\": \"" + String( _soilSensorEnabled ? "1" : "0") + "\"";

  if ( msg.length() > 0) data += ",\n  \"msg\": \"" + msg + "\"";
  data += "\n}";
  return data;
}


String getJSONData( String msg )
{
  String i2c_string = get_i2cString();

  String data =  "{\n";

  data += "  \"hostname\":\"" + _hostname + "\"";
  data += ",\n  \"core_version\":\"" + VERSION + "\"";
  if ( msg.length() > 0) data += ",\n  \"msg\": \"" + msg + "\"";
  data += ",\n";

  data += "  \"sensors\": {";
  //  data += "\n    \"uid\": \"000000\"";
  data += "\n    \"time\": \"000000\"";

  if ( _phSensorEnabled ) data += ",\n    \"ph\": \"" + String(ph_value_double) + "\"";

  if ( _enableTempProbes ) {
    data += ",\n    \"probes\": {";

    data += "\n      { \"temp_c\": \"" + String(temp_c) + "\" }";
    //data += ",\n    \"temp_c\": \"" + String(temp_c) + "\"";

    data += "\n    }";
  }

  if ( _dhtSensorEnabled ) {
    data += ",\n    \"dht\": {";
    data += "\n      \"dht_temp_f\": \"" + String( dht_temp_f ) + "\"";
    data += ",\n      \"dht_humidity\": \"" + String( dht_humidity ) + "\"";
    data += "\n    }";
  }
  if ( _soilSensorEnabled ) {
    data += ",\n    \"soil\": { ";
    data += "\n      \"sensors\": { ";
    data += "\n         \"1\":\"" + String( sensorReadings[0] ) + "\"";
    data += ",\n         \"2\":\"" + String( sensorReadings[1] ) + "\"";
    data += ",\n         \"3\":\"" + String( sensorReadings[2] ) + "\"";
    data += ",\n         \"4\":\"" + String( sensorReadings[3] ) + "\"";
    data += "\n      }";
 
    data+= _soilConfigJSON;
   
    //data += "\"state\":\"" + _soilState + "\"";
    //data += ", \"moisture\":\"" + String(_soilMoistureReading) + "\" ";
    data += "\n    }";

    
  }
  data += "\n";
  data += "   }\n";
  data += "}\n";
  return data;

}

#include "leaflift_ph.h"


void readTemperatureSensors() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }

  Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;

  currentFarenheight = fahrenheit;
  temp_c = (double)celsius;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
}


void scanI2C() {
  Wire.begin();
  byte error, address;
  Serial.println("Scanning...");
  addTextToDisplay("Scanning i2c...\n");
  i2c_device_count = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);

      Serial.print(" [" + String(address) + "]");
      Serial.println("  !");

      addTextToDisplay( " [" + String(address) + "]\n");

      i2c_devices[i2c_device_count] = address;
      i2c_device_count++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (i2c_device_count == 0) {
    Serial.println("No I2C devices found\n");
    addTextToDisplay( " No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  //delay( 2000 );
}


bool haveIOChip = false;
bool _bluetoothEnabled = false;

#include "hostname_config.h"

bool button1Down = false;
bool button2Down = false;
bool switch1 = false;
bool switch2 = false;

bool button3Down = false;
bool button4Down = false;
bool switch3 = false;
bool switch4 = false;

int n = 0;

// Callback methods prototypes
void CycleCallback();
Task tCycle( 1000, TASK_FOREVER, &CycleCallback, &ts, true);
Task tSensor( 10000, TASK_FOREVER, &SensorCallback, &sensorScheduler, true);

void CycleCallback() {
  n++;
  if (n > 255) n = 0;
  renderDisplay();
}


void SensorCallback() {

  n++;
  if (n > 255) n = 0;

  if ( _enableTempProbes ) readTemperatureSensors();
  
  if ( hasDevice( ph_sensor_address ) ) {
    readPhSensor();
  }
  if ( hasDevice( 57 ) ) {
    readLUXSensor();
  }

  if ( hasDevice( 119 ) ) {
    readBMP085Sensor();
  }

  if (_dhtSensorEnabled ) {
    readDHTSensor();
  }

}


bool hasDevice( int address ) {

  for ( int n = 0; n < 10; n++) {
    if ( i2c_devices[n] == address ) return true;
  }
  return false;
}
void setup() {

  configureHostname();

  Serial.begin( 115200 );
  Serial.println( "Starting" );
  scanI2C();

  if ( hasDevice( 60 ) ) {
    Serial.println("[SSD1306] 128x64 Oled Display Detected 0x3C [60]");
    initDisplay();
    //displayTextOnDisplay("leaflift node");
    //delay(2000);
  } else {
    Serial.println("NO Display Detected");
  }

  if ( hasDevice( 32 ) ) {
    haveIOChip = true;
    Serial.println("[MCP23017] IO Device Detected 0x20 [32]");
    setupIO();
  } else {
    Serial.println("No IO Device detected");
  }

  if ( hasDevice( 72 ) ) {
    Serial.println("[ADS1115] Analog Sensors found 0x48 [72]");
  }

  if ( hasDevice( 99 ) ) {
    Serial.println("Altas pH Sensor found [99]");
  }

  if ( hasDevice( 41 ) ) {
    Serial.println("Luminosity/Lux Sensor [41]");
  }

  if ( hasDevice( 57 ) ) {
    Serial.println("[BMP085] Temp Sensor");
    setupLUXSensor();
  }

  if ( hasDevice( 119 ) ) {
    Serial.println("Barometric Pressure, Temp, Altitude");
    setupBMP085Sensor();
  }
  if (_soilSensorEnabled) setupSoilSensors();

  setupWiFi();

  if ( _bluetoothEnabled ) setupBluetooth();

  if (_dhtSensorEnabled ) {
    setupDHT();
  }

  setupHTTPServer(); Task
  MDNSConnect();
  setupOTAUpdate();

  Serial.println("System ready.");
  recordValue( "system", "status", "ready", _hostname );
}

void MDNSConnect() {
  if (!MDNS.begin( _hostname.c_str() )) {
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS hostname: " + _hostname );
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("rootgrid-node", "tcp", 80);
}

void updateSwitchStatus( String switch_number, bool state ) {

  char host[] = "gbsx.net";
  //urlRequest( "10.5.1.105", "/switch/"+switch_number+"/" + (state?"on":"off") );
  //urlRequest( "10.5.1.105", "/switch/"+switch_number+"/toggle" );
}


#include <SPI.h>            // For SPI comm (needed for not getting compile error)
//#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <ESP_SSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility

//#define OLED_CS     15  // Pin 19, CS - Chip select
//#define OLED_DC     2   // Pin 20 - DC digital signal
#define OLED_RESET  12  // Pin 15 -RESET digital signal

ESP_SSD1306 display(OLED_RESET);



#include "leaflift_images.h"

void renderDisplay() {
  if ( _renderDisplayEnabled == false ) return;
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.setTextSize(2);
  //  display.println(  String(
  //        "  [" +(switch1?"1":"0")+ "][" + (switch2?"1":"0") + "]\n"
  //      + "  [" +(switch3?"1":"0")+ "][" + (switch4?"1":"0")+ "]"
  //  )


  //  display.setCursor(30, 5);
  //  display.println(  "(" + String( switch1?"1":"0") +")(" + String( switch2?"1":"0") +")" );
  //  display.setCursor(30, 30);
  //  display.println(  "(" + String( switch3?"1":"0") +")(" + String( switch4?"1":"0") +")" );
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println( "CORE: " + String( VERSION ) );
  display.println( "HOST: " + String( _hostname ) );
  //display.println( "WiFi: " + String( ssid ) );
  display.println( "  IP: " + String( ipAddressString ) );

  int mt = (int)_tickCount / 60;
  int h = (int)mt / 60;
  int m = (int)mt - (h * 60 );

  int s = ( _tickCount ) - mt * 60;
  String hours = (h < 10 ? "0" : "") + String( h );
  String minutes = (m < 10 ? "0" : "") + String( m );
  String seconds = (s < 10 ? "0" : "") + String( s );
  uptime_string = hours + ":" + minutes + ":" + seconds;

  if ( _uptime_display ) display.println( "  UP: " + uptime_string );

  //display.println( " I2C: " + String(  get_i2cString() ) );

  if (haveIOChip) {
    display.println( " I/O: MCP23017" );
    //renderIOInterface();
  }
  if ( _enableTempProbes ) {
    //display.println( "Temp: " + String( temp_c ) + "'C" );
    display.println( "Temp: " + String( currentFarenheight ) + "'F" );
  }
  if ( _phSensorEnabled ) {
    display.println( "  pH: " + String(ph_value_double) + "" );
  }
  if ( _soilSensorEnabled ) {
    display.println( "Soil: " + String( _lastSoilMoistureReading ) + "" );
  }
  if ( _dhtSensorEnabled ) {
    display.println( "Humidity: " + String( dht_humidity ) + "%" );
  }


  if (bluetoothAvailable) {
    //     display.setCursor(100, 0);
    //     display.println( "(>B)" );
    //display.clearDisplay();
    display.drawBitmap(110, 0, bluetoothIcon, 16, 16, WHITE );
  }
  if (_enableOTAUpdate) {
    display.drawBitmap(96, 48, otaIcon, 32, 16, WHITE );
  }
  if ( hasDevice( 57 ) ) {
    display.println( " LUX: " + String( _lastLUXReading ) + "" );
  }
  if ( hasDevice( 119 ) ) {
    display.println( "Temp: " + String( _lastTempF ) + "'F" );
  }


  display.display();
}

void drawGPIOPin(int pinNum, int state, int x, int y) {

  display.setCursor(x, y);
  display.setTextSize(2);

  if ( state ) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    display.setTextColor(WHITE);
  }
  display.println( String(pinNum) );
}
void renderIOInterface() {
  return;
  for ( int n = 0; n < 8; n++) {
    int state = 1;
    if ( random(1, 10) > 5 ) {
      state = 0;
    } else {
      state = 1;
    }
    drawGPIOPin( n, state, (n * 15), 50 );
  }
}

void renderButtonInterface() {

  display.setCursor(0, 50);
  display.setTextSize(2);

  if ( switch4 ) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    display.setTextColor(WHITE);
  }
  //display.println( "" + String( switch4?"1":"0") +"" );
  display.println( "4" );

  display.setCursor(40, 50);
  display.setTextSize(2);

  if ( switch3 ) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    display.setTextColor(WHITE);
  }
  //display.println( "" + String( switch3?"1":"0") +"" );
  display.println( "3" );

  display.setCursor(75, 50);
  display.setTextSize(2);
  if ( switch1 ) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    display.setTextColor(WHITE);
  }
  //display.println( "" + String( switch1?"1":"0") +"" );
  display.println( "1" );

  display.setCursor(110, 50);
  display.setTextSize(2);
  if ( switch2 ) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    display.setTextColor(WHITE);
  }
  display.println( "2" );
  //display.println( "" + String( switch2?"1":"0") +"" );





}


void addTextToDisplay( String txt ) {
  currentDisplayText += txt;
  updateDisplay();
}

void displayTextOnDisplay( String txt )
{
  currentDisplayText = txt;
  updateDisplay();
}
void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println( String( currentDisplayText ) );
  display.display();
}

void initDisplay()
{
  // SSD1306 Init
  display.begin(SSD1306_SWITCHCAPVCC);  // Switch OLED
  display.clearDisplay();
  display.drawBitmap(0, 0, launchScreen, 128, 128, WHITE );
  display.display();
  delay( 3 * 1000 );
  display.clearDisplay();

  if ( TEST_MODE ) {
    updateDisplay();
    for ( int n = 0; n < 10; n++) {
      addTextToDisplay( ".");
      delay( 100 );
    }
  }
}

bool buttonADown = false;
int buttonAState = 0;

int buttondownstates[] = {false, false, false, false};
int buttonstates[] = {LOW, LOW, LOW, LOW};

int buttonPins[] = {8, 9, 10, 11};
int ledPins[] = {4, 5, 6, 7};

void handleButtons() {

  for ( int n = 0; n < 4; n++ ) {
    int state = mcp.digitalRead(buttonPins[n]);

    if ( LOW == state ) {
      Serial.println("Button[" + String(n) + "] DOWN");
      buttondownstates[n] = true;
    } else {

      // UP
      if ( buttondownstates[n] == true ) {
        Serial.println("Button[" + String(n) + "] UP");
        // toggle
        buttonstates[n] = !buttonstates[n];
      }
    }
    mcp.digitalWrite( ledPins[n], buttonstates[n] );
  }
}

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);


void setupLUXSensor() {


  /* Initialise the sensor */
  if (!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

void readLUXSensor() {

  sensors_event_t event;
  tsl.getEvent(&event);

  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    Serial.print(event.light); Serial.println(" lux");

    int newVal = event.light;
    if ( _lastLUXReading != newVal ) {
      _lastLUXReading = newVal;
      recordValue( "environment", "lux", String(_lastLUXReading), _hostname );
    }

  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }

}
void loop() {

  //printDebug("Loop");
  handleBluetooth();

  //pinValues = read_shift_regs();



  //handleButtons();
  //
  //  if ( buttonAState == 0 ) {
  //    if ( buttonADown ) {
  //      //
  //      Serial.println("Button A held down");
  //    } else {
  //      buttonADown = true;
  //      BT.println("{\"id\":\"buttonA\",\"state\":\"DOWN\"}");
  //
  //    }
  //  } else if ( buttonADown ) {
  //
  //    Serial.println("Button 4 UP");
  //    Serial.println("Toggling state of switch A ");
  //    BT.println("{\"id\":\"buttonA\",\"state\":\"UP\"}");
  //
  //    //buttonAState = true;
  //    buttonADown = false;
  //    renderDisplay();
  //  }
  //mcp.digitalWrite( 6, buttonADown );
  //readShiftRegister();
  //updateShiftRegister( outValues );
  ts.execute();
  sensorScheduler.execute();
  ticker.execute();

  server.handleClient();

  ArduinoOTA.handle();
}

