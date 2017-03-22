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
//https://www.cooking-hacks.com/documentation/tutorials/raspberry-pi-to-arduino-shields-connection-bridge/
//https://www.cooking-hacks.com/forum/viewtopic.php?f=43&t=8167
//https://github.com/pfalcon/esp-open-sdk
//
//        I2C   I2C
//   NC : SDA : SCL : NC  : GND : 3v : 7  : 6  : 5  : 4  : 3  : 2  : 1  : 0
//  -----------------------------------------------------------------------------------
//  |                                                                                 |
//  |                                                                                 |
//  |                                                                                /
//  |                            MCP23017                                           |
//  |                                                                                \ 
//  |                                                                                 |
//  |                                                                                 |
//  -----------------------------------------------------------------------------------
//   A0 : A1 : A3 : RST : INTB : INTA : 0    : 1    : 2    : 3    : 4   : 5   : 6   : 7
//                  10KΩ                Light  Pump   Drain Doser   AC1   AC2   AC3   Ac4
//   GND  GND  GND   V+
//   (I2C Addr 32)

String VERSION = "0.1-redish";

String BOARD_ID = "";
bool _buttonBoardConnected = false;
bool TEST_MODE = false;
String chip_id = "";
String _hostname = "";




int _lastTempC;
int _lastTempF;
int _lastAltitude;
int _lastPressure;




bool _co2_sensor_enabled = false;
unsigned int _currentCO2 = 0;


int dry_calibration[] = {0, 0, 0, 0};
int wet_calibration[] = {0, 0, 0, 0};
bool _showBootScreen = false;

int _now = 0;
/**
   This will set the value _now using the api

 **/

bool _enableTempProbes = false;

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BMP085.h>
#include <OneWire.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_STATUS_REQUEST
#include <TaskScheduler.h>

#include "leaflift_color_display.h"

int oneWirePin = 0;
OneWire  ds(oneWirePin);  //a 2.2K resistor is necessary for 3.3v on the signal line, 4.7k for 5v

Scheduler ts;
Scheduler sensorScheduler;
Scheduler updateScheduler;

#include "leaflift_crypto.h"

#include "Adafruit_MCP23017.h"
Adafruit_MCP23017 mcp;

//#include <ESP8266httpUpdate.h>


bool recentSensorErrors = false;

String API_HOST = "10.5.1.160";
int API_PORT = 80;
//char API_HOST[] = "10.5.1.25";
//int API_PORT = 3000;

bool SEND_DATA_TO_API = false;
int SEND_DATA_INTERVAL = 10000;

bool _soilSensorEnabled = false;

double temp_c = 0.00;
double ph_value_double = 0.00;


int i2c_devices[10];
int i2c_device_count = 0;
String currentDisplayText = "";
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
bool _renderDisplayEnabled = true;

bool bluetoothAvailable = false;
bool _enableOTAUpdate = true;
bool _phSensorEnabled = false;
bool _uptime_display = true;
String uptime_string = "";
bool _luxSensorEnabled = false;
bool _BMP085Enabled = false;
bool _flowCounterEnabled = false;

#include <SoftwareSerial.h>

//SoftwareSerial BT(14, 12);
// creates a "virtual" serial port/UART
// connect BT module TX to D10
// connect BT module RX to D11
// connect BT Vcc to 5V, GND to GND




#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>


#include "leaflift_ADS1X15.h"

char* wifi_ssid     = "gbsx";
char* wifi_psk = "OrlandoNakazawa!";


String ipAddressString = "";
ESP8266WebServer server(80);

const char WiFiAPPSK[] = "password";

const char *ap_name      = "Enterprise-D";
const char *ap_passwd    = "1234567890";

void setupWiFi()
{
  WiFi.mode(WIFI_STA);
  Serial.println("Connecting to wiFi ssid: " + String( wifi_ssid ) + "...");

  displayTextOnDisplay("Connecting to:\n   " + String(wifi_ssid) + "...");
  WiFi.begin(wifi_ssid, wifi_psk);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    addTextToDisplay( "." );
  }
  saveIP();
}

bool promptVisible = false;
int promptTimeout = 0;
String promptTitle;
String promptText;
double promptLength = 0.00;

#include "leaflift_api.h"

#include "leaflift_switches.h"



unsigned long _tickCount = 0;
#include "flow_counter.h"

Scheduler ticker;

Task tTicker( 1000, TASK_FOREVER, &TickCallback, &ticker, true);

void TickCallback() {
  _tickCount++;
  // if we have counted 60 seconds
  if ( _tickCount % 60 == 0 ) {
  }


  if ( _flowCounterEnabled ) {
    flowCounterHandler();
  }
  _now++;
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

double _currentFarenheightEnv = 0.00;
void readBMP085Sensor() {

  _lastTempC = bmp.readTemperature();

  int newVal = (_lastTempC * 1.8) + 32;
  if ( newVal != _lastTempF ) {
    _lastTempF = newVal;
    recordValue( "environment", "bmp180_temp_f", String(_lastTempF), _hostname );
  }

  _lastPressure = bmp.readPressure();
  Serial.print("Pressure = ");
  Serial.print( String(_lastPressure) );
  Serial.println(" Pa");

  _lastAltitude = bmp.readAltitude();

  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print( String(_lastAltitude) );
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

  //  mcp.pinMode(0, OUTPUT);
  //  mcp.pinMode(1, OUTPUT);
  //  mcp.pinMode(2, OUTPUT);
  //  mcp.pinMode(3, OUTPUT);
  //  mcp.digitalWrite(0, HIGH);
  //  mcp.digitalWrite(1, HIGH);
  //  mcp.digitalWrite(2, HIGH);
  //  mcp.digitalWrite(3, HIGH);
  //
  //  mcp.pinMode(4, OUTPUT);
  //  mcp.pinMode(5, OUTPUT);
  //  mcp.pinMode(6, OUTPUT);
  //  mcp.pinMode(7, OUTPUT);
  //
  //  mcp.digitalWrite(4, HIGH);
  //  mcp.digitalWrite(5, HIGH);
  //  mcp.digitalWrite(6, HIGH);
  //  mcp.digitalWrite(7, HIGH);
  //
  //
  //  mcp.pinMode(8, INPUT);
  //  mcp.pinMode(9, INPUT);
  //  mcp.pinMode(10, INPUT);
  //  mcp.pinMode(11, INPUT);
  //  mcp.pinMode(12, OUTPUT);
  //  mcp.pinMode(13, OUTPUT);
  //  mcp.pinMode(14, OUTPUT);
  //  mcp.pinMode(15, OUTPUT);
  //  mcp.pullUp(8, HIGH);
  //  mcp.pullUp(9, HIGH);
  //  mcp.pullUp(10, HIGH);
  //  mcp.pullUp(11, HIGH);

  //if ( TEST_MODE ) testSwitches();

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

int switch_state = 0;
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


void sendJSON( String msg ) {
  Serial.println(msg);
  server.send(200, "text/plain", msg );
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
  String data = "{\n  \"board_id\":\"" + BOARD_ID + "\",\n  \"chip_id\":\"" + chip_id + "\",\n  \"now\":\"" + String(_now) + "\",\n  \"uptime\":\"" + uptime_string + "\",\n  \"core_version\":\"" + VERSION + "\",\n  \"ip\":\"" + ipAddressString + "\",\n  \"hostname\":\"" + _hostname + "\",\n  \"i2c\":[" + i2c_string + "]";

  data += ",\n  \"api_enabled\": \"" + String(SEND_DATA_TO_API ? "1" : "0") + "\"";
  data += ",\n  \"api_interval\": \"" + String(SEND_DATA_INTERVAL) + "\"";
  data += ",\n  \"api_host\": \"" + String(API_HOST) + "\"";
  data += ",\n  \"api_port\": \"" + String(API_PORT) + "\"";
  data += ",\n  \"ph_sensor\": \"" + String(_phSensorEnabled ? "1" : "0") + "\"";
  data += ",\n  \"temp_probes\": \"" + String(_enableTempProbes ? "1" : "0") + "\"";
  data += ",\n  \"uptime_display\": \"" + String(_uptime_display ? "1" : "0") + "\"";
  data += ",\n  \"bluetooth\": \"" + String(bluetoothAvailable ? "1" : "0") + "\"";
  data += ",\n  \"ota\": \"" + String(_enableOTAUpdate ? "1" : "0") + "\"";
  data += ",\n  \"lux_sensor\": \"" + String( _luxSensorEnabled ? "1" : "0") + "\"";
  data += ",\n  \"dht_sensor\": \"" + String( _dhtSensorEnabled ? "1" : "0") + "\"";
  data += ",\n  \"soil\": \"" + String( _soilSensorEnabled ? "1" : "0") + "\"";
  data += ",\n  \"soil_sensor\": \"" + String( _soilSensorEnabled ? "1" : "0") + "\"";


  //data += ",\n  \"channels\":\n" + String( getChannelStatusJSON() ) + "\n";
  data += ",\n" + getChannelsNode();

  if ( msg.length() > 0) data += ",\n  \"msg\": \"" + msg + "\"";
  data += "\n}";
  return data;
}


String getJSONData( String msg )
{
  String i2c_string = get_i2cString();

  String data =  "{\n";

  // always have a first entry, that way all next ones prefix a comma
  //data += "  \"uid\": \"000000\"";

  data += "  \"hostname\":\"" + _hostname + "\"";
  data += ",\n  \"core_version\":\"" + VERSION + "\"";
  if ( msg.length() > 0) data += ",\n  \"msg\": \"" + msg + "\"";
  data += ",\n";

  data += "  \"now\":\"" + String(_now) + "\",\n";


  data += "  \"environment\": {";

  data += "\n    \"light\": {";
  if ( _luxSensorEnabled ) {
    data += "\n      \"lux\": " + String( _lastLUXReading ) + "";
  }
  data += "\n    }";

  if ( _dhtSensorEnabled || _co2_sensor_enabled ) {
    data += ",\n    \"air\": {";

    if ( _co2_sensor_enabled ) {
      data += "\n       \"co2\": " + String( _currentCO2 ) + ",";
    }
    data += "\n       \"humidity\": " + String( dht_humidity ) + ",";
    data += "\n       \"temp\": { \n";
    data += "          \"f\": " + String( _lastTempF ) + "";
    //data += ",\n          \"c\": \"" + String( dht_temp ) + "\"";
    data += "\n       }";
    data += "\n    }\n";
  }

  //
  //  data += ",\n    \"water\": {";
  //  data += "\n    }\n";
  //
  //  data += ",\n    \"soil\": {";
  //  data += "\n    }\n";


  // CLOSE environment node
  data += "  }\n";


  if ( true ) {
    data += ",\n  \"sensors\": {";

    data += "  \"uid\": \"000000\"";

    if ( _phSensorEnabled ) data += ",\n    \"ph\": \"" + String(ph_value_double) + "\"";

    if ( _enableTempProbes ) {
      data += ",\n    \"probes\": {";
      String probeid = "avg";
      data += "\n      \"" + probeid + "\": { \"temp_c\": \"" + String(temp_c) + "\" }";
      //data += ",\n    \"temp_c\": \"" + String(temp_c) + "\"";

      data += "\n    }";
    }


    if ( _flowCounterEnabled ) {

      data += ",\n    \"flow\": {";
      data += "\n      \"pin\": \"" + String( flowPin1 ) + "\"";
      data += ",\n      \"count\": \"" + String( flowCounter ) + "\"";
      data += ",\n      \"lpm\": \"" + String( _lastLPM ) + "\"";
      data += ",\n      \"lph\": \"" + String( litersPerHour ) + "\"";
      data += "\n    }";
    }

    if ( _luxSensorEnabled ) {
      data += ",\n    \"tsl2561\": {";
      data += "\n      \"lux\": \"" + String( _lastLUXReading ) + "\"";
      data += "\n    }";
    }
    if ( _BMP085Enabled ) {
      data += ",\n    \"bmp085\": {";
      data += "\n      \"temp_c\": \"" + String( _lastTempC ) + "\"";
      data += ",\n      \"temp_f\": \"" + String( _lastTempF ) + "\"";
      data += ",\n      \"altitude\": \"" + String( _lastAltitude ) + "\"";
      data += ",\n      \"pressure\": \"" + String( _lastPressure ) + "\"";
      data += "\n    }";
    }
    if ( _dhtSensorEnabled ) {
      data += ",\n    \"dht11\": {";
      data += "\n      \"dht_temp_f\": \"" + String( dht_temp_f ) + "\"";
      //data += "\n      \"dht_temp_c\": \"" + String( dht_temp_c ) + "\"";
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

      data += ",\n      \"readings\": { ";
      data += "\n         \"1\":" + String( sensorReadingsNormal[0] ) + "";
      data += ",\n         \"2\":" + String( sensorReadingsNormal[1] ) + "";
      data += ",\n         \"3\":" + String( sensorReadingsNormal[2] ) + "";
      data += ",\n         \"4\":" + String( sensorReadingsNormal[3] ) + "";
      data += "\n      }";


      data += _soilConfigJSON;

      //data += "\"state\":\"" + _soilState + "\"";
      //data += ", \"moisture\":\"" + String(_soilMoistureReading) + "\" ";
      data += "\n    }";
    }

    data += "\n";
    data += "   }";
  }//end sensors node enabled conditional

  //  if ( _useIOForSwitchChannels ) {
  //    data += ",\n  \"switches\": [\n";
  //
  //    data += "    {\n";
  //    data += "      \"controller\":\"" + String(_useIOForSwitchChannels == true ? "mcp23017" : "gpio") + "\",\n";
  //
  //    data += "      \"channels\": { \n";
  //    data += "        \"1\": { \"pin\": " + String(ch1_pin) + ", \"state\": " + String(ch1_state) + ", \"label\":\"" + ch1_label + "\" },\n";
  //    data += "        \"2\": { \"pin\": " + String(ch2_pin) + ", \"state\": " + String(ch2_state) + ", \"label\":\"" + ch2_label + "\" },\n";
  //    data += "        \"3\": { \"pin\": " + String(ch3_pin) + ", \"state\": " + String(ch3_state) + ", \"label\":\"" + ch3_label + "\" },\n";
  //    data += "        \"4\": { \"pin\": " + String(ch4_pin) + ", \"state\": " + String(ch4_state) + ", \"label\":\"" + ch4_label + "\" },\n";
  //    data += "        \"5\": { \"pin\": " + String(ch5_pin) + ", \"state\": " + String(ch5_state) + ", \"label\":\"" + ch5_label + "\" },\n";
  //    data += "        \"6\": { \"pin\": " + String(ch6_pin) + ", \"state\": " + String(ch6_state) + ", \"label\":\"" + ch6_label + "\" },\n";
  //    data += "        \"7\": { \"pin\": " + String(ch7_pin) + ", \"state\": " + String(ch7_state) + ", \"label\":\"" + ch7_label + "\" },\n";
  //    data += "        \"8\": { \"pin\": " + String(ch8_pin) + ", \"state\": " + String(ch8_state) + ", \"label\":\"" + ch8_label + "\" }\n";
  //    data += "      }\n";
  //
  //    data += "    }\n";
  //    data += "  ]\n";
  //  }

  data += "\n}\n";
  return data;

}


#include "FS.h"
#include "leaflift_filesystem.h"



#include "leaflift_ph.h"


void readTemperatureSensors() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

  Serial.println("Reading temperature probes...");
  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
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


  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");


  if ( temp_c != (double)celsius ) {
    temp_c = (double)celsius;

    // HACK: since the ph Sensor reports the probe temp, we only want to report if it's not enabled
    if ( !_phSensorEnabled ) {
      recordValue( "environment", "probe_temp_f", String( fahrenheit ), _hostname );
    }
  }


}


void scanI2C() {

  // Standard hookup is 5, 4
  Wire.begin( 5, 4 );


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
Task tSensor( (5 * 1000), TASK_FOREVER, &SensorCallback, &sensorScheduler, true);
Task updateCycle( 5 * 60 * 1000, TASK_FOREVER, &checkForUpdates, &updateScheduler, true );


int report_interval = 10 * 60 * 1000;
Task tReportSensorData( report_interval, TASK_FOREVER, &ReportSensorData, &sensorScheduler, true);
int displayStepCount = 0;
int displayPhase = 0;


void checkForUpdates(){
  
  Serial.println("------------------------------ SYSTEM CYCLE CALLBACK ------------------------------");
  // Check for update
  String response = urlRequest( "10.5.1.160", "/nodered/update.json?current="+VERSION+"&nodeid=" + _hostname, 80 );
  Serial.println("RESPONSE: [" + response + "]");

  
}

void CycleCallback() {
  n++;


  displayStepCount++;

  switch ( displayPhase ) {
    default:
    case ( 0 ): // regular display
      if ( displayStepCount >= 10 )
      {
        displayStepCount = 0;
        displayPhase = 1;
      }
      break;
    case ( 1 ): // screen save
      if ( displayStepCount >= 1 )
      {
        displayStepCount = 0;
        displayPhase = 0;
      }

      break;
  }

  if (n > 255) n = 0;
  renderDisplay();
}

void ReportSensorData() {

  //apiPOST( "/v1/record/sensordata", getJSONData("") );

}

void SensorCallback() {

  Serial.println("\n----------------------------------\nReading Sensors...\n\n" );
  n++;
  if (n > 255) n = 0;
  if ( _enableTempProbes ) readTemperatureSensors();

  if ( _co2_sensor_enabled ) {
    read_K30();
  }

  if ( hasDevice( ph_sensor_address ) ) {
    readPhSensor();
  }
  if ( hasDevice( 57 ) ) {
    readLUXSensor();
  }

  if (_dhtSensorEnabled ) {
    readDHTSensor();
  }

  //if ( hasDevice( 119 ) ) {
  if ( _BMP085Enabled ) {
    readBMP085Sensor();
  }
  if ( hasDevice( 72 ) ) {
    readADS1X15();
  }
}

#include <WiFiManager.h>
#include <DNSServer.h>

bool hasDevice( int address ) {

  for ( int n = 0; n < 10; n++) {
    if ( i2c_devices[n] == address ) return true;
  }
  return false;
}


void saveWiFiConfigCallback () {

  Serial.println("Should save config");
  displayTextOnDisplay("Saving Config, and rebooting...");

}
//WiFiManager wifiManager;

char loaded_api_url[205] = "api.leaflift.local";
char notes[255] = "";

bool shouldSaveConfig = false;
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


void activateSetupMode() {

  _renderDisplayEnabled = false;
  displayTextOnDisplay("Entering Setup mode");

  String ssid = String("PROF-") + String(ESP.getChipId(), HEX);


  displayTextOnDisplay("Connect to configure: \nSSID: \n" + ssid  );//+ "\n\nPass: \n" + String(ap_default_psk ) + "" );


  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/professor_config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/professor_config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          //          strcpy(loaded_api_url, json["api_url"]);
          //          strcpy(notes, json["notes"]);

          //          strcpy(mqtt_port, json["mqtt_port"]);
          //          strcpy(blynk_token, json["blynk_token"]);

          Serial.println("loaded_api_url: " + String(loaded_api_url) );


        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_api_url("api_url", "API url", loaded_api_url, 205 );

  WiFiManagerParameter custom_notes("notes", "Notes", notes, 255 );


  //  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  //  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10, 0, 1, 99), IPAddress(10, 0, 1, 1), IPAddress(255, 255, 255, 0));

  //add all your parameters here
  wifiManager.addParameter(&custom_api_url);
  wifiManager.addParameter(&custom_notes);

  //reset settings - for testing
  wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  wifiManager.setMinimumSignalQuality(50);

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect( ssid.c_str(), "123456" )) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //wifiManager.startConfigPortal( ssid.c_str(), "123456" );

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  //read updated parameters
  strcpy( loaded_api_url, custom_api_url.getValue() );
  strcpy( notes, custom_notes.getValue() );

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    displayTextOnDisplay("Saved config.");

    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();

    json["api_url"] = loaded_api_url;
    json["notes"] = notes;

    File configFile = SPIFFS.open("/professor_config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  saveIP();
  _renderDisplayEnabled = true;
}


void ___activateSetupMode() {

  WiFiManager wifiManager;
  String ssid = String("PROF-") + String(ESP.getChipId(), HEX);

  _renderDisplayEnabled = false;
  displayTextOnDisplay("Entering Setup mode");
  displayTextOnDisplay("Connect to configure: \nSSID: \n" + ssid  );//+ "\n\nPass: \n" + String(ap_default_psk ) + "" );


  //wifiManager.autoConnect( "PROF-12345", "12345" );

  //wifiManager.setCustomHeadElement("<style>html{filter: invert(100%); -webkit-filter: invert(100%);}</style>");


  WiFiManagerParameter custom_text("<h1>node config</h1>");
  wifiManager.addParameter(&custom_text);

  wifiManager.setMinimumSignalQuality(50);

  WiFiManagerParameter api_url( "API", "API", "api.leafliftsystems.com/v2", 205, " readonly" );
  wifiManager.addParameter(&api_url);

  wifiManager.setSaveConfigCallback(saveWiFiConfigCallback);
  wifiManager.startConfigPortal( ssid.c_str(), "123456" );

}


#define SERIAL_VERBOSE
#define HOSTNAME "PROF-" ///< Hostname. The setup function adds the Chip ID at the end.
const char* ap_default_ssid = "professor"; ///< Default SSID.
const char* ap_default_psk  = "professor"; ///< Default PSK.
void _activateSetupMode() {
  WiFiManager wifiManager;
  _hostname += String(HOSTNAME) + String(ESP.getChipId(), HEX);
  String ssid = _hostname;//String(ap_default_ssid) + "-"+ String(ESP.getChipId(), HEX);

  _renderDisplayEnabled = false;
  displayTextOnDisplay("Entering Setup mode");
  //delay( 5 * 1000 );
  displayTextOnDisplay("Connect to configure: \nSSID: \n" + ssid + "\n\nPass: \n" + String(ap_default_psk ) + "" );

  // Go into software AP mode.
  WiFi.mode(WIFI_AP);
  delay(10);
  WiFi.softAP(_hostname.c_str(), ap_default_psk);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  MDNSConnect();
  setupOTAUpdate();

  server.on("/", handleRoot);
  server.begin();
}

void handleRoot() {
  server.send(200, "text/html", "<h1>You are connected</h1>");
}


void setup() {
  Serial.begin( 115200 );
  Serial.println( "Setup the professor..." );
  scanI2C();

  if ( hasDevice( 60 ) ) {
    Serial.println("[SSD1306] 128x64 Oled Display Detected 0x3C [60]");
    initDisplay();
    //displayTextOnDisplay("leaflift node");
    //delay(2000);
  } else {
    Serial.println("NO Display Detected");
  }

  //  // this will look for the config file
  //  if ( !setupFilesystem() ) {

  //activateSetupMode();
  //    return;
  //  }

  configureHostname();
  
  setup_button_pins();


  if ( hasDevice( 32 ) ) {
    haveIOChip = true;
    Serial.println("[MCP23017] IO Device Detected 0x20 [32]");
    setupIO();
  } else {
    Serial.println("No IO Device detected");
  }

  if ( hasDevice( 72 ) ) {
    Serial.println("[ADS1115] Analog Sensors found 0x48 [72]");
    setupADS1X15();
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
  if ( _co2_sensor_enabled ) {
    setup_K30();
  }
  if (_soilSensorEnabled) setupSoilSensors();
  if ( _bluetoothEnabled ) setupBluetooth();
  if (_dhtSensorEnabled ) {
    setupDHT();
  }

  if ( _flowCounterEnabled ) {
    setupFlowCounter();
  }
  Serial.println("System ready.");

  connectNetwork();
 
}


void connectNetwork(){

   // connect to the network.
  setupWiFi();
  MDNSConnect();
  setupOTAUpdate();
  setupHTTPServer();
  getTime();

  apiPOST( "/nodered/node-ready", getJSONStatus() );
 
  //postValue( "system", "status", "ready" );
  //recordValue( "system", "status", "ready", _hostname );
}

void MDNSConnect() {
  if (!MDNS.begin( _hostname.c_str() )) {
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS hostname: " + _hostname );
  //MDNS.addService("http", "tcp", 80);
  MDNS.addService("rootgrid-node", "tcp", 80);
}

void updateSwitchStatus( String switch_number, bool state ) {

  char host[] = "gbsx.net";
  //urlRequest( "10.5.1.105", "/switch/"+switch_number+"/" + (state?"on":"off") );
  //urlRequest( "10.5.1.105", "/"+switch_number+"/toggle" );
}


#include <SPI.h>            // For SPI comm (needed for not getting compile error)
//#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <ESP_SSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility

//#define OLED_CS     15  // Pin 19, CS - Chip select
//#define OLED_DC     2   // Pin 20 - DC digital signal
#define OLED_RESET  10  // Pin 15 -RESET digital signal

ESP_SSD1306 display(OLED_RESET);


#include "leaflift_images.h"

void renderDisplay() {

  if ( _renderDisplayEnabled == false ) return;

  if ( displayPhase == 1 ) {
    display.clearDisplay();
    display.drawBitmap(0, 0, launchScreen, 128, 128, WHITE );
    display.display();
    return;
  }

  if ( _hostname == "aqua" ) {
    // use a predefined util screen
    display.clearDisplay();
    display.setTextColor(WHITE);


    display.setTextSize(1);
    display.setCursor(2, 0);
    display.println( "Water: ");

    display.setTextSize(2);
    display.setCursor(2, 10);
    display.println( "pH " + String(ph_value_double) + "" );

    display.setCursor(2, 30);
    display.println( "" + String( currentFarenheight ) + "'F" );

    display.setTextSize(1);
    display.setCursor(2, 55);
    display.println( "Air: " + String( dht_temp_f ) + "'F" + " " + String( dht_humidity ) + "%" );


    display.display();

    return;
  }

  if ( _hostname == "professor1" ) {
    // use a predefined util screen
    display.clearDisplay();
    display.setTextColor(WHITE);



    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println( "" + String( int(dht_temp_f)) + "'F " + String( int(dht_humidity) ) + " % " );

    display.setCursor(0, 20);
    display.println( "" + String( "LUX " ) + String(_lastLUXReading)  );

    display.setTextSize(1);
    display.setCursor(0, 45);
    display.println( "CO2"  );
    display.setTextSize(2);
    display.setCursor(20, 40);
    display.println( "" + String( _currentCO2 ) + "ppm"  );

    display.display();

    return;
  }


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
  display.println( "WiFi: " + String( wifi_ssid ) );
  display.println( "  IP: " + String( ipAddressString ) );

  int mt = (int)_tickCount / 60;
  int h = (int)mt / 60;
  int m = (int)mt - (h * 60 );

  int s = ( _tickCount ) - mt * 60;
  String hours = (h < 10 ? "0" : "") + String( h );
  String minutes = (m < 10 ? "0" : "") + String( m );
  String seconds = (s < 10 ? "0" : "") + String( s );
  uptime_string = hours + ": " + minutes + ": " + seconds;

  //if ( _uptime_display ) display.println( "  UP: " + uptime_string );

  //display.println( " I2C: " + String(  get_i2cString() ) );

  if (haveIOChip) {
    //display.println( " I / O: MCP23017" );
    //renderIOInterface();
    renderButtonInterface();
  }
  if ( _enableTempProbes ) {
    //display.println( "Temp: " + String( temp_c ) + "'C" );
    display.println( "Probes: " + String( currentFarenheight ) + "'F" );
  }
  if ( _phSensorEnabled ) {
    display.println( "  pH: " + String(ph_value_double) + "" );
  }
  if ( _soilSensorEnabled ) {
    display.println( "Soil: " + String( _lastSoilMoistureReading ) + "" );
  }
  if ( _dhtSensorEnabled ) {
    display.println( "Humidity: " + String( dht_humidity ) + " % " );
    display.println( "    Temp: " + String( dht_temp_f ) + "'F" );
  }

  if ( _flowCounterEnabled ) {
    display.println( " LPH: " + String( litersPerHour ) + "" );
  }


  if (bluetoothAvailable) {
    //     display.setCursor(100, 0);
    //     display.println( "(>B)" );
    //display.clearDisplay();
    display.drawBitmap(110, 0, bluetoothIcon, 16, 16, WHITE );
  }

  if ( recentSensorErrors ) {
    display.drawBitmap(110, 78, sensorErrorsIcon, 16, 16, WHITE );
  }

  //  if (_enableOTAUpdate) {
  //    display.drawBitmap(96, 48, otaIcon, 32, 16, WHITE );
  //  }
  //if ( hasDevice( 57 ) ) {
  if ( _luxSensorEnabled ) {
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
  //return;
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

  if ( buttonstates[0] == HIGH ) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    display.setTextColor(WHITE);
  }
  //display.println( "" + String( switch4?"1":"0") +"" );
  display.println( "1" );



  display.setCursor(40, 50);
  display.setTextSize(2);

  if ( buttonstates[1] == HIGH ) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    display.setTextColor(WHITE);
  }
  //display.println( "" + String( switch3?"1":"0") +"" );
  display.println( "2" );

  display.setCursor(75, 50);
  display.setTextSize(2);
  if ( buttonstates[2] == HIGH ) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    display.setTextColor(WHITE);
  }
  //display.println( "" + String( switch1?"1":"0") +"" );
  display.println( "3" );

  display.setCursor(110, 50);
  display.setTextSize(2);
  if ( buttonstates[3] == HIGH ) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  } else {
    display.setTextColor(WHITE);
  }
  display.println( "4" );
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

  if ( _hostname == "aqua" ) {
    return;
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println( String( currentDisplayText ) );
  display.display();

}

#define DIMMER_I2C_ADDR 0x27
#define SELECT_DIMMER_CH1 0x80
#define SELECT_DIMMER_CH2 0x81
#define SELECT_DIMMER_CH3 0x82
#define SELECT_DIMMER_CH4 0x83


void setDimmerChannel( int channel, int value ) {

  if ( hasDevice( DIMMER_I2C_ADDR ) ) {
    Wire.beginTransmission(DIMMER_I2C_ADDR); // device address
    Wire.write( SELECT_DIMMER_CH1 );
    Wire.write( value );
    Wire.endTransmission();
  } else {
    Serial.println( "Missing A/C Dimmer device: " + String(DIMMER_I2C_ADDR) );
  }
}

void initDisplay()
{
  _showBootScreen = false;
  // SSD1306 Init
  display.begin(SSD1306_SWITCHCAPVCC);  // Switch OLED
  display.clearDisplay();
  if ( _showBootScreen ) {
    display.drawBitmap(0, 0, launchScreen, 128, 128, WHITE );
    display.display();
    delay( 10  * 1000 );
    display.clearDisplay();
  }

  if ( TEST_MODE ) {
    updateDisplay();
    for ( int n = 0; n < 10; n++) {
      addTextToDisplay( ".");
      delay( 100 );
    }
  }
}


void checkForUpdate() {

  Serial.println("Checking for Update...");




}

bool buttonADown = false;
int buttonAState = 0;

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);


void setupLUXSensor() {

  _luxSensorEnabled = true;
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
  Serial.print  ("Gain :         "); Serial.println("Auto");
  Serial.print  ("Timing :       "); Serial.println("13 ms");
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

// ================================================================
//#include "kSeries.h" //include kSeries Library
//kSeries K_30(12,13); //Initialize a kSeries Sensor with pin 12 as Rx and 13 as Tx
//
//void read_K30() {
//
//  double co2 = K_30.getCO2('p'); //returns co2 value in ppm ('p') or percent ('%')
//
//   _currentCO2 = (int)co2;
//}

// ================================================================
SoftwareSerial K_30_Serial(12, 13); //Sets up a virtual serial port
//Using pin 12 for Rx and pin 13 for Tx
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25}; //Command packet to read Co2 (see app note)
byte response[] = {0, 0, 0, 0, 0, 0, 0}; //create an array to store the response
//multiplier for value. default is 1. set to 3 for K-30 3% and 10 for K-33 ICB
int valMultiplier = 1;
int K30_delayTime = 50;

void setup_K30() {

  K_30_Serial.begin(9600); //Opens the virtual serial port with a baud of 9600
  Serial.println(" \n\n\nSetup K-30 Sensor");
}
void read_K30() {

  Serial.println(" Read K - 30 Sensor");
  poll_K30(readCO2);  
  _currentCO2 = getCO2Value(response);
  
  Serial.print("Co2 ppm = ");
  Serial.println(_currentCO2);

}
void poll_K30(byte packet[])
{
  Serial.println(" poll Sensor");
  while (!K_30_Serial.available()) //keep sending request until we start to get a response
  {
    K_30_Serial.write(readCO2, 7);
    delay(K30_delayTime);

  }
  int timeout = 0; //set a timeout counter
  while (K_30_Serial.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;
    if (timeout > 10) //if it takes too long there was probably an error
    {
      while (K_30_Serial.available()) //flush whatever we have
        K_30_Serial.read();
      break; //exit and try again
    }
    delay(K30_delayTime);
  }
  for (int i = 0; i < 7; i++)
  {
    response[i] = K_30_Serial.read();
  }
}
int getCO2Value(byte packet[])
{
//  int high = packet[3]; //high byte for value is 4th byte in packet in the packet
//  int low = packet[4]; //low byte for value is 5th byte in the packet
//
//  Serial.println( "high: " + String(high) );
//  Serial.println( "low: " + String(low) );

  int co2value = static_cast<int>(packet[3]) * 256 + static_cast<int>(packet[4]);
  return co2value * valMultiplier;
//  unsigned long val = high * 256 + low; //Combine high byte and low byte with this formula to get value
//  return val * valMultiplier;
}
// ================================================================




void loop() {

  //handleBluetooth();
  read_button_pins();
  //pinValues = read_shift_regs();

  ts.execute();
  sensorScheduler.execute();
  ticker.execute();
  server.handleClient();

  updateScheduler.execute();

  
  ArduinoOTA.handle();
}

