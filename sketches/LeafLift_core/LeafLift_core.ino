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
//          SDA    SCL   Temp                                 Rx2   Tx2   Rx0   Tx0
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

int oneWirePin = 0;
OneWire  ds(oneWirePin);  //a 2.2K resistor is necessary for 3.3v on the signal line, 4.7k for 5v


#include "leaflift_crypto.h"

#include "Adafruit_MCP23017.h"

//char API_HOST[] = "api-quadroponic.rhcloud.com";
char API_HOST[] = "10.5.1.25";
int API_PORT = 3000;

bool SEND_DATA_TO_API = true;
int SEND_DATA_INTERVAL = 10000;

double temp_c = 0.00;
double ph_value_double = 0.00;

String BOARD_ID = "";

String VERSION = "bird";

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


#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_STATUS_REQUEST
#include <TaskScheduler.h>

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

#include "leaflift_OTA.h"

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



void setupSoilSensor() {
  pinMode(A0, INPUT); //set up analog pin 0 to be input
  mcp.pinMode(3, OUTPUT);
  mcp.digitalWrite(3, LOW);
  //pinMode(10, OUTPUT);

}
int _lastLUXReading;
int numSoilSamples = 5;
int _soilMoistureReadings[5] = {0, 0, 0, 0, 0};
int _soilMoistureReadingIndex = 0;

int _lastSoilMoistureReading = 0;
int _soilMoistureReading = 0;
bool _soilSensorEnabled = false;
bool _enableTempProbes = false;
String _soilState = "?";

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
  if( newVal != _lastTempF ){
    _lastTempF = newVal;
    recordValue( "environment", "temp_f", String(_lastTempF), _hostname );
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

char ch;
bool bluetoothAvailable = false;
void setupBluetooth() {

  displayTextOnDisplay("initializing\nBluetooth...\n");

  Serial.println("initializing Bluetooth...");
  // set the data rate for the SoftwareSerial port
  BT.begin(9600);

  delay( 1 * 1000);
  //
  //  BT.println( "AT" );
  //  delay(1000);
  //  BT.println( "AT+NAME" );
  //  delay(1000);
  //
  String ok = sendATCommand( "AT", "OK", 10 * 1000 );
  if ( ok == "OK") {
    bluetoothAvailable = true;
    Serial.println("Found BT module.");
    Serial.println("Setting name: " + String(_hostname) );
    addTextToDisplay( " + Found Adapter" );
    addTextToDisplay( " + Setting name to [" + String(_hostname) + "]" );

    String r = sendATCommand( "AT+NAME" + String(_hostname), "OKsetname", 2000 );
    //BT.print("AT+NAME" + String(_hostname)  );
    //BT.print("AT+NAME");
    //delay(600);
    Serial.print("BT Name: " + r );
    addTextToDisplay( "response: " + String( r ) );

    String b = sendATCommand( "AT+BAUD8", "OKsetbaud", 2000 );

    // Send test message to other device
    //BT.println( getJSONStatus() );
  } else {
    Serial.println("BT not available");
  }
  delay( 1000 );
}

String sendATCommand(String command, String expectedResponse, int timeout ) {
  Serial.println("> " + command );
  addTextToDisplay( "> " + command + "\n" );
  int timeoutTime = millis() + timeout;
  String response = "";
  int count = 0;
  int len = expectedResponse.length();

  BT.print( command );

  while (1) {

    if ( millis() >= timeoutTime ) {

      addTextToDisplay( "Timeout.\n" );
      Serial.println("Command timeout.");
      break;
      return "";
    }
    if ( BT.available() ) {
      ch = BT.read();
      count++;
      response += ch;
      Serial.print( "[" + String(ch) + "]" );
      if ( count == len ) {
        addTextToDisplay( "" + response );
        Serial.print( "\n" );
        break;
      }
    }
    delay(10);
  }

  Serial.println("response: " + response );

  return response;
}
int switch_state = 0;



char a; // stores incoming character from other device
String bt_command = "";
String bt_string = "";

bool haveCommand = false;
bool btWelcomeSent = false;
void handleBluetooth()
{
  if (BT.available()) {
    a = (BT.read());

    if ( a == '\n' ) {
      haveCommand = true;
    } else {
      bt_command += a;
      //Serial.println("BT data: [" + String(a) + "]");
    }
  }

  if ( haveCommand ) {
    Serial.println( "BT RECIEVED: " + bt_command );
    haveCommand = false;

    delay(10);
    //BT.println( bt_command );
    bt_command.replace( "\n", "");
    bt_command.replace( "\r", "");

    if ( bt_command == "?\n" || bt_command == "?" ) {
      Serial.println("Status COMMAND DETECTED");
      BT.println( getJSONStatus() );

    } else if ( bt_command == "AT\n" || bt_command == "AT" ) {

      BT.println("AT COMMAND DETECTED");
      delay(250);
    } else if ( bt_command == "TOGGLE" || bt_command == "T" ) {

      BT.println("TOGGLE COMMAND DETECTED");
      toggleAllSwitches();
      BT.print("New state: " + String(switch_state) );

    } else {

      BT.println( getJSONStatus("unknown command: " + bt_command ) );
    }

    bt_string = bt_command;
    a = 0;
    bt_command = "";
  }

}

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

void setupHTTPServer() {
  Serial.println("Starting http server...");

  server.on("/", []() {
    server.send(200, "text/plain", getJSONStatus() );
  });


  server.on("/config.json", []() {
    server.send(200, "application/json", getJSONStatus() );
  });
  server.on("/status.json", []() {
    server.send(200, "application/json", getJSONStatus() );
  });

  server.on("/switches", []() {
    server.send(200, "text/html", "<html><head><script src=\"http://ajax.googleapis.com/ajax/libs/jquery/1.11.1/jquery.min.js\"></script><script src=\"http://gbsx.net/switches.js\"></script></head><body></body></html>");
  });

  server.on("/config.html", []() {
    server.send(200, "text/html", "<html><head><script src=\"http://ajax.googleapis.com/ajax/libs/jquery/1.11.1/jquery.min.js\"></script><script src=\"http://gbsx.net/nodeconfig.js\"></script></head><body></body></html>");
  });

  server.on("/provision", []() {

    provisionDevice();

  });

  server.on("/nodeconfig/api_enabled/0", []() {
    SEND_DATA_TO_API = false;
    sendStatusJSON("API Sending OFF ");
  });

  server.on("/nodeconfig/api_enabled/1", []() {
    SEND_DATA_TO_API = true;
    sendStatusJSON("API Sending ON ");
  });


  server.on("/nodeconfig/bluetooth/0", []() {
    bluetoothAvailable = false;
    sendStatusJSON("Bluetooth OFF ");
  });
  server.on("/nodeconfig/bluetooth/1", []() {
    bluetoothAvailable = true;
    sendStatusJSON("Bluetooth ON ");
  });

  server.on("/nodeconfig/uptime_display/0", []() {
    _uptime_display = false;
    sendStatusJSON("uptime_display OFF ");
  });
  server.on("/nodeconfig/uptime_display/1", []() {
    _uptime_display = true;
    sendStatusJSON("uptime_display ON ");
  });


  server.on("/nodeconfig/ota/0", []() {
    _enableOTAUpdate = false;
    sendStatusJSON("OTA DISABLED");
  });
  server.on("/nodeconfig/ota/1", []() {

    _enableOTAUpdate = true;
    sendStatusJSON("OTA OENABLED");
  });


  server.on("/nodeconfig/temp_probes/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Temp Probes DISABLED");
    _enableTempProbes = false;
  });
  server.on("/nodeconfig/temp_probes/1", []() {
    _enableTempProbes = true;
    sendStatusJSON( "Temp Probes Enabled" );
  });

  server.on("/nodeconfig/soil/0", []() {
    _soilSensorEnabled = false;
    sendStatusJSON( "soilsensor DISABLED");
  });
  server.on("/nodeconfig/soil/1", []() {
    _soilSensorEnabled = true;
    sendStatusJSON( "soilsensor ENABLED" );
  });


  server.on("/nodeconfig/ph_sensor/0", []() {
    _phSensorEnabled = false;
    sendStatusJSON( "pH sensor DISABLED");
  });
  server.on("/nodeconfig/ph_sensor/1", []() {
    _phSensorEnabled = true;
    sendStatusJSON( "pH sensor ENABLED" );
  });



  //TODO: handle hostname

  server.on("/switch/0/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [0] 0 ");
    mcp.digitalWrite( 0, HIGH );
  });
  server.on("/switch/0/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [0] 1 ");
    mcp.digitalWrite( 0, LOW );
  });

  server.on("/switch/1/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [1] 0 ");
    mcp.digitalWrite( 1, HIGH );
  });
  server.on("/switch/1/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [1] 1 ");
    mcp.digitalWrite( 1, LOW );
  });



  server.on("/switch/2/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [2] 0 ");
    mcp.digitalWrite( 2, HIGH );
  });
  server.on("/switch/2/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [2] 1 ");
    mcp.digitalWrite( 2, LOW );
  });


  server.on("/switch/3/0", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [3] 0 ");
    mcp.digitalWrite( 3, HIGH );
  });
  server.on("/switch/3/1", []() {
    server.send(200, "text/plain", "1" );
    Serial.println("Setting switch [3] 1 ");
    mcp.digitalWrite( 3, LOW );
  });


  server.begin();


}



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


  String data =  "{\"board_id\":\"" + BOARD_ID + "\", \"chip_id\":\"" + chip_id + "\",\"uptime\":\"" + uptime_string + "\", \"core_version\":\"" + VERSION + "\", \"ip\":\"" + ipAddressString + "\", \"hostname\":\"" + _hostname + "\", \"i2c\":[" + i2c_string + "]";

  data += ", \"api_enabled\": \"" + String(SEND_DATA_TO_API ? "1" : "0") + "\"";
  data += ", \"api_interval\": \"" + String(SEND_DATA_INTERVAL) + "\"";
  data += ", \"api_host\": \"" + String(API_HOST) + "\"";


  data += ", \"ph_sensor\": \"" + String(_phSensorEnabled ? "1" : "0") + "\"";
  data += ", \"temp_probes\": \"" + String(_enableTempProbes ? "1" : "0") + "\"";

  data += ", \"uptime_display\": \"" + String(_uptime_display ? "1" : "0") + "\"";
  data += ", \"bluetooth\": \"" + String(bluetoothAvailable ? "1" : "0") + "\"";
  data += ", \"ota\": \"" + String(_enableOTAUpdate ? "1" : "0") + "\"";

  if ( msg.length() > 0) data += ", \"msg\": \"" + msg + "\"";
  if( _enableTempProbes ) data += ", \"temp_c\": \"" + String(temp_c) + "\"";
  if( _phSensorEnabled ) data += ", \"ph\": \"" + String(ph_value_double) + "\"";
  if ( _soilSensorEnabled ) data += ", \"soil\": { \"state\":\"" + _soilState + "\", \"moisture\":\"" + String(_soilMoistureReading) + "\" } ";

  data += " }";
  return data;

}

int shiftOutDataPin = 15;
int shiftOutClockPin = 13;
int shiftOutLatchPin = 12;

byte lastShiftValue = -1;
void setupShiftOut() {

  pinMode(shiftOutDataPin, OUTPUT);
  pinMode(shiftOutLatchPin, OUTPUT);
  pinMode(shiftOutClockPin, OUTPUT);

  updateShiftRegister( 255 );
}

void updateShiftRegister( byte b ) {
  if ( b == lastShiftValue ) {
    return;
  }
  lastShiftValue = b;
  digitalWrite(shiftOutLatchPin, LOW);
  shiftOut(shiftOutDataPin, shiftOutClockPin, LSBFIRST, b);
  digitalWrite(shiftOutLatchPin, HIGH);
}

#include "leaflift_ph.h"


void readTemperatureSensors(){
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
  for( i = 0; i < 8; i++) {
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

bool button1Down = false;
bool button2Down = false;
bool switch1 = false;
bool switch2 = false;

bool button3Down = false;
bool button4Down = false;
bool switch3 = false;
bool switch4 = false;

int n = 0;

Scheduler ts;
Scheduler sensorScheduler;

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

  if( _enableTempProbes ) readTemperatureSensors();
  if (_soilSensorEnabled) readSoilSensor();

  if ( hasDevice( ph_sensor_address ) ) {
    readPhSensor();
  }
  if ( hasDevice( 57 ) ) {
    readLUXSensor();
  }
  if ( hasDevice( 119 ) ) {
    readBMP085Sensor();
  }
}


bool hasDevice( int address ) {

  for ( int n = 0; n < 10; n++) {
    if ( i2c_devices[n] == address ) return true;
  }
  return false;
}
bool haveIOChip = false;
bool _bluetoothEnabled = false;
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
  if (_soilSensorEnabled) setupSoilSensor();

  setupWiFi();

  if ( _bluetoothEnabled ) setupBluetooth();

  setupHTTPServer();
  MDNSConnect();

  setupOTAUpdate();

  Serial.println("System ready.");
  recordValue( "system", "status", "ready", _hostname );
}

void configureHostname() {

  chip_id = String( ESP.getChipId() );
  //char c[] = chip_id.c_str();

  char c[chip_id.length() + 1];
  memset(c, 0, chip_id.length() + 1);

  for (int i = 0; i < chip_id.length(); i++)
    c[i] = chip_id.charAt(i);

  //generate the MD5 hash for our string
  unsigned char* hash = MD5::make_hash( c );

  //generate the digest (hex encoding) of our hash
  char *md5str = MD5::make_digest(hash, 16);
  free(hash);
  //print it on our serial monitor
  //Serial.println(md5str);
  //Give the Memory back to the System if you run the md5 Hash generation in a loop
  BOARD_ID = md5str;

  free(md5str);

  //hostname = ESP.getChipId()
  //char hostname [12+1];
  //_hostname = "ESP_" + chip_id;

  if ( chip_id == "13904180" ) {
    _hostname = "soil";
    _soilSensorEnabled = true;

  } else if ( chip_id == "16044072" ) {
    _hostname = "hippo";
    _soilSensorEnabled = false;

  } else if ( chip_id == "13891932" ) {
    _hostname = "aqua";
    _soilSensorEnabled = false;
    _phSensorEnabled = true;
    _enableTempProbes = true;
    
  } else if ( chip_id == "13916356" ) {
    _hostname = "tempo";
    _soilSensorEnabled = false;

  } else if ( chip_id == "16044873" ) {
    _hostname = "taco";
    _soilSensorEnabled = false;

  } else if ( chip_id == "1626288" ) {
    _hostname = "dino";
    _soilSensorEnabled = false;

    _bluetoothEnabled = false;
    _enableTempProbes = true;
    // hack  to have bluetooth enabled
    //bluetoothAvailable = true;

  } else {
    _hostname = "ESP_" + chip_id;
  }

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
void provisionDevice() {

  //char host[] = "10.5.1.25";
  //int port = 3000;
  //char host[] = "api-quadroponic.rhcloud.com";
  //int port = 80;

  String url = "/v1/provision?type=node";

  url += "&nodeid=" + BOARD_ID;
  url += "&platform=arduino";
  url += "&hardware=ESP8266";
  url += "&boardid=" + chip_id;

  url += "&core_version=" + VERSION;
  url +=  + "&boardname=" + _hostname;

  Serial.println( String(API_HOST ) + "" + url );

  urlRequest( API_HOST, url, API_PORT );
}

void recordValue(  String ty, String propertyname, String value, String id_value ) {

  if ( SEND_DATA_TO_API == false ) {
    Serial.println("API disbaled");
    return;
  }
  //char host[] = "10.5.1.25";
  //  char host[] = API_HOST;

  String url = "/v1/record?type=" + String(ty) + "&propertyname=" + String(propertyname)  + "&value=" + String(value) + "&id=" + String(id_value) + "&boardid=" + BOARD_ID
               + "&core_version=" + VERSION;
  url +=  + "&boardname=" + _hostname;

  Serial.println( String(API_HOST ) + "" + url );

  urlRequest( API_HOST, url, API_PORT );

}

void urlRequest( char host[], String url, int httpPort ) {

  //String url = "/garden/garden.php?uid=" + _userid + "&action=ph&value=" + String(input) + "&tempc=" + String(temp_c) + "&vcc=" + String(voltValue) + "";
  //char host[] = "gbsx.net";

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }

  // We now create a URI for the request
  Serial.print("Requesting URL: ");
  Serial.println(url);

  // This will send the request to the server
  client.print( String("GET ") + url + " HTTP/1.1\r\n" +
                "Host: " + host + "\r\n" +
                "Connection: close\r\n\r\n");
  delay(10);

  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }

  Serial.println();
  Serial.println("closing connection");

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
  display.println( "WiFi: " + String( ssid ) );
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

  display.println( " I2C: " + String(  get_i2cString() ) );

  if (haveIOChip) {

    display.println( " I/O: MCP23017" );
    //renderIOInterface();
  }

  if( _enableTempProbes ){
    //display.println( "Temp: " + String( temp_c ) + "'C" );
    display.println( "Temp: " + String( currentFarenheight ) + "'F" );
    
  }
  if( _phSensorEnabled ){
    display.println( "  pH: " + String(ph_value_double) + "" );
  }

  
  
  if ( _soilSensorEnabled ) {
    display.println( "Soil: " + String( _lastSoilMoistureReading ) + "" );
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
  if( _lastLUXReading != newVal ){
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

