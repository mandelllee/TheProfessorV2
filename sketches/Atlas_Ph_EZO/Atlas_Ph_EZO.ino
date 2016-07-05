#include <OneWire.h>
#include <ESP8266mDNS.h>

double temp_c = 0.00;
String powerState = "";
float voltValue = 0.0;
bool promptVisible = false;
int promptTimeout=0;
String promptTitle;
String promptText;
double promptLength=0.00;
String currentDisplayText = "";


// PINS


//I2C 
// SDA =4 ?accurate
// SCL =5 ?accurate

int flowPin1 = 2;

int oneWirePin = 0;


// IDENTITY
//TODO: this should be stired in the EEPROM upon setup.
String _userid = "proto1";



// ---------- START WEB --------------------
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid     = "gbsx";
const char* password = "OrlandoNakazawa!";
String ipAddressString = "";
ESP8266WebServer server(80);

void connectWiFi() {

  displayTextOnDisplay("wiFi:" + String(ssid) + "...");
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
}


void recordPh( double input ) {
  Serial.println("pH Changed: " + String(input) );

  String url = "/garden/garden.php?uid=" + _userid + "&action=ph&value=" + String(input) + "&tempc=" + String(temp_c) + "&vcc=" + String(voltValue) + "";
  char host[] = "gbsx.net";

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
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

// ---------- END WEB --------------------


#include <Wire.h>


//void serialEvent() {
//
//  Serial.println("Serial Event");
//
//  received_from_computer = Serial.readBytesUntil(13, computerdata, 20);
//  computerdata[received_from_computer] = 0;
//  serial_event = 1;
//}




#define ph_sensor_address 99 //0x63

String sendPhCommand( char *command ) {

  char out_data[20];
  byte code = 0;
  byte in_char = 0;
  byte i = 0;
  int time_ = 1800;

  Serial.println("sendPhCommand(" + String(command) + ")...");
  ///addTextToDisplay( "reading pH..." );

  int _time_ = 1800;

  Wire.beginTransmission(ph_sensor_address);
  Wire.write( command );
  Wire.endTransmission();
  delay(_time_);
  Wire.requestFrom(ph_sensor_address, 20, 1);
  code = Wire.read();
  Serial.println("Response:[" + String(code) + "]" );
  switch (code) {
    case 1:
      Serial.println("Success"); break;
    case 2: Serial.println("Failed");
      break;
    case 254: Serial.println("Pending");
      break;
    case 255: Serial.println("No Data");
      break;
  }
  while (Wire.available()) {
    in_char = Wire.read();
    out_data[i] = in_char; i += 1;
    if (in_char == 0) {
      Wire.endTransmission(); break;
    }
  }
  return String( out_data );
}

char volt_string[16] = "";
char ph_string[16] = "";
char temp_c_string[16] = "";

float ph_value_float;
double ph_value_double;
String ph = "";
String ph_status = "";
String ph_info = "";
String ph_cal = "";


void readPhSensor() {
  //ph_info = sendPhCommand( "I" );
  //ph_cal = sendPhCommand( "Cal,?" );

  //ph_status = sendPhCommand( "Status" );
  //parseStatus( ph_status );

//  Serial.println("Status: " + ph_status );
//  Serial.println("Cal: " + ph_cal );
//  Serial.println("Info: " + ph_info );

Serial.println("Current temp: " + String(temp_c) );
if( temp_c > 0.00 ){

    char v[8] = "";
    String val = "T,"+ String( temp_c );
    val.toCharArray( v,8);
    
    sendPhCommand( v );
} else {
  sendPhCommand( "T,25.00" );
}

  ph = sendPhCommand( "R" );

  float p = ph.toFloat();


  double new_ph_double = roundf( p * 10 ) / 10;

  // is new value different from last ?
  if ( new_ph_double != ph_value_double ) {
    ph_value_double = new_ph_double;
    dtostrf( ph_value_double, 2, 1, ph_string );
    dtostrf( temp_c, 1, 1, temp_c_string );
    updateDisplay();
    
    recordPh( ph_value_double );
  }


}


void parseStatus( String input) {

  String firstVal;

  for (int i = 0; i < input.length(); i++) {
    if (input.substring(i, i + 1) == ",") {
      firstVal = input.substring(0, i);
      powerState = input.substring(i + 1);
      voltValue = input.substring(i + 3).toFloat();

      dtostrf( voltValue, 1, 1, volt_string );

      break;
    }

  }
}



#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_STATUS_REQUEST
#include <TaskScheduler.h>
Scheduler ts;
Scheduler ticker;

// Callback methods prototypes
Task tCycle( 10 * 1000, TASK_FOREVER, &SensorReadCallback, &ts, true);
Task tTicker( 1000, TASK_FOREVER, &TickCallback, &ticker, true);


void SensorReadCallback() {
Serial.println("Reading sensors");

  readPhSensor();
  readTemperatureSensors();  
  updateDisplay();
   
}



unsigned long flow1count = 0;

double litersPerMinute = 0.0;
int flowCounter = 0;

#define TICKS_PER_LITER 450.0


void inputHandler()
{
  // total flow
  flow1count +=1;

  flowCounter += 1;

}

void setupInput(){
  pinMode(flowPin1, INPUT_PULLUP);
  attachInterrupt(flowPin1, inputHandler, CHANGE);
}

double currentFarenheight = 0.00;

OneWire  ds(oneWirePin);  //a 2.2K resistor is necessary for 3.3v on the signal line, 4.7k for 5v
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


unsigned long _tickCount=0;

void TickCallback() {

  _tickCount++;
    
  Serial.println( "tick["+String(_tickCount)+"] flow["+String(flowCounter)+"]" );
  
  // if we have counted 60 seconds
  if( _tickCount%15 == 0 ){

    Serial.println("Calculating the flow rate: flowCounter=" + String(flowCounter  ) );
    litersPerMinute = ( flowCounter / TICKS_PER_LITER ) * 4;
    flowCounter = 0;    
    Serial.println("after: flowCounter=" + String(flowCounter  ) );
  }
  
  if( promptVisible ) {
    if( _tickCount > promptTimeout  ){
        // done
        nextCalibrationStep();      
    } else {
      // still ticking...
      updatePrompt();
    }
  } else {
    //Serial.println("prompt not visible");

//    currentDisplayText =  String( "pH   Temp\n")
//    + "" +String( ph_string ) + "  " + String( temp_c_string ) + "c" ;


currentDisplayText = "" +String( currentFarenheight ) + "'F\n"
+ "" +  String( "pH   L/h\n")
+ "" +String( ph_string ) + "  " + String( (int)(litersPerMinute * 60.0) ) + "\n";

  //display.println( "pH: " + String( ph ) + "  t: " + String( _tickCount ) );

    updateDisplay();
    
  }
}







int calibrationStep = 0;

void nextCalibrationStep(){
   promptVisible = false;
   calibrationStep++;
   runPhCalibration();
}
void runPhCalibration()
{
  switch ( calibrationStep ) {
    case (0):
      displayTimedStep("pH Calib", "Prepare wash & testing soltions.", 30 );  
    break;
    case(1):
      displayTimedStep("Prepare", "Remove probe from environment & rinse.", 10 );  
     break;
    case(2):
      displayTimedStep("Midpoint", "Put probe in 7.0pH solution.", 2 *60 );      
    break;
    case(3):
      // submit midpoint calibration
      sendPhCommand("Cal,mid,7.0");
      
      displayTimedStep("Done", "Remove & rinse probe", 10 );  
    break;
    case(4):
      displayTimedStep("Lowpoint", "Put probe in 4.0pH solution.", 2 *60 );
    break;
    case(5):    
      // submit lowpoint calibration
      sendPhCommand("Cal,low,4.0");

      displayTimedStep("Done", "Remove & rinse probe", 10 );  
    break;
    case(6):
      displayTimedStep("Highpoint", "Put probe in 10.0pH solution.", 2 *60);
    break;
    case(7):    
      // submit high calibration
      sendPhCommand("Cal,high,10.0");

      displayTimedStep("Done", "Remove & rinse probe.", 10 );  
    break;
    case(8):

      // record the calibration
      displayTimedStep("Completed", "Return the probe to the environment", 5);
    break;
    default:
      // off the end
    break;
  }

}



#include <SPI.h>            // For SPI comm (needed for not getting compile error)
//#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)
#include <ESP_SSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility

//#define OLED_CS     15  // Pin 19, CS - Chip select
//#define OLED_DC     2   // Pin 20 - DC digital signal
#define OLED_RESET  12  // Pin 15 -RESET digital signal

ESP_SSD1306 display(OLED_RESET);


void initDisplay()
{
  // SSD1306 Init
  display.begin(SSD1306_SWITCHCAPVCC);  // Switch OLED
  display.clearDisplay();
  display.display();
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
  
  if ( promptVisible==true ) return;
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);

//  currentDisplayText =   ;

  
  display.println( String( currentDisplayText ) );

//  display.setCursor(0, 37);
//  display.setTextSize(1);
//  display.println( "flow: " + String( flow1count ) );
//
//  display.setCursor(0, 47);
//  display.setTextSize(1);
//  display.println( "pH: " + String( ph ) + "  t: " + String( _tickCount ) );

  display.setCursor(0, 57);
  display.setTextSize(1);
  display.println( "IP: " + String( ipAddressString )  + "");

  display.display();
}



void displayTimedStep( String title, String text, double num_seconds )
{
  promptVisible = true;
  promptTimeout = _tickCount + num_seconds;
  promptLength = num_seconds;
  promptTitle = title;
  promptText = text;
  updatePrompt();
 
//  int duration_ms = promptLength * 1000;
//  int num_steps = 21;
//  int step_delay = duration_ms / num_steps;
  
//  for ( int n = 0; n < num_steps; n++) {
//    display.print(".");
//    display.display();
//    delay( step_delay );
//  }
//  display.display();
//  delay( 200 );
}

void updatePrompt(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println( String( promptTitle ) );

  display.setTextSize(1);
  display.println( "\n" + String( promptText ) );

  int secondsLeft = promptTimeout - _tickCount;
  
  display.println( "duration: " + String( secondsLeft ) + " sec" );


  
  display.println( "\n" + String( _tickCount ) );
  display.display();
  display.print("\n");
}

void displayStepPrompt( String title, String text )
{

  promptVisible = true;
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println( String( title ) );

  display.setTextSize(1);
  display.println( "\n" + String( text ) );

  display.display();

  delay( 1000 );
  display.print( ".");
  display.display();
  delay( 1000 );
  display.print( ".");
  display.display();
  delay( 1000 );
  display.print( ".");
  display.display();
  delay( 1000 );
  display.print( ".");
  display.display();
  delay( 1000 );

  promptVisible = false;
}


String getJSONStatus(String msg="")
{
  return "{\"ip\":\"" + ipAddressString + "\",\"ph\":\"" + String(ph) + "\",\"flow\":\"" + String( (int)(litersPerMinute * 60.0) ) + "\",\"temp_c\":\"" + String(temp_c) + "\",\"vcc\":\"" + String(voltValue) + "\",\"msg\":\""+msg+"\" }";
}

void setupHTTPServer() {

  server.on("/", []() {
    server.send(200, "text/plain", getJSONStatus() );
  });

  server.on("/calibrate", []() {

    runPhCalibration();
    server.send(200, "text/plain", getJSONStatus("Running calibration") );
  });
  server.begin();

}


void setup() {

  Serial.begin(115200);

  initDisplay();
  displayTextOnDisplay("\nbooting...");

  connectWiFi();

  setupInput();
  
  Wire.begin();
  Serial.println("Started");

  readPhSensor();
  readTemperatureSensors();
    //runPhCalibration();

  setupHTTPServer();

  MDNSConnect();
}

 void MDNSConnect() {  

    String hostname = String( ESP.getChipId() );
    if (!MDNS.begin("aquaruim")) {
      while (1) {
        delay(1000);
      }
    }
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("gbsxnode", "tcp", 80);
  }

void loop() {

  ts.execute();
  server.handleClient();
  ticker.execute();

}


