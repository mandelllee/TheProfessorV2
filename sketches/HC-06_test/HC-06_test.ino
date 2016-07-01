#include <Wire.h>
#include "Adafruit_MCP23017.h"

#include <SoftwareSerial.h>
SoftwareSerial BT(14,12); 
// creates a "virtual" serial port/UART
// connect BT module TX to D10
// connect BT module RX to D11
// connect BT Vcc to 5V, GND to GND

Adafruit_MCP23017 mcp;
void setup()  
{
  Serial.begin(115200);
  mcp.begin();      // use default address 0
  mcp.pinMode(0, OUTPUT);
  mcp.digitalWrite(0, LOW);
  Serial.println("Starting BT");
  delay(2000);
  mcp.digitalWrite(0, HIGH);
 delay(200);
  mcp.digitalWrite(0, LOW);
 delay(200);
  mcp.digitalWrite(0, HIGH);
  delay(200);
  mcp.digitalWrite(0, LOW);
 delay(200);
  mcp.digitalWrite(0, HIGH);
 delay(200);
  mcp.digitalWrite(0, LOW);


  Serial.println("GO!");
  
  // set digital pin to control as an output
  //pinMode(13, OUTPUT);
  
  // set the data rate for the SoftwareSerial port
  BT.begin(9600);
  // Send test message to other device
  BT.println("Hello from Arduino");
}
char a; // stores incoming character from other device
void loop() 
{
  if (BT.available())
  // if text arrived in from BT serial...
  {
    a=(BT.read());
    Serial.println("BT data: [" + String(a) + "]");
    if (a=='1')
    {
      mcp.digitalWrite(0, HIGH);
      BT.println("LED on");
    }
    if (a=='2')
    {
      mcp.digitalWrite(0, LOW);
      BT.println("LED off");
    }
    if (a=='?')
    {
      BT.println("Send '1' to turn LED on");
      BT.println("Send '2' to turn LED on");
    }   
    // you can add more "if" statements with other characters to add more commands
  }
}
