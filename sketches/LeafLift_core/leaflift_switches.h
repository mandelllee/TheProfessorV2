
bool _useIOForSwitchChannels = true;

String ch1_label = "";
String ch2_label = "";
String ch3_label = "";
String ch4_label = "";
String ch5_label = "";
String ch6_label = "";
String ch7_label = "";
String ch8_label = "";

int ch1_pin = -1;
int ch2_pin = -1;
int ch3_pin = -1;
int ch4_pin = -1;
int ch5_pin = -1;
int ch6_pin = -1;
int ch7_pin = -1;
int ch8_pin = -1;

#define CHANNEL_ON true
#define CHANNEL_OFF false


bool ch1_state = CHANNEL_OFF;
bool ch2_state = CHANNEL_OFF;
bool ch3_state = CHANNEL_ON;
bool ch4_state = CHANNEL_ON;
bool ch5_state = CHANNEL_OFF;
bool ch6_state = CHANNEL_OFF;
bool ch7_state = CHANNEL_OFF;
bool ch8_state = CHANNEL_OFF;

bool channel_states[] = {
  CHANNEL_OFF,

  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,

  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,
  CHANNEL_OFF,

  CHANNEL_OFF,
  CHANNEL_OFF

};


int buttondownstates[] = {false, false, false, false};
int buttonstates[] = {LOW, LOW, LOW, LOW};

int buttonPins[] = {8, 9, 10, 11};
int ledPins[] = {12, 13, 15, 14};

//TODO: these need to map to the actual ones.
int channel_pins[] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
int button_pins[] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
int button_led_pins[] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

int button_pin_state[] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};



void setup_button_pins() {

  for ( int button_num = 1; button_num <= 16; button_num++ ) {

    int button_pin = button_pins[button_num];
    int led_pin = button_led_pins[button_num];

    if ( _useIOForSwitchChannels ) {

     if ( button_pin != -1 ) {
        mcp.pinMode(button_pin, INPUT_PULLUP);
      }
//      if ( led_pin != -1 ) {
//        mcp.pinMode(led_pin, OUTPUT);
//      }
    } else {

      if ( button_pin != -1 ) {
        pinMode(button_pin, INPUT_PULLUP);
      }
      if ( led_pin != -1 ) {
        pinMode(led_pin, OUTPUT);
      }
    }
  }

}

void read_button_pins() {

  for ( int button_num = 1; button_num <= 16; button_num++ ) {

    int button_pin = button_pins[button_num];;
    int led_pin = button_led_pins[button_num];
    if ( button_pin != -1 ) {
      //read the pushbutton value into a variable
      
      int sensorVal;
      
       if ( _useIOForSwitchChannels ) {
           sensorVal = mcp.digitalRead(button_pin);
       } else {
        sensorVal = digitalRead(button_pin);
       }

      //print out the value of the pushbutton
      //Serial.println(sensorVal);

      // Keep in mind the pullup means the pushbutton's
      // logic is inverted. It goes HIGH when it's open,
      // and LOW when it's pressed. Turn on pin 13 when the
      // button's pressed, and off when it's not:


      if (sensorVal == HIGH) {
        //if ( led_pin != -1 ) digitalWrite(led_pin, LOW);
        if ( button_pin_state[button_num] == 1 ) {
          button_pin_state[button_num] = -1;
          apiPOST( "/nodered/button-up", "{ \"button_id\":\"" + String(_hostname) + "-" + String(button_num) + "\", \"state\":\"up\" }" );
          Serial.println( "BUTTON " + String(button_num) + " UP" );
        }
      } else {

        if ( button_pin_state[button_num] == -1 ) {
          button_pin_state[button_num] = 1;
          Serial.println( "BUTTON " + String(button_num) + " DOWN" );

          apiPOST( "/nodered/button-down", "{ \"button_id\":\"" + String(_hostname) + "-" + String(button_num) + "\", \"state\":\"down\" }" );
        }
        //if ( led_pin != -1 ) digitalWrite(led_pin, HIGH);

      }
    }
  }


}

void setChannelState( int channel_num, bool state ) {

  //mcp.digitalWrite( channel_num, state?HIGH:LOW );
  channel_states[channel_num] = state;
  Serial.println("setChannel state: [" + String(channel_num) + "]=[" + String(state) + "]");

}

String numPadStr( int val ) {

  if ( val < 0 ) {
    return String( val );
  }
  if ( val < 10 ) {
    return "0" + String( val );
  }
  return String( val );
}

String getChannelsNode() {

  String _json =  "";
  _json += "      \"channels\": { \n";
  _json += "        \"1\": { \"channel\":\"1\", \"pin\": " + String(channel_pins[1]) + ", \"state\": " + String(channel_states[1] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch1_label + "\" },\n";
  _json += "        \"2\": { \"channel\":\"2\", \"pin\": " + String(channel_pins[2]) + ", \"state\": " + String(channel_states[2] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch2_label + "\" },\n";
  _json += "        \"3\": { \"channel\":\"3\", \"pin\": " + String(channel_pins[3]) + ", \"state\": " + String(channel_states[3] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch3_label + "\" },\n";
  _json += "        \"4\": { \"channel\":\"4\", \"pin\": " + String(channel_pins[4]) + ", \"state\": " + String(channel_states[4] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch4_label + "\" },\n";
  _json += "        \"5\": { \"channel\":\"5\", \"pin\": " + String(channel_pins[5]) + ", \"state\": " + String(channel_states[5] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch5_label + "\" },\n";
  _json += "        \"6\": { \"channel\":\"6\", \"pin\": " + String(channel_pins[6]) + ", \"state\": " + String(channel_states[6] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch6_label + "\" },\n";
  _json += "        \"7\": { \"channel\":\"7\", \"pin\": " + String(channel_pins[7]) + ", \"state\": " + String(channel_states[7] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch7_label + "\" },\n";
  _json += "        \"8\": { \"channel\":\"8\", \"pin\": " + String(channel_pins[8]) + ", \"state\": " + String(channel_states[8] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch8_label + "\" }\n";


  _json += "      }\n";

  return _json;

}
String getChannelStatusJSON() {

  String _json =  "{\n";

  _json += "  \"hostname\":\"" + _hostname + "\"";
  _json += ",\n  \"core_version\":\"" + VERSION + "\"";
  //  if ( msg.length() > 0) _json += ",\n  \"msg\": \"" + msg + "\"";

  _json += ",\n";
  _json += getChannelsNode();

  _json += "  \"now\":\"" + String(_now) + "\",\n";



  _json += "}\n";

  return _json;
}


void updateChannelStates() {

  for ( int button_num = 1; button_num <= 16; button_num++ ) {
    int pin_num = channel_pins[button_num];
    int state = channel_states[button_num];
    if ( pin_num >= 0 ) {

      Serial.println( "ch[" + numPadStr(button_num) + "] state: [" + String(channel_states[button_num]) + "] PIN[" + numPadStr(channel_pins[button_num]) + "] LED[" + numPadStr(ledPins[button_num]) + "]");

      if ( _useIOForSwitchChannels ) {

        mcp.pinMode( pin_num, OUTPUT );
        if ( state == true ) {
          // ON
          //mcp.digitalWrite( ledPins[button_num], HIGH );
          mcp.digitalWrite( pin_num, HIGH );
        } else {
          //OFF
          //mcp.digitalWrite( ledPins[button_num], LOW );
          mcp.digitalWrite( pin_num, LOW );
        }
      } else {
        // using GPIO pins
        if ( state == true ) {

          Serial.println("GPIO PIN: " + String(pin_num) + "] = HIGH" );

          pinMode( pin_num, OUTPUT );
          digitalWrite( pin_num, HIGH );
        } else {

          Serial.println("GPIO PIN: " + String(pin_num ) + "] = LOW" );
          pinMode( pin_num, OUTPUT );
          digitalWrite( pin_num, LOW );
        }


      }
    }
  }
}

void toggleChannelState( int channel_num ) {

  channel_states[channel_num] = !channel_states[channel_num];
  updateChannelStates();

  //
  //    switch ( channel_num ) {
  //    case ( 1 ):
  //      Serial.println("channel");
  //      ch1_state = !ch1_state;
  //      break;
  //    case ( 2 ): Serial.println("channel");
  //      ch2_state = !ch2_state;
  //      break;
  //    case ( 3 ): Serial.println("channel");
  //      ch3_state = !ch3_state;
  //      break;
  //    case ( 4 ): Serial.println("channel");
  //      ch4_state = !ch4_state;
  //      break;
  //    case ( 5 ): Serial.println("channel");
  //      ch5_state = !ch5_state;
  //      break;
  //    case ( 6 ): Serial.println("channel");
  //      ch6_state = !ch6_state;
  //      break;
  //    case ( 7 ): Serial.println("channel");
  //      ch7_state = !ch7_state;
  //      break;
  //    case ( 8 ): Serial.println("channel");
  //      ch8_state = !ch8_state;
  //      break;
  //  }
}




void handleButtonDown( int button_num ) {

  Serial.println("Button[" + String(button_num) + "] DOWN");
  mcp.digitalWrite( ledPins[button_num], HIGH );
}

void handleButtonUp( int button_num ) {

  buttonstates[button_num] = !buttonstates[button_num];

  toggleChannelState( button_num );

  if ( channel_states[button_num] == true ) {
    // ON
    mcp.digitalWrite( ledPins[button_num], HIGH );
    mcp.digitalWrite( channel_pins[button_num], LOW );
  } else {
    //OFF
    mcp.digitalWrite( ledPins[button_num], LOW );
    mcp.digitalWrite( channel_pins[button_num], HIGH );
  }
}



void handleButtons() {

  for ( int n = 0; n < 4; n++ ) {
    int state = mcp.digitalRead(buttonPins[n]);

    if ( LOW == state ) {
      buttondownstates[n] = true;
      handleButtonDown( n );
    } else {
      // UP
      if ( buttondownstates[n] == true ) {
        //Serial.println("Button[" + String(n) + "] UP");
        handleButtonUp( n );
        // toggle
        buttondownstates[n] = false;
      }
    }
    //mcp.digitalWrite( ledPins[n], buttonstates[n] );
  }
}



bool getChannelState( int channel_num ) {

  switch ( channel_num ) {
    case ( 1 ):
      Serial.println("channel");
      return ch1_state;
      break;
    case ( 2 ): Serial.println("channel");
      return ch2_state;
      break;
    case ( 3 ): Serial.println("channel");
      return ch3_state;
      break;
    case ( 4 ): Serial.println("channel");
      return ch4_state;
      break;
    case ( 5 ): Serial.println("channel");
      return ch5_state;
      break;
    case ( 6 ): Serial.println("channel");
      return ch6_state;
      break;
    case ( 7 ): Serial.println("channel");
      return ch7_state;
      break;
    case ( 8 ): Serial.println("channel");
      return ch8_state;
      break;
  }
}
String getSwitchJSON() {

  String _json =  "{\n";

  _json += "  \"hostname\":\"" + _hostname + "\"";
  _json += ",\n  \"core_version\":\"" + VERSION + "\"";
  //  if ( msg.length() > 0) _json += ",\n  \"msg\": \"" + msg + "\"";
  _json += ",\n";

  _json += "  \"now\":\"" + String(_now) + "\",\n";
  _json += "  \"switches\": [\n";

  _json += "    {\n";
  _json += "      \"controller\":\"" + String(_useIOForSwitchChannels == true ? "mcp23017" : "gpio") + "\",\n";

  _json += "      \"channels\": { \n";
  _json += "        \"1\": { \"channel\":\"1\", \"pin\": " + String(channel_pins[0]) + ", \"state\": " + String(channel_states[0] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch1_label + "\" },\n";
  _json += "        \"2\": { \"channel\":\"2\", \"pin\": " + String(channel_pins[1]) + ", \"state\": " + String(channel_states[1] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch2_label + "\" },\n";
  _json += "        \"3\": { \"channel\":\"3\", \"pin\": " + String(channel_pins[2]) + ", \"state\": " + String(channel_states[2] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch3_label + "\" },\n";
  _json += "        \"4\": { \"channel\":\"4\", \"pin\": " + String(channel_pins[3]) + ", \"state\": " + String(channel_states[3] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch4_label + "\" },\n";
  _json += "        \"5\": { \"channel\":\"5\", \"pin\": " + String(ch5_pin) + ", \"state\": " + String(channel_states[5] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch5_label + "\" },\n";
  _json += "        \"6\": { \"channel\":\"6\", \"pin\": " + String(ch6_pin) + ", \"state\": " + String(channel_states[6] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch6_label + "\" },\n";
  _json += "        \"7\": { \"channel\":\"7\", \"pin\": " + String(ch7_pin) + ", \"state\": " + String(channel_states[7] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch7_label + "\" },\n";
  _json += "        \"8\": { \"channel\":\"8\", \"pin\": " + String(ch8_pin) + ", \"state\": " + String(channel_states[8] == CHANNEL_ON ? "1" : "0") + ", \"label\":\"" + ch8_label + "\" }\n";
  _json += "      }\n";

  _json += "    }\n";

  _json += "  ]\n";
  _json += "}\n";

  return _json;
}
