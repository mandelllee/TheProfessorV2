
#define ph_sensor_address 99 //0x63
double currentFarenheight = 0.00;
String ph = "";
float ph_value_float;

char ph_string[16] = "";
char temp_c_string[16] = "";

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

void readPhSensor() {
  //ph_info = sendPhCommand( "I" );
  //ph_cal = sendPhCommand( "Cal,?" );

  //ph_status = sendPhCommand( "Status" );
  //parseStatus( ph_status );

  //  Serial.println("Status: " + ph_status );
  //  Serial.println("Cal: " + ph_cal );
  //  Serial.println("Info: " + ph_info );

  Serial.println("Current temp: " + String(temp_c) );
  if ( temp_c > 0.00 ) {
    char v[8] = "";
    String val = "T," + String( temp_c );
    val.toCharArray( v, 8);

    sendPhCommand( v );
  } else {
    sendPhCommand( "T,25.00" );
  }
  ph = sendPhCommand( "R" );
  float p = ph.toFloat();
  double new_ph_double = roundf( p * 100 ) / 100;

  // is new value different from last ?
  if ( new_ph_double != ph_value_double ) {
    ph_value_double = new_ph_double;
    dtostrf( ph_value_double, 2, 1, ph_string );
    dtostrf( temp_c, 1, 1, temp_c_string );
    updateDisplay();

    recordValue( "water", "ph", String(ph_value_double), _hostname );
    recordValue( "water", "probe_temp_f", String(currentFarenheight), _hostname );
    
    //recordPh( ph_value_double );
  }


}
