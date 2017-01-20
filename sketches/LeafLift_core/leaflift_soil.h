
String _soilConfigJSON = "";

int numSoilSamples = 5;
int _soilMoistureReadings[5] = {0, 0, 0, 0, 0};
int _soilMoistureReadingIndex = 0;

int _lastSoilMoistureReading = 0;
int _soilMoistureReading = 0;
bool _enableTempProbes = false;
String _soilState = "?";


//int soilSensorPin1 = 14;
//int soilSensorPin2 = 12;
//int soilSensorPin3 = 13;
//int soilSensorPin4 = 15;

int soilSensorPin1 = -1;
int soilSensorPin2 = -1;
int soilSensorPin3 = -1;
int soilSensorPin4 = -1;

int relayPin1 = 4;
int relayPin2 = 5;
int relayPin3 = 6;
int relayPin4 = 7;

int _lastSoilReportTime[] = {0, 0, 0, 0};
int sensorReadings[] = {0, 0, 0, 0};
double sensorReadingsNormal[] = {0,0,0,0,0};

String soilSensorLabel[] = {"soil1", "soil2", "soil3", "soil4"};

bool useIOForSoilSensor = false;

int soilReadFrequency = 10 * 1000;
int soilReportFrequencySeconds = 5 * 60;


void readSoilSensor( int pin, int index, String label  ) {

  int relayPin = pin + 4;
  Serial.println("POWER ON SOIL SENSOR [" + String(pin) + "]");
  if ( useIOForSoilSensor ) {
    //mcp.digitalWrite(relayPin, LOW);
    mcp.digitalWrite(pin, HIGH);
  } else {
    digitalWrite( pin, HIGH );
  }
  Serial.println("WAIT 1 SECOND....");
  //TODO: make this use a task instead of a delay
  delay(1000);
  int s = analogRead(A0); //take a sample
  delay( 250 );
  sensorReadings[index] = s;
  Serial.println(label + ":[pin: " + String(pin) + "]=[" + String(s) + "]" );
  Serial.println("POWER OFF SOIL PIN[" + String(pin) + "]");
  if ( useIOForSoilSensor ) {
    mcp.digitalWrite(pin, LOW);
    //mcp.digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite( pin, LOW );
  }

  // if now is a sane value
  //if( _now > 1468540194 ){
  //TODO: verify that we have a now value, since it will be needed for proper time recording
  if ( true ) {
    int lastTime = _lastSoilReportTime[index];
    Serial.println("lasttime=[" + String(_lastSoilReportTime[index]) + "]" );
    Serial.println("Next report in: [" + String( (_now - lastTime) ) + "] seconds [" + String(soilReportFrequencySeconds) + "]");

    //if it's time to report, then let's go for it.
    if ( (_now - lastTime) > soilReportFrequencySeconds ) {
      Serial.println("\n\n\n- - - - - - - - - - - - - - - - - - - \nTime to report soil value");
      recordValue( "soil", "" + String( soilSensorLabel[index] ), String(s), _hostname );
      _lastSoilReportTime[index] = _now;
    }
  }

  delay( 250 );
}


float normalizeReading( int max_value, int min_value, int reading ){

  Serial.println( "min: " + String(min_value) + " max: " + String(max_value) + " reading: " + String(reading) );
  
  float a = ( reading - min_value );
  float b = ( max_value - min_value );
  Serial.println( "a: " + String(a) + " b: " + String( b ) );
  
  float v = a / b;

  Serial.println( "Normalized: " + String(v) );
  return v;
}


void readSoilSensors() {

  if (!_soilSensorEnabled) return;

  Serial.println("reading soil sensors...");
  if ( hasDevice( 72 ) ) {
    Serial.println("Reading ANALOG from 16bit ADC [ADS1X15]");

    int16_t adc0, adc1, adc2, adc3;

    if ( soilSensorPin1 != -1 ) mcp.digitalWrite(soilSensorPin1, HIGH);
    if ( soilSensorPin2 != -1 ) mcp.digitalWrite(soilSensorPin2, HIGH);
    if ( soilSensorPin3 != -1 ) mcp.digitalWrite(soilSensorPin3, HIGH);
    if ( soilSensorPin4 != -1 ) mcp.digitalWrite(soilSensorPin4, HIGH);

    delay( 2000 );

    if ( soilSensorPin1 != -1 ) {
      adc0 = ads.readADC_SingleEnded(0);
      Serial.print("AIN0: "); Serial.println(adc0);
      sensorReadings[0] = adc0;
       Serial.println("AIN0: "  +String(sensorReadings[0]) );
      sensorReadingsNormal[0] = normalizeReading( wet_calibration[0], dry_calibration[0], adc0 );
     Serial.println("normalized_reading[0]: "  +String( sensorReadingsNormal[0] ) ); 
    }
    if ( soilSensorPin2 != -1 ) {
      adc1 = ads.readADC_SingleEnded(1);
      Serial.print("AIN1: "); Serial.println(adc1);
      sensorReadings[1] = adc1;
    }
    if ( soilSensorPin3 != -1 ) {
      adc2 = ads.readADC_SingleEnded(2);
      Serial.print("AIN2: "); Serial.println(adc2);
      sensorReadings[2] = adc2;
    }
    if ( soilSensorPin4 != -1 ) {
      adc3 = ads.readADC_SingleEnded(3);
      Serial.print("AIN3: "); Serial.println(adc3);
      sensorReadings[3] = adc3;
    }

    if ( useIOForSoilSensor ) {
      if ( soilSensorPin1 != -1 ) mcp.digitalWrite(soilSensorPin1, LOW);
      if ( soilSensorPin2 != -1 ) mcp.digitalWrite(soilSensorPin2, LOW);
      if ( soilSensorPin3 != -1 ) mcp.digitalWrite(soilSensorPin3, LOW);
      if ( soilSensorPin4 != -1 ) mcp.digitalWrite(soilSensorPin4, LOW);
    }
  } else {
    if ( soilSensorPin1 != -1 ) readSoilSensor( soilSensorPin1, 0, "1" );
    if ( soilSensorPin2 != -1 ) readSoilSensor( soilSensorPin2, 1, "2" );
    if ( soilSensorPin3 != -1 ) readSoilSensor( soilSensorPin3, 2, "3" );
    if ( soilSensorPin4 != -1 ) readSoilSensor( soilSensorPin4, 3, "4" );
  }
}



void setupSoilSensors() {
  Serial.println( "setupSoilSensors...");

  //useIOForSoilSensor = hasDevice( 32 );

  if ( useIOForSoilSensor ) {
    mcp.pinMode(soilSensorPin1, OUTPUT);
    mcp.pinMode(soilSensorPin2, OUTPUT);
    mcp.pinMode(soilSensorPin3, OUTPUT);
    mcp.pinMode(soilSensorPin4, OUTPUT);

//    mcp.pinMode(relayPin1, OUTPUT);
//    mcp.pinMode(relayPin2, OUTPUT);
//    mcp.pinMode(relayPin3, OUTPUT);
//    mcp.pinMode(relayPin4, OUTPUT);

//    mcp.pinMode(relayPin1, HIGH);
//    mcp.pinMode(relayPin2, HIGH);
//    mcp.pinMode(relayPin3, HIGH);
//    mcp.pinMode(relayPin4, HIGH);

    mcp.digitalWrite(soilSensorPin1, LOW);
    mcp.digitalWrite(soilSensorPin2, LOW);
    mcp.digitalWrite(soilSensorPin3, LOW);
    mcp.digitalWrite(soilSensorPin4, LOW);

  } else {
    if ( soilSensorPin1 != -1 ) pinMode(soilSensorPin1, OUTPUT);
    if ( soilSensorPin2 != -1 ) pinMode(soilSensorPin2, OUTPUT);
    if ( soilSensorPin3 != -1 ) pinMode(soilSensorPin3, OUTPUT);
    if ( soilSensorPin4 != -1 ) pinMode(soilSensorPin4, OUTPUT);
  }
  pinMode(A0, INPUT); //set up analog pin 0 to be input

  Serial.println( "setupSoilSensors...");


}

Task soilSensorInterval( soilReadFrequency, TASK_FOREVER, &readSoilSensors, &sensorScheduler, true);

//if(_soilSensorEnabled) {

//}
//void setup() {
//
//  Serial.begin( 115200 );
//
//  setupSoilSensors();
//}
//
//void loop() {
//  Serial.println("loop");
//
//  readSoilSensors();
//  int delayTime = 2 * 1000;
//  delay( delayTime );
//}


void interrogate() {

  int s = 0;

  _lastSoilMoistureReading = s;
  _soilMoistureReading = s;

  _soilMoistureReadings[_soilMoistureReadingIndex] = s;
  _soilMoistureReadingIndex++;
  if ( _soilMoistureReadingIndex > numSoilSamples ) _soilMoistureReadingIndex = 0;

  //  bool allGood = true;
  //  while ( allGood ) {
  //    int total = 0;
  //    for (int n = 0; n < numSoilSamples; n++) {
  //
  //      Serial.println("not enough samples to give a value");
  //      int reading = _soilMoistureReadings[_soilMoistureReadingIndex];
  //
  //      if ( reading == 0 ) {
  //        Serial.println("not enough samples to give a value");
  //        allGood = false;
  //      } else {
  //        total += reading;
  //      }
  //    }
  //    Serial.println("We have enough samples to give a value");
  //    _soilMoistureReading = total / numSoilSamples;
  //    allGood = false;
  //  }

  if (s >= 1000) {
    _soilState = "?";
    //Serial.println("Sensor is not in the Soil or DISCONNECTED");

  }
  if (s < 1000 && s >= 600) {
    //Serial.println("Soil is DRY");
    _soilState = "DRY";
  }
  if (s < 600 && s >= 370) {
    //Serial.println("Soil is HUMID");
    _soilState = "HUMID";
  }
  if (s < 370) {
    //Serial.println("Sensor in WATER");
    _soilState = "WET";
  }

}



