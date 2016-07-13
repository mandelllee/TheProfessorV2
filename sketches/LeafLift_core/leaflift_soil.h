

int numSoilSamples = 5;
int _soilMoistureReadings[5] = {0, 0, 0, 0, 0};
int _soilMoistureReadingIndex = 0;

int _lastSoilMoistureReading = 0;
int _soilMoistureReading = 0;
bool _enableTempProbes = false;
String _soilState = "?";


int soilSensorPin1 = 14;
int soilSensorPin2 = 12;
int soilSensorPin3 = 13;
int soilSensorPin4 = 15;


int relayPin1 = 4;
int relayPin2 = 5;
int relayPin3 = 6;
int relayPin4 = 7;


int sensorReadings[] = {0, 0, 0, 0};
bool useIOForSoilSensor = false;

int soilReadFrequency = 30 * 1000;



void setupSoilSensors() {

  //useIOForSoilSensor = hasDevice( 32 );

  if ( useIOForSoilSensor ) {
    mcp.pinMode(soilSensorPin1, OUTPUT);
    mcp.pinMode(soilSensorPin2, OUTPUT);
    mcp.pinMode(soilSensorPin3, OUTPUT);
    mcp.pinMode(soilSensorPin4, OUTPUT);

    mcp.pinMode(relayPin1, OUTPUT);
    mcp.pinMode(relayPin2, OUTPUT);
    mcp.pinMode(relayPin3, OUTPUT);
    mcp.pinMode(relayPin4, OUTPUT);

    mcp.pinMode(relayPin1, HIGH);
    mcp.pinMode(relayPin2, HIGH);
    mcp.pinMode(relayPin3, HIGH);
    mcp.pinMode(relayPin4, HIGH);
    
    mcp.digitalWrite(soilSensorPin1, LOW);
    mcp.digitalWrite(soilSensorPin2, LOW);
    mcp.digitalWrite(soilSensorPin3, LOW);
    mcp.digitalWrite(soilSensorPin4, LOW);

  } else {
    if( soilSensorPin1 != -1 ) pinMode(soilSensorPin1, OUTPUT);
     if( soilSensorPin2 != -1 ) pinMode(soilSensorPin2, OUTPUT);
     if( soilSensorPin3 != -1 ) pinMode(soilSensorPin3, OUTPUT);
     if( soilSensorPin4 != -1 ) pinMode(soilSensorPin4, OUTPUT);
  }
  pinMode(A0, INPUT); //set up analog pin 0 to be input
}


void readSoilSensor( int pin, int index, String label  ) {

  int relayPin = pin + 4;
  
  //Serial.println("POWER ON SOIL SENSOR [" + String(pin) + "]");
  if ( useIOForSoilSensor ) {
    mcp.digitalWrite(relayPin, LOW);    
    mcp.digitalWrite(pin, HIGH);
  } else {
    digitalWrite( pin, HIGH );
  }
  //mcp.digitalWrite(3, HIGH);

  Serial.println("WAIT 1 SECOND....");
  //TODO: make this use a task instead of a delay
  delay(1000);

  int s = analogRead(A0); //take a sample
  delay( 250 );

  sensorReadings[index] = s;
  Serial.println(label + ": [" + String(s) + "] [pin: " + String(pin) + "] " );

  Serial.println("POWER OFF SOIL PIN[" + String(pin) + "]");
  if ( useIOForSoilSensor ) {
    mcp.digitalWrite(pin, LOW);
    mcp.digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite( pin, LOW );
  }
  delay( 250 );

  //recordValue( "environment", "soil", String(_soilMoistureReading), _hostname );

}

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


void readSoilSensors() {

  Serial.println("reading soil sensors...");

  if( soilSensorPin1 != -1 ) readSoilSensor( soilSensorPin1, 0, "1" );
  if( soilSensorPin2 != -1 ) readSoilSensor( soilSensorPin2, 1, "2" );
  if( soilSensorPin3 != -1 ) readSoilSensor( soilSensorPin3, 2, "3" );
  if( soilSensorPin4 != -1 ) readSoilSensor( soilSensorPin4, 3, "4" );

}



//if(_soilSensorEnabled) {

  Task soilSensorInterval( soilReadFrequency, TASK_FOREVER, &readSoilSensors, &sensorScheduler, true);
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

