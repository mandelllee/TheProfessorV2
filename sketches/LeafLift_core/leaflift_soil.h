

int numSoilSamples = 5;
int _soilMoistureReadings[5] = {0, 0, 0, 0, 0};
int _soilMoistureReadingIndex = 0;

int _lastSoilMoistureReading = 0;
int _soilMoistureReading = 0;
bool _enableTempProbes = false;
String _soilState = "?";


int sensorPin1 = 14;
int sensorPin2 = 12;
int sensorPin3 = 13;
int sensorPin4 = 15;

int sensorReadings[] = {0, 0, 0, 0};
bool useIOForSoilSensor = false;

void setupSoilSensors() {

  useIOForSoilSensor = hasDevice( 32 );

  if ( useIOForSoilSensor ) {

    sensorPin1 = 3;
    sensorPin2 = 2;
    sensorPin3 = 0;
    sensorPin4 = 1;

    mcp.pinMode(sensorPin1, OUTPUT);
    mcp.pinMode(sensorPin2, OUTPUT);
    mcp.pinMode(sensorPin3, OUTPUT);
    mcp.pinMode(sensorPin4, OUTPUT);
    mcp.digitalWrite(sensorPin1, LOW);
    mcp.digitalWrite(sensorPin2, LOW);
    mcp.digitalWrite(sensorPin3, LOW);
    mcp.digitalWrite(sensorPin4, LOW);

  } else {
    pinMode(sensorPin1, OUTPUT);
    pinMode(sensorPin2, OUTPUT);
    pinMode(sensorPin3, OUTPUT);
    pinMode(sensorPin4, OUTPUT);
  }
  pinMode(A0, INPUT); //set up analog pin 0 to be input
}



void readSoilSensor( int pin, int index, String label  ) {


  //Serial.println("POWER ON SOIL SENSOR [" + String(pin) + "]");
  if ( useIOForSoilSensor ) {
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

  readSoilSensor( sensorPin1, 0, "1" );
  readSoilSensor( sensorPin2, 1, "2" );
  readSoilSensor( sensorPin3, 2, "3" );
  readSoilSensor( sensorPin4, 3, "4" );

}
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

