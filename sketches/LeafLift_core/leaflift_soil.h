
void setupSoilSensor() {
  pinMode(A0, INPUT); //set up analog pin 0 to be input
  mcp.pinMode(3, OUTPUT);
  mcp.digitalWrite(3, LOW);
  //pinMode(10, OUTPUT);

}

int numSoilSamples = 5;
int _soilMoistureReadings[5] = {0, 0, 0, 0, 0};
int _soilMoistureReadingIndex = 0;

int _lastSoilMoistureReading = 0;
int _soilMoistureReading = 0;
bool _soilSensorEnabled = false;
bool _enableTempProbes = false;
String _soilState = "?";

void readSoilSensor() {

  Serial.println("POWER ON SOIL SENSOR MCP[3]");
  //digitalWrite( 10, HIGH );
  mcp.digitalWrite(3, HIGH);

  Serial.println("WAIT 2 SECONDS....");
  //TODO: make this use a task instead of a delay
  delay(2000);

  int s = analogRead(A0); //take a sample
  Serial.println("reading: [" + String(s) + "]" );

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

  Serial.println("POWER OFF SOIL SENSOR MCP[3]");
  //digitalWrite( 10, LOW );
  mcp.digitalWrite(3, LOW);
}
