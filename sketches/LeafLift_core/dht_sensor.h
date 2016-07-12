
#include "DHT.h"

bool _dhtSensorEnabled = true;

#define DHTPIN 10 
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

float dht_temp_f;
float dht_humidity; 
//TODO: is double precise enough?

void setupDHT() {
  dht.begin();
}

void readDHTSensor() {
  
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  delay( 450 );
  
  dht_temp_f = f;
  dht_humidity = h;

  recordValue( "environemnt", "dht_temp_f", String( dht_temp_f ), _hostname );
  recordValue( "environemnt", "dht_humidity", String( dht_humidity ), _hostname );
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);

  Serial.print("{ \"humidity\": \"");
  Serial.print(h);
  Serial.print("%\"");
  Serial.print(",\"temperature\": \"");
  Serial.print(f);
  Serial.print("*F\",\"");
  Serial.print(",\"heat_index\": \"");
  Serial.print(hif);
  Serial.println("*F\"}");
}


