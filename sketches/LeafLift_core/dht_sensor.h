
#include "DHT.h"

bool _dhtSensorEnabled = false;

#define DHTPIN 2
#define DHTTYPE DHT22   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

float dht_temp_f;
float dht_humidity;
//TODO: is double precise enough?

void setupDHT() {
  dht.begin();
}

void readDHTSensor() {

  //displayTextOnDisplay( "Reading DHT22 sensor...");

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    recentSensorErrors = true;
//    displayTextOnDisplay( "Failed to read from DHT sensor!");
    delay(1000);
    return;
  } else {

   recentSensorErrors = false;
    if ( h != dht_humidity ) {
      dht_humidity = h;
      recordValue( "environemnt", "dht_humidity", String( dht_humidity ), _hostname );
    }

    if ( f != dht_temp_f ) {
      _lastTempF = dht_temp_f = f;
      // _lastTempF gets overwritten if we have a bmp sensor, which is faster
      recordValue( "environemnt", "dht_temp_f", String( dht_temp_f ), _hostname );
    }
  }

  // Compute heat index in Fahrenheit (the default)
  //  float hif = dht.computeHeatIndex(f, h);
  //
  //  Serial.print("{ \"humidity\": \"");
  //  Serial.print(h);
  //  Serial.print("%\"");
  //  Serial.print(",\"temperature\": \"");
  //  Serial.print(f);
  //  Serial.print("*F\",\"");
  //  Serial.print(",\"heat_index\": \"");
  //  Serial.print(hif);
  //  Serial.println("*F\"}");
}


