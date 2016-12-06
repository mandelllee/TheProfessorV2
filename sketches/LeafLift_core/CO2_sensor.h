#include <NDIR_I2C.h>

NDIR_I2C mySensor(0x4D); //Adaptor's I2C address (7-bit, default: 0x4D)

int CO2Concentration = 0;

void setupCO2Sensor() {
      if (mySensor.begin()) {
        Serial.println("Wait 10 seconds for sensor initialization...");
        delay(10000);
    } else {
        Serial.println("ERROR: Failed to connect to the CO2 sensor.");
        while(1);
    }

}

void readCO2Sensor() {
      if (mySensor.measure()) {
        CO2Concentration = mySensor.ppm;
        Serial.print("CO2 Concentration is ");
        Serial.print(CO2Concentration);
        Serial.println("ppm");
    } else {
        Serial.println("CO2 Sensor communication error.");
    }

}

