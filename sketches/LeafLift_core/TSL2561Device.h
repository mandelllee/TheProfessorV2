#include "SingleValueDevice.h"
#include <Adafruit_TSL2561_U.h>


class TSL2561Device: public SingleValueDevice{
private:
    Adafruit_TSL2561_Unified *sensor_;

public:
    TSL2561Device();
    const char* getName();
    float getValue();
    const char* getSensorType();
    const int getSensorQuantities(int quantities[]);
    static TSL2561Device* fromConfig(void *);
};

TSL2561Device::TSL2561Device(){
    sensor_ = new Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
//    sensor_->begin();

        /* You can also manually set the gain or enable auto-gain support */
      // sensor_->setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
      // sensor_->setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
      sensor_->enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
    
      /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
//      sensor_->setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
      // sensor_->setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
      // sensor_->setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
    
      /* Update these values depending on what you've set above! */
//      Serial.println("------------------------------------");
//      Serial.print  ("Gain:         "); Serial.println("Auto");
//      Serial.print  ("Timing:       "); Serial.println("13 ms");
//      Serial.println("------------------------------------");
    
}

const char * TSL2561Device::getName(){
    return "TSL2561";
}

const char * TSL2561Device::getSensorType(){
    return "TSL2561";
}

const int TSL2561Device::getSensorQuantities(int quantities[]){
    quantities[0] = ILLUMINANCE;
    return 1;
}

float TSL2561Device::getValue() {
  sensors_event_t event;
  sensor_->getEvent(&event);
  return event.light;
}

TSL2561Device * TSL2561Device::fromConfig(void *){
  return new TSL2561Device();
}

