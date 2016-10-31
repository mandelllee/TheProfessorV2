#include "MultipleValueDevice.h"
#include <DHT_U.h>

class DHTDevice: public MultipleValueDevice {
private:
    char name_[17];
    DHT_Unified *sensor_;
    int pin_;
public:
    DHTDevice(int pin, int dhtType);
    const char* getName();
    int totalValues();
    int getValues(float *values);
    JsonObject& getValuesJson();
    const char* getSensorType();
    const int getSensorQuantities(String quantities[]);
};

DHTDevice::DHTDevice(int pin, int dhtType){
    pin_ = pin;
    sensor_ = new DHT_Unified(pin, dhtType);
    sensor_->begin();
    sprintf(name_, "dht%d_%d",dhtType, pin);

}

const char * DHTDevice::getName(){
    return name_;
}

const char * DHTDevice::getSensorType(){
    return "DHT";
}

const int DHTDevice::getSensorQuantities(String quantities[]){
    quantities[0] = quantities_table[TEMPERATURE];
    quantities[1] = quantities_table[HUMIDITY];
    return 2;
}

int DHTDevice::getValues(float *values){
    sensors_event_t event;
    sensor_->temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        values[0] = -1;
    }
    else {
        values[0] = event.temperature;
    }

    sensor_->humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        values[1] = -1;
    }
    else {
        values[1] = event.relative_humidity;
    }

    return 2;
}

JsonObject& DHTDevice::getValuesJson() {
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& jsonWrapper = jsonBuffer.createObject();
  JsonObject& data = jsonWrapper.createNestedObject(getName());

    sensors_event_t event;
    sensor_->temperature().getEvent(&event);
   if (isnan(event.temperature)) {
        data["temperature"] = -1;
    }
    else {
        data["temperature"] = event.temperature;
    }
    sensor_->humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        data["humidity"] = -1;
    }
    else {
        data["humidity"] = event.relative_humidity;
    }
  return jsonWrapper;

}

int DHTDevice::totalValues() {
    return 2;
}
