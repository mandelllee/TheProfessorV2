class Reading {
  public:
    Reading(String label, float value, String suffix);
    String getLabel();
    float getValue();
    String getSuffix();

  private:
    String _label;
    float _value;
    String _suffix;
};

Reading::Reading(String label, float value, String suffix) {
  _label = label;
  _value = value;
  _suffix = suffix;
};

String Reading::getLabel() {
  return _label;
};

float Reading::getValue() {
  return _value;
};

String Reading::getSuffix() {
  return _suffix;
};


class DigitalSensor {
  public:

    DigitalSensor (uint8_t pin);
    DigitalSensor (uint8_t pin, String type);
    virtual void initialize() = 0;
    virtual void takeReadings();
    virtual boolean getReadings(JsonObject& readings) = 0;

  protected:

    Reading _readings[];
    String _type;
    uint8_t _pin;
    
};

DigitalSensor::DigitalSensor(uint8_t pin){
  _pin = pin;
};

DigitalSensor::DigitalSensor(uint8_t pin, String type){
  _type = type;
  _pin = pin;
};

class DHT_DigitalSensor : public DigitalSensor {

  public:

    DHT_DigitalSensor(uint8_t pin);
    boolean getReadings(JsonObject& readings);
    virtual void initialize();

  protected:

    void takeReadings();
    Reading _readings[];
    DHT _dht(uint8_t pin, uint8_t type);
//    DHT _dht;
    float _dht_temp_f;
    float _dht_humidity;

};

//#define DHTTYPE DHT22   // DHT 11

DHT_DigitalSensor::DHT_DigitalSensor(uint8_t pin) : DigitalSensor(pin){
  _type = DHTTYPE;
   _dht(pin, DHTTYPE);

};

void DHT_DigitalSensor::initialize() {
//   DHT  _dht.readDHTSensor();
}

boolean DHT_DigitalSensor::getReadings(JsonObject& readings) {

  readings["Label"] = "reading 1";
  readings["value"] = 85;
  readings["suffix"] = "F";

  return readings.success();
}

