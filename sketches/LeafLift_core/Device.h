#include "quantities.h"

class Device {
public:
    virtual const char* getName();
    virtual int getValues(float *values);
    virtual float getValue();
    virtual JsonObject& getValuesJson();
    virtual bool multipleValues();
    static char* getSensorType();
    virtual const int getSensorQuantities(String quantities[]);
    virtual int totalValues();
};

