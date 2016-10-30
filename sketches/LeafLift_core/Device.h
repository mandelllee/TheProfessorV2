#include "quantities.h"

class Device {
public:
    virtual const char* getName();
    virtual int getValues(float *values);
    virtual float getValue();
    virtual bool multipleValues();
    static char* getSensorType();
    virtual const int getSensorQuantities(int quantities[]);
};

