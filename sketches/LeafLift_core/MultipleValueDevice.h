#include "Device.h"

class MultipleValueDevice: public Device {
public:
    float getValue();
    bool multipleValues();
    virtual int totalValues();
};

float MultipleValueDevice::getValue(){
    return 0;
}

bool MultipleValueDevice::multipleValues() {
    return true;
}

