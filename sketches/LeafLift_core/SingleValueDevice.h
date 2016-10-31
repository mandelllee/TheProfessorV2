class SingleValueDevice: public Device {
public:
    int getValues(float *values);
    bool multipleValues();
    int totalValues();
};

int SingleValueDevice::getValues(float *values){
    return 0;
}

bool SingleValueDevice::multipleValues() {
    return false;
}

int SingleValueDevice::totalValues() {
  return 1;
}

