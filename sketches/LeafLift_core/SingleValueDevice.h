class SingleValueDevice: public Device {
public:
    int getValues(float *values);
    bool multipleValues();
};

int SingleValueDevice::getValues(float *values){
    return 0;
}

bool SingleValueDevice::multipleValues() {
    return false;
}
