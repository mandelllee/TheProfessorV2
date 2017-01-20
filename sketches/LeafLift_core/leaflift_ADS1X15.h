#include <Adafruit_ADS1015.h>


Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */


void setupADS1X15(){
	Serial.println("setupADS1X15");
  ads.setGain(GAIN_TWOTHIRDS); 
	ads.begin();
	
}


void readADS1X15(){
//	Serial.println("readADS1X15");
//	int16_t adc0, adc1, adc2, adc3;
//
//	adc0 = ads.readADC_SingleEnded(0);
//	adc1 = ads.readADC_SingleEnded(1);
//	adc2 = ads.readADC_SingleEnded(2);
//	adc3 = ads.readADC_SingleEnded(3);
//	Serial.print("AIN0: "); Serial.println(adc0);
//	Serial.print("AIN1: "); Serial.println(adc1);
//	Serial.print("AIN2: "); Serial.println(adc2);
//	Serial.print("AIN3: "); Serial.println(adc3);
//	Serial.println(" ");
//
//	delay(1000);
}
