#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BMP3XX.h"
#define BMP_CS 6
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp; //bmp390

void setup() 
{
  Serial.begin(9600);
  if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() 
{
   if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
bmp.performReading();
    Serial.println();
    Serial.print("Temp: ");
    Serial.print((bmp.temperature * 1.8) + 32);
    Serial.println(" F");
    Serial.println();
    Serial.print("Pres.: ");
    Serial.print(bmp.pressure / 100.0);
    Serial.println(" hPa");
    Serial.println();
    Serial.print("Alt: ");
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084);
    Serial.println(" ft");
    delay(1000); // not necessary - just to slow down readings
}