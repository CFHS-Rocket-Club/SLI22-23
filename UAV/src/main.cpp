
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <servo.h>

//#include <I2C_Anything.h>
//#include <Adafruit_Sensor.h>
//#include <Gyro>
//#include <radio>
//#include <gps>
//#include "Adafruit_BMP3XX.h"

  #define BMP_SCK
  #define BMP_MISO
  #define BMP_MOSI
  #define BMP_CS
  #define SEALEVELPRESSURE_HPA (1013.25)
// Adafruit_BMP3XX bmp; //bmp390

int output = 1024;


void setup() 
{

  Serial.begin(57600);
  Serial.println("Initialized");
  Wire.begin();
  /*  
  if(!bmp.begin_I2C()) //Int BMP390 - hardware I2C mode, can pass in address & alt Wire
  { 
    Serial.println("Could not find BMP390 - Error");
    while (1); 
  }
  */
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);
  analogWriteFrequency(5, 250);
  analogWriteResolution(12);
  delay(250); 
}

void loop() 
{
  if (output < 1500)
  {
    output += 1;
  }
  else 
  {
    output = 1024;
  }

  analogWrite(2, output);
  analogWrite(3, output);
  analogWrite(4, output);
  analogWrite(5, output);

  delay(10);
}