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
  analogWriteResolution(12);
  delay(1250); 
  analogWrite(2, 1024);
  delay(2000);
  analogWrite(2, 2047);
  delay(2000);
  analogWrite(2, 1024);
  delay(3000);
}

void loop() 
{
 //analogwrite(2,2047);
 
  if (output < 2047)
  {
    output += 1;
  }
  analogWrite(2, output);
}