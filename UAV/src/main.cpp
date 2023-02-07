#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

// #include <I2C_Anything.h>
// #include <Adafruit_Sensor.h>
// #include <Gyro>
// #include <radio>
// #include <gps>
// #include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)
// Adafruit_BMP3XX bmp; //bmp390

int output = 1000;

Servo esc;

enum MotorMode
{
    ArmRampUp,
    ArmRampDown,
    Pause,
    Operate,
};

MotorMode motorMode = ArmRampUp;

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
    esc.attach(2);
    delay(500);
    Serial.println("Delay Complete");
}

void loop()
{
    // analogwrite(2,2047);

    if (motorMode == ArmRampUp)
    {
        if (output < 1500)
        {
            output += 1;
        }
        else
        {
            output = 1490;
            motorMode = Operate;
            Serial.println("Ramp Up Done");
        }
    }

    esc.writeMicroseconds(output);

    delay(10);
}