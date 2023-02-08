#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

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

    esc.attach(2);
    delay(500);
    Serial.println("Delay Complete");
}

void loop()
{
    // analogwrite(2,2047);

    if (motorMode == ArmRampUp)
    {
        if (output < 1472)
        {
            output += 1;
        }
        else
        {
            output = 1472;
            motorMode = Operate;
            Serial.println("Ramp Up Done");
        }
    }

    esc.writeMicroseconds(output);

    delay(20);
}