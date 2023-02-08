#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include "Adafruit_BMP3XX.h"

#define BMP_CS 6
#define SEALEVELPRESSURE_HPA (1021.3)

int output = 1000;
uint32_t printTime = 0;

Adafruit_BMP3XX bmp; //bmp390
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

    if (!bmp.begin_SPI(BMP_CS)) // hardware SPI mode
    {
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        while (true);
    }

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    esc.attach(2);

    delay(500);

    Serial.println("Delay Complete");

    
}

void loop()
{
    uint32_t time = micros();
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

    
    if (! bmp.performReading())
    {
        Serial.println("Failed to perform reading :(");
    }

    if (time - printTime > 2000000)
    {
        printTime = time;
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
    }
    
    delay(20);
}