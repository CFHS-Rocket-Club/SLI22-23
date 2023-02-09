#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM6DSO32.h>

#define BMP_CS 6
#define SEALEVELPRESSURE_HPA (1021.3)

#define LSM_CS 9  // For SPI mode, we need a CS pin

Adafruit_BMP3XX bmp; //bmp390
Adafruit_LSM6DSO32 dso32;
Servo esc;

int output = 1000;
uint32_t printTime = 0;

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

    if (!dso32.begin_SPI(LSM_CS)) {
        Serial.println("Failed to find LSM6DSO32 chip");
        while (true) {}
    }

    dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
    dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);


    esc.attach(2);

    delay(500);

    Serial.println("Delay Complete");

    
}

void loop()
{
    uint32_t time = micros();
    
    if (! bmp.performReading())
    {
        Serial.println("Failed to perform altimeter reading");
    }

    //  /* Get a new normalized sensor event */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    if (!dso32.getEvent(&accel, &gyro, &temp))
    {
        Serial.println("Failed to perform gyro reading");
    }

    if (motorMode == ArmRampUp)
    {
        if (output < 1488)
        {
            output += 1;
        }
        else
        {
            output = 1488 + 68;
            motorMode = Operate;
            Serial.println("Ramp Up Done");
        }
    }

    esc.writeMicroseconds(output);
    
    if (time - printTime > 2000000)
    {
        printTime = time;
        Serial.println();
        Serial.print("Temp: ");
        Serial.print((bmp.temperature * 1.8) + 32);
        Serial.println(" F");
        Serial.print("Pres.: ");
        Serial.print(bmp.pressure / 100.0);
        Serial.println(" hPa");
        Serial.print("Alt: ");
        Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084);
        Serial.println(" ft");

        Serial.print("Temperature ");
        Serial.print((temp.temperature) * 1.8 + 32);
        Serial.println(" F");

        /* Display the results (acceleration is measured in m/s^2) */
        Serial.print("Accel X: ");
        Serial.print(accel.acceleration.x);
        Serial.print("\tY: ");
        Serial.print(accel.acceleration.y);
        Serial.print("\tZ: ");
        Serial.print(accel.acceleration.z);
        Serial.println(" m/s^2");

        /* Display the results (rotation is measured in rad/s) */
        Serial.print("Gyro X: ");
        Serial.print(gyro.gyro.x);
        Serial.print("\tY: ");
        Serial.print(gyro.gyro.y);
        Serial.print("\tZ: ");
        Serial.print(gyro.gyro.z);
        Serial.println(" radians/s ");
    }

    delay(10);
}
