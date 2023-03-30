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

Servo escFL;
Servo escFR;
Servo escBL;
Servo escBR;

int output = 1488;

uint32_t printTime = 0;
uint32_t motorTime = 0;
uint32_t armTime = 0;
uint32_t lastTime = 0;

double altitudeSepoint = 0.0;

double velocityX;
double velocityY;
double velocityZ;
double gyroPitch;
double gyroRoll;
double gyroYaw;

enum MotorMode
{
    Arm,
    Disabled,
    Enabled
};

MotorMode motorMode = Arm;

double rollSum;
double rollPrev;
double pitchSum = 0.0;
double pitchPrev = 0.0;
double yawSum;
double yawPrev;
double altSum;
double altPrev;

double pidCalculate(double input, double setpoint, double p, double i, double d, double * const &prevError, double * const &errorSum, double timeDiff)
{
    double error = setpoint - input;

    *errorSum += error * timeDiff;

    double pTerm = p * error;
    double iTerm = i * *errorSum;
    double dTerm = d * (error - *prevError) * timeDiff;

    return pTerm + iTerm + dTerm;
}

void printData(sensors_event_t temp, sensors_event_t accel, sensors_event_t gyro)
{
        Serial.println();
        Serial.print("Temp: ");
        Serial.print((bmp.temperature * 1.8) + 32);
        Serial.println(" F");
        Serial.print("Pres.: ");
        Serial.print(bmp.pressure / 100.0);
        Serial.println(" hPa");
        Serial.print("Alt: ");
        Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084);
        Serial.print(" ft ");
        Serial.println(altitudeSepoint * 3.28084);

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

        Serial.print("Pitch: ");
        Serial.print(gyroPitch);
        Serial.print("\tRoll: ");
        Serial.println(gyroRoll);

        switch (motorMode)
        {
        case Arm:
            Serial.println(output);
            break;

        case Disabled:
            Serial.println("Disabled");
            break;

        case Enabled:
            Serial.println("Enabled");
        
        default:
            break;
        }
        
}

void setMotor(Servo motor, double percentOutput)
{
    // 1488 - 1832
    percentOutput = (percentOutput > 1.0) ? 1.0 : (percentOutput < -1.0) ? -1.0 : percentOutput;
    motor.writeMicroseconds((int) (1488 + percentOutput*69 ) - (percentOutput < 0 ? 30 : 0));
}

void setMotor(Servo motor, double percentOutput, boolean invert)
{
    if (percentOutput < 0.0)
    {
        percentOutput = 0.0;
    }

    if (invert)
    {
        percentOutput *= -1;
    }
    
    setMotor(motor, percentOutput);
}

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

    dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
    dso32.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    dso32.setAccelDataRate(LSM6DS_RATE_52_HZ);
    dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

    escFL.attach(4);
    escFR.attach(5);
    escBL.attach(2);
    escBR.attach(3);

    escFL.writeMicroseconds(output);
    escFR.writeMicroseconds(output);
    escBL.writeMicroseconds(output);
    escBR.writeMicroseconds(output);

    uint32_t time = millis();

    lastTime = time;
    armTime = time;
    printTime = time;

}

void loop()
{
    uint32_t time = micros();

    if (Serial.available())
    {
        char serialInput = Serial.read();
        if (strcmp(&serialInput, "k") == 0)
        {
            if (motorMode == Disabled)
            {
                motorMode = Enabled;
            }

            else if (motorMode == Enabled)
            {
                motorMode = Disabled;
            }
        }

        if (strcmp(&serialInput, " ") == 0)
        {
            if (motorMode != Arm)
            {
                motorMode = Disabled;
            }
        }
    }
    
    if (!bmp.performReading())
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

    gyroRoll=-atan(accel.acceleration.x/sqrt(accel.acceleration.y*accel.acceleration.y+accel.acceleration.z*accel.acceleration.z))*1/(3.142/180);
    gyroPitch=atan(accel.acceleration.y/sqrt(accel.acceleration.x*accel.acceleration.x+accel.acceleration.z*accel.acceleration.z))*1/(3.142/180);

    double alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    if (millis() - armTime > 10000 && motorMode == Arm)
    {
        altitudeSepoint = alt + 1;
        motorMode = Disabled;
    }

    double timeDiff = (double)(time-lastTime) / 1000000.0;

    double pitchOutput = pidCalculate(gyroPitch, 0.0, 0.01, 0.0, 0.0, &pitchPrev, &pitchSum, timeDiff);
    double rollOuput   = pidCalculate(gyroRoll,  0.0, 0.01, 0.0, 0.0, &rollPrev,  &rollSum, timeDiff);
    double yawOutput   = pidCalculate(gyroYaw,   0.0, 0.0, 0.0, 0.0, &yawPrev,   &yawSum, timeDiff);
    double altOutput   = pidCalculate(alt, altitudeSepoint, 0.15, 0.0, 0.0, &altPrev,   &altSum, timeDiff) + 0.3;

    if (motorMode == Enabled)
    {
        setMotor(escFL, altOutput + rollOuput + pitchOutput + yawOutput, false);
        setMotor(escFR, altOutput - rollOuput + pitchOutput - yawOutput, true);
        setMotor(escBL, altOutput + rollOuput - pitchOutput + yawOutput, true);
        setMotor(escBR, altOutput - rollOuput - pitchOutput - yawOutput, false);
    }
    else
    {
        setMotor(escFL, 0.0);
        setMotor(escFR, 0.0);
        setMotor(escBL, 0.0);
        setMotor(escBR, 0.0);
    }
    
    if (time - printTime > 1000000)
    {
        printTime = time;
        printData(temp, accel, gyro);
    }

    lastTime = time;
}
