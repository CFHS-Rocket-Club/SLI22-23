#include <Arduino.h>
#include <AccelStepper.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include <Servo.h>

TinyGPSPlus gps;                                   // GPS Driver
RH_RF95 rf95(3, 2);                                // Singleton instance of the radio driver
AccelStepper motor1(AccelStepper::DRIVER, 8, 9);   // Stepper Motors: 8 = pinStep and 9 = pinDirection
AccelStepper motor2(AccelStepper::DRIVER, 14, 15); // Stepper Motors: 14 = pinStep and 15 = pinDirection
Servo servo1;                                      // create servo object #1 to control servo 1
Servo servo2;                                      // create servo object #2 to control servo 2
bool separate = false;                             // Variable used to run different functions when a command is given

void sendposition()
{
    Serial.println("Postrue");
    uint8_t data[24];

    String lat = String("Rlat ") + String(gps.location.lat(), 8);

    for (int i = 0; i < lat.length(); i++)
    {
        data[i] = lat.charAt(i);
    }
    rf95.send(data, lat.length());

    delay(5);

    String lng = String("Rlng ") + String(gps.location.lng(), 8);
    for (int i = 0; i < lng.length(); i++)
    {
        data[i] = lng.charAt(i);
    }
    rf95.send(data, lng.length());

    delay(5);
}

void setup()
{
    Serial.begin(9600);  // Serial Monitor
    Serial1.begin(9600); // GPS hardware serial

    if (!rf95.init()) // Radio Initialization
    {
        Serial.println("Radio init failed");
    }                           // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on...The default transmitter power is 13dBm, using PA_BOOST.
    rf95.setTxPower(20, false); // The default transmitter power is 13dBm, using PA_BOOST. - If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 2 to 20 dBm:

    motor1.setMaxSpeed(500);    // Nosecone Max Speed
    motor1.setAcceleration(100); // Nosecone Accel
    motor2.setMaxSpeed(500);    // Coupler Max Speed
    motor2.setAcceleration(100); // Nosecone Accel
    pinMode(6, OUTPUT);          // M0 - defining M0 pin for stepper motors
    digitalWrite(6, LOW);        // M0 - setting the step pin to low to initialize full step control
    pinMode(7, OUTPUT);          // M1 - defining M1 pin for stepper motors
    digitalWrite(7, LOW);        // M1 - setting the step pin to low to initialize full step control
    pinMode(10, OUTPUT);         // GPS - defining GPS pin to enable / disable
    digitalWrite(10, LOW);       // GPS - setting the GPS pin to low to disable it for the flight
    servo2.attach(18);           // attaches the servo on pin 9 to the servo object
    servo1.attach(19);           // attaches the servo on pin 9 to the servo object
    servo1.write(00);           // Set Servos - Values are from 0 to 180
    servo2.write(180);             // Set Servos - Values are from 0 to 180
}

void loop()
{
    while (Serial1.available() > 0)
    {
        gps.encode(Serial1.read());
    }

    if (rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) // if "separate" message detected
        {
            if (strcmp((char *)buf, "key:s") == 0)
            {
                motor1.moveTo(28000);   // moves stepper motor 1 (Nosecone)
                motor2.moveTo(64000);   // moves stepper motor 2 (Coupler)
                digitalWrite(10, HIGH); // Turns GPS on
                separate = true;        // set separate to true
            }
            if (strcmp((char *)buf, "rocketpos") == 0)
            {
                if (gps.location.isValid() && (gps.location.age() < 10000))
                {
                    sendposition();
                }
            }
            if (strcmp((char *)buf, " ") == 0)
            {
                motor1.stop();
                motor2.stop();
            }
            Serial.println((char *)buf);
        }
    }

    if (separate == true) // if separating
    {
        if ((motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0)) // Checks if Stepper Motors & Time are at their respective endpoints
        {
            separate = false;  // Stops this function from running more than once
            servo1.write(180);   // Set Servos - Values are from 0 to 180
            servo2.write(0); // Set Servos - Values are from 0 to 180
        }
    }

    motor1.run(); // run the stepper
    motor2.run(); // run the stepper
}