
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <servo.h>
/*
#include <I2C_Anything.h>
#include <Adafruit_Sensor.h>
#include <Gyro>
#include <radio>
#include <gps>
#include "Adafruit_BMP3XX.h"

  #define BMP_SCK
  #define BMP_MISO
  #define BMP_MOSI
  #define BMP_CS
  #define SEALEVELPRESSURE_HPA (1013.25)
// Adafruit_BMP3XX bmp; //bmp390

int Led = 13;
int DELAY = 1000;

Servo motor1;



void setup() 
  {

    Serial.begin(115200);
    Serial.println("Initialized");
    Wire.begin();
    pinMode(Led, OUTPUT);
    motor1.attach(2);
    
    if(!bmp.begin_I2C()) //Int BMP390
    { //hardware I2C mode, can pass in address & alt Wire
      Serial.println("Could not find BMP390 - Error")
      while (1);
    }
    

  // put your setup code here, to run once:
  analogWrite(2, 127);
  delay(250);
}

void loop() {
  // put your main code here, to run repeatedly:
//analogWriteFrequency(2, 25); //values 1 through 256
  //delay(4500);
  //digitalWrite(LED_BUILTIN, HIGH);
  //analogWriteFrequency(2, 25);
  //delay(500);
  //digitalWrite(LED_BUILTIN, LOW);


//motor1.writeMicroseconds(1500);
analogWrite(2, 1500);
delay(250);

}
*/


/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  5 October 2022 -  initial release
*/

#include <PulsePosition.h>
PulsePositionInput ReceiverInput(RISING);

float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber=0; 
float InputThrottle;

void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
  for (int i=1; i<=ChannelNumber;i++){
    ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}
void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT); 
  digitalWrite(13, HIGH);
  ReceiverInput.begin(14);
  analogWriteFrequency(1, 250);
  analogWriteResolution(12);
  delay(250);
  while (ReceiverValue[2] < 1020 ||
    ReceiverValue[2] > 1050) {
      read_receiver();
      delay(4);
    }
}
void loop() {
  read_receiver();
  InputThrottle=ReceiverValue[2];
  analogWrite(2, 1.024*InputThrottle);
}