#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_LSM6DSO32.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPSPlus.h>
#include <SimpleKalmanFilter.h>
//#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

boolean auto_level = true;                 //Auto level on (true) or off (false)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






RH_RF95 rf95(10, 7); // Singleton instance of the radio driver

SimpleKalmanFilter simpleKalmanFilter(1, 1, 1);
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

//#define BMP_CS 6
//#define SEALEVELPRESSURE_HPA (1021.3)

#define LSM_CS 9 // For SPI mode, we need a CS pin

//Adafruit_BMP3XX bmp; // bmp390
Adafruit_LSM6DSO32 dso32;

Servo escFL;
Servo escFR;
Servo escBL;
Servo escBR;

int output = 1488;

uint32_t printTime;
uint32_t motorTime;
uint32_t armTime;
uint32_t lastTime;

TinyGPSPlus gps;

double altitudeSetpoint = 0.0;
double pitchSetpoint = 0.0;
double rollSetpoint = 0.0;

double targetLat = 0.0;
double targetLng = 0.0;
double targetHeading = 0.0;

double velocityX;
double velocityY;
double velocityZ;
double gyroPitch;
double gyroRoll;
double gyroYaw;
bool ReadySwitch = false;
double currentvalue = 0.00;
double maxvalue = 0.00;
double gravity = 10.201;

enum MotorMode
{
    Arm,
    Disabled,
    Hold,
    NavigateHands,
    NavigateRocket,
    Land,
    Starting
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
double velClimb = 0;
double ax = 0;
double az = 0;
double vx = 0;
double vz = 0;

     double PitchPValue = 0.0013; 
    double PitchIValue = 0.00001;
     double PitchDValue = 0.0000075; 
//double pitchOutput = pidCalculate(gyroPitch, pitchSetpoint, (0.6*PUltimate), (1.2*(PUltimate/PPeriod)), ((3*PUltimate*PPeriod)/40), &pitchPrev, &pitchSum, 1, timeDiff);
    
     double RUltimate = 0; //0.01 //1
     double RPeriod = 0;   //1.35
//double rollOuput   = pidCalculate(gyroRoll,  rollSetpoint,  (0.6*RUltimate), (1.2*(RUltimate/RPeriod)), ((3*RUltimate*RPeriod)/40), &rollPrev,  &rollSum,  1, timeDiff);

     double YUltimate = 0.0017;
     double YPeriod = 2.5;
//double yawOutput   = pidCalculate(gyroYaw,   0.0,   (0.6*YUltimate), (1.2*(YUltimate/YPeriod)), ((3*YUltimate*YPeriod)/40),    &yawPrev,   &yawSum,   1, timeDiff);

     double AUltimate = 1;
     double APeriod = 1;

     double constanthovering = 0.7;


double pidCalculate(double input, double setpoint, double p, double i, double d, double* const &prevError, double* const &errorSum, double outputRange, double timeDiff)
{
    double error = setpoint - input;

    *errorSum += error * timeDiff;

    if (i != 0.0)
    { 
        double maxSum = outputRange * p / i;
        *errorSum = (*errorSum < -maxSum) ? -maxSum : (*errorSum > maxSum) ? maxSum : *errorSum;
    }

    double pTerm = p * error; //Remained the same
    double iTerm = i * *errorSum; // i * *errorSum;
    double dTerm = d * (error - *prevError) / timeDiff; //used to be d * (error - *prevError) * timeDiff);
    //Serial.println(error);
    //Serial.println(error-*prevError);

    double output = pTerm + iTerm + dTerm;
    *prevError = error;

    if (output < -outputRange && outputRange >= 0.0)
    {
        output = -outputRange;
    }

    if (output > outputRange && outputRange >= 0.0)
    {
        output = outputRange;
    }
   
    return output;
}

void printData(sensors_event_t temp, sensors_event_t accel, sensors_event_t gyro)
{
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
    Serial.print("\tYaw: ");
    Serial.println(gyroYaw);

    Serial.println(gps.location.lat(), 8);
    Serial.println(gps.location.lng(), 8);
    Serial.println(gps.satellites.value());

    switch (motorMode)
    {
    case Arm:
        Serial.println(output);
        break;

    case Disabled:
        Serial.println("Disabled");
        break;

    case Hold:
        Serial.println("Hold");
        break;

    case NavigateHands:
        Serial.println("Navigate");
        break;

    case Land:
        Serial.println("Land");
        break;

    case Starting:
        Serial.println("Starting");
        break;

    default:
        break;
    }
}

void setMotor(Servo motor, double percentOutput)
{
    // 1488 - 1832
    percentOutput = (percentOutput > 1.0) ? 1.0 : (percentOutput < -1.0) ? -1.0
                                                                         : percentOutput;
    motor.writeMicroseconds((int)(1488 + percentOutput * 300) - (percentOutput < 0 ? 30 : 0));
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void setup(){
  //Serial.begin(57600);
  //Copy the EEPROM data for fast access data.
  for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  start = 0;                                                                //Set start back to zero.
  gyro_address = eeprom_data[32];                                           //Store the gyro address in the variable.

  Wire.begin();                                                             //Start the I2C as master.

  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz.

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                                                        //Configure digital poort 12 and 13 as output.

  //Use the led on the Arduino for startup indication.
  digitalWrite(12,HIGH);                                                    //Turn on the warning led.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program  
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  set_gyro_registers();                                                     //Set the specific gyro registers.

  for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Take 2000 readings for calibration.
    if(cal_int % 15 == 0)digitalWrite(12, !digitalRead(12));                //Change the led status to indicate calibration.
    gyro_signalen();                                                        //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if(start == 125){                                                       //Every 125 loops (500ms).
      digitalWrite(12, !digitalRead(12));                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0;                                                                //Set start back to 0.

  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();                                                    //Set the timer for the next loop.

  //When everything is done, turn off the led.
  digitalWrite(12,LOW);                                                     //Turn off the warning led.
}
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(57600);
    Serial.println("Initialized");

    Serial1.begin(9600);

    if (!rf95.init())
        Serial.println("Radio init failed");

    if (!dso32.begin_SPI(LSM_CS))
    {
        Serial.println("Failed to find LSM6DSO32 chip");
        while (true)
        {
        }
    }

    dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_4_G);
    dso32.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    dso32.setAccelDataRate(LSM6DS_RATE_52_HZ);
    dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

    escFL.attach(2);
    escFR.attach(3);
    escBL.attach(4);
    escBR.attach(5);

    escFL.writeMicroseconds(output);
    escFR.writeMicroseconds(output);
    escBL.writeMicroseconds(output);
    escBR.writeMicroseconds(output);

    armTime = millis();
    printTime = micros();
    lastTime = micros();
}

void loop()
{
    uint32_t time = micros();
    double timeDiff = (double)(time - lastTime) / 1000000.0;
    lastTime = time;

    pitchSetpoint /= 1.008;
    rollSetpoint /= 1.008;

  float velkalmon = velClimb;
  float estimated_velocity = simpleKalmanFilter.updateEstimate(velkalmon);


    if (rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) // if "separate" message detected
        {
            if (strcmp((char *)buf, "key:k") == 0)
            {
                if (motorMode != Arm && motorMode != Disabled)
                {
                    motorMode = Land;
                    currentvalue = 0;
                }
                else if (motorMode == Disabled && motorMode != Land)
                {
                    motorMode = Starting;
                }
            }

/* MotorModes
    Arm,
    Disabled,
    Hold,
    NavigateHands,
    NavigateRocket,
    Land,
    Starting
*/

//double altOutput   = pidCalculate(alt, altitudeSetpoint,    (0.6*AUltimate), (1.2*(AUltimate/APeriod)), ((3*AUltimate*APeriod)/40),    &altPrev,   &altSum,   0.0, timeDiff) + 0.565;

            if (strcmp((char *)buf, "key: ") == 0)
            {
                motorMode = Disabled;
                currentvalue = 0;
                maxvalue = 0;
            }

            if (strcmp((char *)buf, "key:r") == 0)
            {
               PitchIValue += 0.00001;
             uint8_t data[24];

        String Ahovering = String("PitchI") + String(PitchIValue, 8);
        for (int i = 0; i < Ahovering.length(); i++)
        {
            data[i] = Ahovering.charAt(i);
        }
        rf95.send(data, Ahovering.length());
            }            

            if (strcmp((char *)buf, "key:f") == 0)
            {
                PitchIValue -= 0.00001;
             uint8_t data[24];

        String Ahovering = String("PitchI ") + String(PitchIValue, 8);
        for (int i = 0; i < Ahovering.length(); i++)
        {
            data[i] = Ahovering.charAt(i);
        }
        rf95.send(data, Ahovering.length());
            }   
            

            if (strcmp((char *)buf, "key:w") == 0)
            {
                PitchPValue += 0.0001;
                  uint8_t data[24];

        String Ultimate = String("PitchP ") + String(PitchPValue, 8);
        for (int i = 0; i < Ultimate.length(); i++)
        {
            data[i] = Ultimate.charAt(i);
        }
        rf95.send(data, Ultimate.length());
            }

            if (strcmp((char *)buf, "key:s") == 0)
            {
                PitchPValue -= 0.0001;
                  uint8_t data[24];

        String Ultimate = String("PitchP ") + String(PitchPValue, 8);
        for (int i = 0; i < Ultimate.length(); i++)
        {
            data[i] = Ultimate.charAt(i);
        }
        rf95.send(data, Ultimate.length());
            }


            if (strcmp((char *)buf, "key:e") == 0)
            {
                PitchDValue += 0.0001;
                  uint8_t data[24];

        String Period = String("PitchD ") + String(PitchDValue, 8);
        for (int i = 0; i < Period.length(); i++)
        {
            data[i] = Period.charAt(i);
        }
        rf95.send(data, Period.length());
            }

            if (strcmp((char *)buf, "key:d") == 0)
            {
                PitchDValue -= 0.0001;
             uint8_t data[24];

        String Period = String("PitchD ") + String(PitchDValue, 8);
        for (int i = 0; i < Period.length(); i++)
        {
            data[i] = Period.charAt(i);
        }
        rf95.send(data, Period.length());
            }                     
        }
    }

    while (Serial1.available() > 0)
    {
        gps.encode(Serial1.read());
    }

    //  /* Get a new normalized sensor event */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    if (!dso32.getEvent(&accel, &gyro, &temp))
    {
        Serial.println("Failed to perform gyro reading");
    }

    gyroRoll = (-atan(accel.acceleration.x / sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 1 / (3.142 / 180)) - 0.66;
    gyroPitch = (atan(accel.acceleration.y / sqrt(accel.acceleration.x * accel.acceleration.x + accel.acceleration.z * accel.acceleration.z)) * 1 / (3.142 / 180)) - 1.78;
    gyroYaw += (gyro.gyro.z + 0.008) * timeDiff / (3.142 / 180);
    
    /*
    ax = ((accel.acceleration.x-0.07)-(gravity*sin(gyroPitch-2)));
    vx = (ax*timeDiff);
    az = (accel.acceleration.z-(gravity*cos(gyroPitch-2)));;
    vz = (az*timeDiff);
    velClimb = ((vz*cos(gyroPitch-2))-(vx*sin(gyroPitch-2)));
*/

    //double alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    if (motorMode == Arm && millis() - armTime > 10000)
    {
       // altitudeSetpoint = alt - 18; // meter
        motorMode = Disabled;
    }

    if (motorMode != Arm && accel.acceleration.z < 0.0)
    {
        motorMode = Disabled;

    }


    /*  Ziegler-Nichols PID Tuning Method
        U = P value when starting to go unstable (Oscillating)
        T = Oscillation period
        Equation : 
        P = 0.6 * U
        I = 1.2 * (U/T)
        D = (3*U*T) / 40

        PITCH U=0.0012 because P oscillated then during testing
        T = ~1 Second   
   
   
   
   //(0.6*AUltimate), (1.2*(AUltimate/APeriod)), ((3*AUltimate*APeriod)/40)
   
    */
        // double pitchOutput = pidCalculate(gyroPitch, pitchSetpoint, (0.2*PitchPValue), ((0.4*PitchPValue)/0.5), (0.066*0.5*PitchPValue), &pitchPrev, &pitchSum, 1, timeDiff);
    //                                current,   sp,            p,      i,        d,       prev,        sum,     range,  dt
    //double pitchOutput = pidCalculate(gyroPitch, pitchSetpoint, PitchPValue, PitchIValue, PitchDValue, &pitchPrev, &pitchSum, 1, timeDiff);//p = 0.005      d   = 0.000006
    // double rollOuput   = pidCalculate(gyroRoll,  rollSetpoint,  0.000, 0.00, 0, &rollPrev,  &rollSum,  0, timeDiff); //p = 0.005, d = 0.000003
    double pitchOutput = pidCalculate(gyroPitch, pitchSetpoint, PitchPValue, PitchIValue, PitchDValue, &pitchPrev, &pitchSum, 1, timeDiff);//p = 0.005      d   = 0.000006
    double rollOuput   = pidCalculate(gyroRoll,  rollSetpoint,  0.00, 0.00, 0.00003, &rollPrev,  &rollSum,  0, timeDiff); //p = 0.005, d = 0.000003
    double yawOutput   = pidCalculate(gyroYaw,   0.000,   0, 00, 0,    &yawPrev,   &yawSum,   0, timeDiff);
    double altOutput   = pidCalculate(estimated_velocity, 0.0, 0, 0, 0,    &altPrev,   &altSum,   0, timeDiff) + constanthovering; //0.565 is hover speed -- 0.46 is sit on ground and spin speed
    Serial.println(pitchOutput);        //velClimb or estimated_velocity
  //  Serial.println(alt);
    double FLout = altOutput + rollOuput + pitchOutput + yawOutput;
    double FRout = altOutput - rollOuput + pitchOutput - yawOutput;
    double BLout = altOutput + rollOuput - pitchOutput - yawOutput;
    double BRout = altOutput - rollOuput - pitchOutput + yawOutput;


    double Wingup = 0.33;
   
  //  if ((alt >= (altitudeSetpoint + 0.5)) && motorMode == Hold) 
  //  {
  //    setMotor(escFL, 0.5, true);
  //    setMotor(escFR, 0.5, false);
  //    setMotor(escBL, 0.5, false);
  //    setMotor(escBR, 0.5, true);  
  //    Serial.println("DROPPING");
  //  }
// && (alt <= (altitudeSetpoint + 0.5)
    if ((motorMode == Hold))
    { 
        //         IF PID greater than wingup true : false    && FLout < maxvalue
        setMotor(escFL, (FLout > Wingup ? (FLout < maxvalue ? (FLout) : (maxvalue)) : Wingup) * 1, true);
        setMotor(escFR, FRout > Wingup ? (FRout < maxvalue ? FRout : maxvalue) : Wingup, false);
        setMotor(escBL, (BLout > Wingup ? (BLout < maxvalue ? BLout : maxvalue) : Wingup) / 1.385 * 1.075, false);
        setMotor(escBR, (BRout > Wingup ? (BRout < maxvalue ? BRout : maxvalue) : Wingup) * 1.005, true);
        

        if (maxvalue <= 0.5)  //0.92
        {
            maxvalue = (maxvalue + (0.00015));
        }
        else
        {
            maxvalue = 0.5;
        }
    }

    
    if (motorMode == Starting)
    {
    Serial.println(currentvalue);
        if (currentvalue <= 0.29)
        {
            currentvalue = (currentvalue + (0.0001));
        }
        else
        {
            currentvalue = 0.3;
            maxvalue = 0.3;
            motorMode = Hold;
        }
       setMotor(escFL, currentvalue, true);
       setMotor(escFR, currentvalue, false);
       setMotor(escBL, currentvalue / 1.391, false);
       setMotor(escBR, currentvalue, true); 
    }

    if (motorMode == Arm || motorMode == Disabled)
    {
        setMotor(escFL, 0.0);
        setMotor(escFR, 0.0);
        setMotor(escBL, 0.0);
        setMotor(escBR, 0.0);
    }

if (motorMode == Land)
{
    Serial.println("Landing");
    Serial.println(maxvalue);
//           IF PID greater than wingup true : false    && FLout < maxvalue
        setMotor(escFL, FLout > Wingup ? (FLout < maxvalue ? FLout : maxvalue) : (Wingup > maxvalue ? Wingup : maxvalue), true);
        setMotor(escFR, FRout > Wingup ? (FRout < maxvalue ? FRout : maxvalue) : (Wingup > maxvalue ? Wingup : maxvalue), false);
        setMotor(escBL, (BLout > Wingup ? (BLout < maxvalue ? BLout : maxvalue) : (Wingup > maxvalue ? Wingup : maxvalue)) / 1.385, false);
        setMotor(escBR, BRout > Wingup ? (BRout < maxvalue ? BRout : maxvalue) : (Wingup > maxvalue ? Wingup : maxvalue), true);
        
        if (maxvalue >= 0.24)
        {
            maxvalue = (maxvalue - (0.0005/8));
        }
        else
        {
            maxvalue = 0;
            motorMode = Disabled;
        }
}


    if (time - printTime > 1000000)
    {
        printTime = time;
        Serial.println(timeDiff, 6);
        printData(temp, accel, gyro);
    }

delay(1);

    lastTime = time;
}
/*

//MAIN LOOP//
void loop(){

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }


  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }
  
  calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(12, HIGH);


  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is 
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure 
  //that the loop time is still 4000us and no longer! More information can be found on 
  //the Q&A page: 
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    
  if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  gyro_signalen();

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
  //Read the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.
    
    receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}
  }  
}
*/