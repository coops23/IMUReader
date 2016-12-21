#include <Wire.h>
#include "IMU.h"

#define BAUDRATE (38400)
#define blinkPin 13
#define COMPLIMENTARY_FILTER_TIME_CONSTANT (0.8f) //Not calculated just a guess from online research

boolean blinkOn = false;
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate

IMU imu;

void setup()
{
    Wire.begin();
    Serial.begin(BAUDRATE);

    pinMode(blinkPin, OUTPUT);
    digitalWrite(blinkPin, blinkOn);

    if(!imu.Initialize(COMPLIMENTARY_FILTER_TIME_CONSTANT))
    {
        Serial.println("IMU Initialization Failure");
        while(1); //Should change to trigger a uC reset
    }
}

void loop()
{  
    if(imu.Update())
    {
        Serial.print("Info: "); Serial.print(imu.GetYaw()); Serial.print(" "); Serial.print(imu.GetPitch() * (180.0f/PI)); Serial.print(" "); Serial.print(imu.GetRoll() * (180.0f/PI)); Serial.print(" ");
        Serial.print(imu.GetAccelX()); Serial.print(" "); Serial.print(imu.GetAccelY()); Serial.print(" "); Serial.print(imu.GetAccelZ()); Serial.print(" ");
        Serial.print(imu.GetQuaternion0()); Serial.print(" "); Serial.print(imu.GetQuaternion1()); Serial.print(" "); Serial.print(imu.GetQuaternion2()); Serial.print(" "); Serial.print(imu.GetQuaternion3()); Serial.print(" ");
        Serial.println(imu.GetDeltaT());
        
        blinkOn = !blinkOn;
        digitalWrite(blinkPin, blinkOn);
    }   
}

