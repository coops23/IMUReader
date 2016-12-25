#include "IMU.h"
#include <Wire.h>

#define BAUDRATE (38400)
#define COMPLIMENTARY_FILTER_TIME_CONSTANT (0.8f) //Not calculated just a guess from online research

IMU imu;

void(* resetFunc) (void) = 0;

void setup()
{
    Serial.begin(BAUDRATE);
    Wire.begin();
    
    if(!imu.Initialize(COMPLIMENTARY_FILTER_TIME_CONSTANT))
    {
        Serial.println("IMU Initialization Failure");
        resetFunc();
    }
}

void loop()
{  
    if(imu.Update())
    {
        Serial.print("Info: "); Serial.print(imu.GetYaw()); Serial.print(" "); Serial.print(imu.GetPitch() * (180.0f/PI)); Serial.print(" "); Serial.print(imu.GetRoll() * (180.0f/PI)); Serial.print(" ");
        Serial.print(imu.GetAccelX()); Serial.print(" "); Serial.print(imu.GetAccelY()); Serial.print(" "); Serial.print(imu.GetAccelZ()); Serial.print(" ");
        Serial.println(imu.GetDeltaT());
    }   
}

