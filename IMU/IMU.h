/* MPU6050 Basic Example with IMU  
 by: Kris Winer
 modified by: Lewis Cooper
 filter algorithm credit to: Sebastian Madgwick
 date: October 25, 2016
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time. I owe you a beer
 Kris.
 
 Demonstrate  MPU-6050 basic functionality including initialization, accelerometer trimming, sleep mode functionality as well as
 parametrizing the register addresses. Added display functions to allow display to on breadboard monitor. 
 No DMP use. We just want to get out the accelerations, temperature, and gyroscope readings.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors worked for me. They should be on the breakout
 board.
 
 Hardware setup:
 MPU6050 Breakout --------- Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note IMPORTANT!!!!: The MPU6050 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
 
 /*
 IMPORTANT and TODO: The fastest a single IMU can output raw data is at 200 Hz. This may be sufficient 
 and likely not to be fixed. But keep in mind that that the data rate at post processing will always 
 be less than 200 Hz.
 */
 #include <stdint.h>
 
 class IMU {
    public:
        IMU();
      
        int Initialize(float timeConstant); //return true for success. Program should be halted or throw an error upon failure.
        int Initialize(float timeConstant, uint8_t address);
        int Update();
        
        float GetQuaternion0();
        float GetQuaternion1();
        float GetQuaternion2();
        float GetQuaternion3();
        float GetAccelX();
        float GetAccelY();
        float GetAccelZ();
        float GetFilteredAccelX();
        float GetFilteredAccelY();
        float GetFilteredAccelZ();
        float GetGyroX();
        float GetGyroY();
        float GetGyroZ();
        float GetYaw();
        float GetPitch();
        float GetRoll();
        uint32_t GetDeltaT();
        
    private:
        
        int8_t _address; //I2C device's address      
        float _yaw, _pitch, _roll; //orientation angle in degrees
        int16_t _accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
        float _a[3];       // Stores the real accel value in g's
        float _aPrevious[3];
        float _aFiltered[3];
        int16_t _gyroCount[3];   // Stores the 16-bit signed gyro sensor output
        float _gx, _gy, _gz;       // Stores the real gyro value in degrees per seconds
        float _gyroBias[3] = {0, 0, 0}; 
        float _accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
        int16_t _tempCount;   // Stores the real internal chip temperature in degrees Celsius
        float _temperature;      
        float _aRes, _gRes; // scale resolutions per LSB for the sensors
        int _Gscale;
        int _Ascale;
        
        // parameters for 6 DoF sensor fusion calculations
        float _GyroMeasError;     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
        float _beta;  // compute beta
        float _GyroMeasDrift;      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
        float _zeta;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
        float _q[4];            // vector to hold quaternion    
        uint32_t _deltat;
        uint32_t _before;
        uint32_t _time;
        float _time_constant;
        
        void MadgwickSensorFusionAlgorithm(float ax, float ay, float az, float gx, float gy, float gz);
        void RemoveGravity();
        
        void initMPU6050();
        void calibrateMPU6050(float * dest1, float * dest2);
        void MPU6050SelfTest(float * destination);
        
        void readAccelData(int16_t * destination);
        void readGyroData(int16_t * destination);
        int16_t readTempData();
        void getGres();
        void getAres();
        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
        uint8_t readByte(uint8_t address, uint8_t subAddress);
        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
};
