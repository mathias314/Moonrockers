//////////////////////////////////////////////////////////////////////////////
// Description: Code for testing Nano IOT IMU.
// Example code with reporting IMU to ROS: https://github.com/sdsmt-robotics/nrc-avc-2020/blob/master/driveCode/driveCode.ino
// To run:
//       pio run -t upload -c examples.ini -e imu
/////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

//=====setup==============================
void setup() {
    Serial.begin(115200);

    for (int i = 0; i < 10; i++) {
        delay(1000);
        Serial.println(10-i);
    }

    // Initialize library
    Serial.println("Initializing IMU...");
    Serial.flush();
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");
}

//=====loop==============================
void loop() {
    static float gyro_x, gyro_y, gyro_z;  // rad/s
    static float acc_x, acc_y, acc_z;  // m/s^2
    static unsigned long gyroTime, accelTime;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        // Get current values
        gyroTime = millis();
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        gyroTime = millis() - gyroTime;
        accelTime = millis();
        IMU.readAcceleration(acc_x, acc_y, acc_z);
        accelTime = millis() - accelTime;

        // Output reading
        Serial.print("gyro:[x:");
        Serial.print(gyro_x, 2);
        Serial.print(",\ty:");
        Serial.print(gyro_y, 2);
        Serial.print(",\tz:");
        Serial.print(gyro_z, 2);
        Serial.print(",\tt:");
        Serial.print(gyroTime);
        Serial.print("], accel:[x:");
        Serial.print(acc_x, 2);
        Serial.print(",\ty:");
        Serial.print(acc_y, 2);
        Serial.print(",\tz:");
        Serial.print(acc_z, 2);
        Serial.print(",\tt:");
        Serial.print(gyroTime);
        Serial.println("]");
    }

    delay(250);
}
