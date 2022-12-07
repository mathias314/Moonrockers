/**
 * Example code for full bot control. This will display menus for actuator
 * control and reading sensor values.
 *
 * To Run:
 *      pio run -t upload -c examples.ini -e full_bot_control
 */

#include <Arduino.h>
#include <SPI.h>
// #include <Arduino_LSM6DS3.h>

// #include "can-serial.h"
#include "MoonBot.h"
#include "globals.h"
#include "canFuncs.h"

//#define STAT_LED 6

MoonBot robot(UPDATE_INTERVAL / 1000.0);

enum ControlStates {
    NONE,
    WAITING,
    MOTOR_CONTROL,
    PIVOT_CONTROL,
    LED_CONTROL,
    // LOAD_CELL,
    /*LIMIT_SWITCH*/
};
ControlStates controlState = NONE;

/**
 * Get the current reading from a given loadcell in kg.
 *
 * @param loadcell - load cell from which to read.
 * @param numRead - Number of readds to perform (default=1)
 * @param timeout - Time to wait for response (default=10ms)
 * @return The value using the load cell's calibration. NAN if no response.
 */
/*float readLoadCell(HX711 &loadcell, unsigned numRead = 1, unsigned timeout = 10) {
    if (loadcell.wait_ready_timeout(timeout)) {
        return loadcell.get_units(numRead);
    }
    return NAN;
}*/

bool controlMotor() {
    static int motorToControl = -1;
    static bool menuDisplayed = false;
    char readChar = 0;
    // Get the motor to control
    if (motorToControl == -1) {
        if (!menuDisplayed) {
            Serial.println("[MOTOR CONTROL]: Select motor");
            Serial.println("\t1 - Front Left");
            Serial.println("\t2 - Front Right");
            Serial.println("\t3 - Back Left");
            Serial.println("\t4 - Back Right");
            Serial.println("\t5 - Center Left");
            Serial.println("\t6 - Center Right");
            Serial.println("\tq - Quit motor control");
            menuDisplayed = true;
        }

        // Read the motor control number if given
        while (Serial.available()) {
            readChar = Serial.read();
        }
        if (readChar == 'q') {
            menuDisplayed = false;
            return false;
        } else if (readChar >= '1' && readChar <= '6') {
            menuDisplayed = false;
            motorToControl = readChar - '1';
        }
    } else {
        Serial.print("[MOTOR ");
        Serial.print(motorToControl+1);
        Serial.println(" power (-1.0,1.0)]: ");
        float val = Serial.parseFloat();
        robot.setMotorPower((MoonBot::Joint)motorToControl, val);
        Serial.println(val);

        while (Serial.available()) {
            readChar = Serial.read();
        }
        motorToControl = -1;
    }

    return true;
}

bool controlPivot() {
    static int pivotToControl = -1;
    static bool menuDisplayed = false;
    char readChar = 0;
    // Get the motor to control
    if (pivotToControl == -1) {
        if (!menuDisplayed) {
            Serial.println("[PIVOT CONTROL]: Select pivot");
            Serial.println("\t1 - Front Left");
            Serial.println("\t2 - Front Right");
            Serial.println("\t3 - Back Left");
            Serial.println("\t4 - Back Right");
            Serial.println("\tq - Quit motor control");
            menuDisplayed = true;
        }

        // Read the motor control number if given
        while (Serial.available()) {
            readChar = Serial.read();
        }
        if (readChar == 'q') {
            menuDisplayed = false;
            return false;
        } else if (readChar >= '1' && readChar <= '6') {
            menuDisplayed = false;
            pivotToControl = readChar - '1';
        }
    } else {
        Serial.print("[PIVOT ");
        Serial.print(pivotToControl+1);
        Serial.println(" angle (-90,90)]: ");
        float val = Serial.parseFloat();
        robot.setPivotAngle((MoonBot::Joint)pivotToControl, val*DEG_TO_RAD);
        Serial.println(val);

        while (Serial.available()) {
            readChar = Serial.read();
        }
        pivotToControl = -1;
    }

    return true;
}

/*bool controlLed() {
    static char stripToControl = 0;
    static bool menuDisplayed = false;
    char readChar = 0;
    // Get the strip to control
    if (stripToControl == 0) {
        if (!menuDisplayed) {
            Serial.println("[LED CONTROL]: Select strip");
            Serial.println("\tf - Front");
            Serial.println("\tb - Back");
            Serial.println("\tq - Quit LED control");
            menuDisplayed = true;
        }

        // Read the motor control number if given
        while (Serial.available()) {
            readChar = Serial.read();
        }
        if (readChar == 'q') {
            menuDisplayed = false;
            return false;
        } else if (readChar == 'f' || readChar == 'b') {
            menuDisplayed = false;
            stripToControl = readChar;
        }
    } else {
        if (!menuDisplayed) {
            Serial.print("[STRIP ");
            Serial.print(stripToControl);
            Serial.println("]: Select function");
            Serial.println("\t' ' - OFF");
            Serial.println("\tr - Red");
            Serial.println("\tg - Green");
            Serial.println("\tb - Blue");
            Serial.println("\tq - Exit control of this motor");
            menuDisplayed = true;
        }

        while (Serial.available()) {
            readChar = Serial.read();
        }

        // Control the selected motor
        menuDisplayed = false;
        uint32_t color = 0;
        switch (readChar) {
            case ' ':  // STOP
                color = Adafruit_NeoPixel::Color(0, 0, 0);
                Serial.println("[off]");
                break;
            case 'r':
                Serial.print("[red]");
                color = Adafruit_NeoPixel::Color(255, 0, 0);
                break;
            case 'g':
                Serial.print("[green]");
                color = Adafruit_NeoPixel::Color(0, 255, 0);
                break;
            case 'b':
                Serial.print("[blue]");
                color = Adafruit_NeoPixel::Color(0, 0, 255);
                break;
            case 'q':  // Quit
                stripToControl = 0;
                break;
            default:
                menuDisplayed = true;  // No need to re-display menu
                break;
        }

        if (menuDisplayed == false) {  // Value handled
            if (stripToControl == 'f') {
                frontLeds.fill(color);
                frontLeds.show();
            } else if (stripToControl == 'b') {
                backLeds.fill(color);
                backLeds.show();
            }
        }
    }

    return true;
}*/

/*
bool readImu() {
    static bool menuDisplayed = false;
    bool retVal = true;
    static unsigned long lastPrint = millis();

    // Print the menu
    if (!menuDisplayed) {
        Serial.println("[IMU]: Press 'q' to quit.");
        menuDisplayed = true;
        lastPrint = millis() + 2000;
    }

    // Check for user input
    while (Serial.available()) {
        if (Serial.read() == 'q') {
            retVal = false;
            menuDisplayed = false;
        }
    }

    // Print the values
    if (millis() - lastPrint > 500) {

        if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
            float gyro_x, gyro_y, gyro_z;  // rad/s
            float acc_x, acc_y, acc_z;  // m/s^2
            unsigned long gyroTime, accelTime;

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
        lastPrint = millis();
    }

    return retVal;
}*/

//=====setup================================================================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(5000);
    while (!Serial)
        ;  // wait for Serial

    delay(1000);
    Serial.println("initializing CAN motors...");
    Serial.flush();

    // set the baudrate and let it know we are using the 8MHz oscilator on the CAN module
    if (!canInit()) {
        Serial.println("CAN initialization failed.");
        while (true) {
            delay(1000);
        }
    }

    // Add the sender and receiver for the CAN communications.
    robot.addCanSender(canSend);
    // robot.addCanReceiver(CAN_INT_PIN, canReceive);

    // Initialize the robot
    robot.init();

    // Initialize lights
    /*Serial.println("Initializing LEDs...");
    frontLeds.begin();
    backLeds.begin();
    frontLeds.clear();
    backLeds.clear();
    frontLeds.show();
    backLeds.show();*/

    // Status LED
    // pinMode(STAT_LED, OUTPUT);
    // digitalWrite(STAT_LED, LOW);

    Serial.println("Done! Starting main loop.");
}

//=====loop================================================================
void loop() {
    char readVal = 0;
    bool result = true;

    robot.update();


    switch (controlState) {
        case NONE:
            Serial.println("ENTER CONTROL MODE:");
            Serial.println("\tm - Motor control");
            Serial.println("\tp - Pivot control");
            Serial.println("\tl - LED control");
            controlState = WAITING;
            break;
        case MOTOR_CONTROL:
            result = controlMotor();
            break;
        case PIVOT_CONTROL:
            result = controlPivot();
            break;
        case LED_CONTROL:
            //result = controlLed();
            break;
        default:
            while (Serial.available()) {
                readVal = Serial.read();
            }

            switch (readVal) {
                case '\0':
                case '\r':
                case '\n':
                case '\t':
                    break;
                case 'm':
                    controlState = MOTOR_CONTROL;
                    break;
                case 'p':
                    controlState = PIVOT_CONTROL;
                    break;
                case 'l':
                    controlState = LED_CONTROL;
                    break;
                default:
                    Serial.print("'");
                    Serial.print(readVal);
                    Serial.println("' is not a valid option.");
                    break;
            }

            break;
    }

    // Check if we requested exit from that control mode
    if (result == false) {
        controlState = NONE;
    }
}
