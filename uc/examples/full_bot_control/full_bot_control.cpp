/**
 * Example code for full bot control. This will display menus for actuator
 * control and reading sensor values.
 *
 * To Run:
 *      pio run -t upload -c examples.ini -e full_bot_control
 */

#include <Arduino.h>
#include <HX711.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Arduino_LSM6DS3.h>
#include <Adafruit_NeoPixel.h>

#include "PwmMotor.h"
#include "SparkMc.h"
#include "can-serial.h"
#include "mcp2515_can.h"
#include "mcp_can.h"
#include "pins.h"

#define STAT_LED 6

mcp2515_can CAN(CAN_CS_PIN);  // Set CS pin

bool canInit() {
    return CAN.begin(CAN_1000KBPS, MCP_16MHz) == CAN_OK;  //! Make sure oscilator set to correct value!!!
}

void canSend(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat) {
    // sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf)
    CAN.sendMsgBuf(id, ext, len, dat);
}

unsigned canReceive(uint32_t *id, uint8_t *dat) {
    byte len = 0;
    CAN.readMsgBuf(&len, dat);
    *id = CAN.getCanId();
    return len;
}

SparkMc mtrFrontLeft(FL_MTR_ID);
SparkMc mtrFrontRight(FR_MTR_ID);
SparkMc mtrBackRight(BR_MTR_ID);
SparkMc mtrBackLeft(BL_MTR_ID);
SparkMc mtrBucket(DELIVERY_MTR_ID);
SparkMc mtrPlunge(PLUNGE_MTR_ID);

// this should include ALL motors on the robot
SparkMc *motors[] = {&mtrFrontLeft, &mtrFrontRight, &mtrBackRight, &mtrBackLeft, &mtrPlunge, &mtrBucket};
uint8_t numMotors = sizeof(motors) / sizeof(SparkMc *);

PwmMotor mtrGate(GATE_PIN, GATE_PWM_CHANNEL);
PwmMotor mtrConveyor(CONVEYOR_PIN, CONVEYOR_PWM_CHANNEL);

Adafruit_NeoPixel frontLeds(NUM_FRONT_LEDS, FRONT_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel backLeds(NUM_BACK_LEDS, BACK_LED_PIN, NEO_GRB + NEO_KHZ800);

HX711 loadcellLeft;
HX711 loadcellRight;

enum ControlStates { NONE,
                     WAITING,
                     MOTOR_CONTROL,
                     LED_CONTROL,
                     LOAD_CELL,
                     LIMIT_SWITCH };
ControlStates controlState = NONE;

/**
 * Get the current reading from a given loadcell in kg.
 *
 * @param loadcell - load cell from which to read.
 * @param numRead - Number of readds to perform (default=1)
 * @param timeout - Time to wait for response (default=10ms)
 * @return The value using the load cell's calibration. NAN if no response.
 */
float readLoadCell(HX711 &loadcell, unsigned numRead = 1, unsigned timeout = 10) {
    if (loadcell.wait_ready_timeout(timeout)) {
        return loadcell.get_units(numRead);
    }
    return NAN;
}

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
            Serial.println("\t3 - Back Right");
            Serial.println("\t4 - Back Left");
            Serial.println("\t5 - Plunge");
            Serial.println("\t6 - Delivery");
            Serial.println("\t7 - Gate");
            Serial.println("\t8 - Conveyor");
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
        } else if (readChar >= '1' && readChar <= '8') {
            menuDisplayed = false;
            motorToControl = readChar - '0';
        }
    } else {
        if (!menuDisplayed) {
            Serial.print("[MOTOR ");
            Serial.print(motorToControl);
            Serial.println("]: Select function");
            Serial.println("\t' ' - STOP");
            Serial.println("\tr - Percent output control (-1 to 1)");
            if (motorToControl <= 6) {
                Serial.println("\tv - Velocity control (rpm)");
                Serial.println("\tp - Position control (rotations)");
                Serial.println("\td - Display motor stats");
            }
            Serial.println("\tq - Exit control of this motor");
            menuDisplayed = true;
        }

        while (Serial.available()) {
            readChar = Serial.read();
        }

        // Control the selected motor
        menuDisplayed = false;
        if (motorToControl <= 6) {
            float val = 0.0;
            switch (readChar) {
                case ' ':  // STOP
                    motors[motorToControl - 1]->setPower(0.0);
                    Serial.println("[stop]");
                    break;
                case 'r':  // Percent output control
                    Serial.print("[percent]:");
                    val = Serial.parseFloat(SKIP_WHITESPACE);
                    motors[motorToControl - 1]->setPower(val);
                    Serial.println(val);
                    break;
                case 'v':  // Velocity control
                    Serial.print("[velocity]:");
                    val = Serial.parseFloat(SKIP_WHITESPACE);
                    motors[motorToControl - 1]->setPidSlot(0);
                    motors[motorToControl - 1]->setVelocity(val);
                    Serial.println(val);
                    break;
                case 'p':  // Position control
                    Serial.print("[position]:");
                    val = Serial.parseFloat(SKIP_WHITESPACE);
                    motors[motorToControl - 1]->setPidSlot(1);
                    motors[motorToControl - 1]->setPosition(val);
                    Serial.println(val);
                    break;
                case 'd':  // Display motor stats
                    Serial.flush();
                    Serial.print("[stats]: T:");
                    Serial.print(motors[motorToControl - 1]->getTemperature());
                    Serial.print(", %:");
                    Serial.print(motors[motorToControl - 1]->getAppliedOutput());
                    Serial.print(", A:");
                    Serial.print(motors[motorToControl - 1]->getOutputCurrent());
                    Serial.print(", V:");
                    Serial.print(motors[motorToControl - 1]->getBusVoltage());
                    Serial.print(", P:");
                    Serial.print(motors[motorToControl - 1]->getPosition());
                    Serial.print(", v:");
                    Serial.println(motors[motorToControl - 1]->getVelocity());
                    Serial.flush();
                    break;
                case 'q':  // Quit
                    motorToControl = -1;
                    break;
                default:
                    menuDisplayed = true;  // No need to re-display menu
                    break;
            }
        } else {
            float val = 0.0;
            switch (readChar) {
                case ' ':  // STOP
                    if (motorToControl == 7) {
                        mtrGate.setPower(0.0);
                    } else if (motorToControl == 8) {
                        mtrConveyor.setPower(0.0);
                    }
                    Serial.println("[stop]");
                    break;
                case 'r':  // Percent output control
                    Serial.print("[percent]: ");
                    val = Serial.parseFloat(SKIP_WHITESPACE);
                    if (motorToControl == 7) {
                        mtrGate.setPower(val);
                    } else if (motorToControl == 8) {
                        mtrConveyor.setPower(val);
                    }
                    Serial.println(val);
                    break;
                case 'q':  // Quit
                    motorToControl = -1;
                    break;
                default:
                    menuDisplayed = true;  // No need to re-display menu
                    break;
            }
        }
    }

    return true;
}

bool controlLed() {
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
}

bool readLoadcell() {
    static bool menuDisplayed = false;
    bool retVal = true;
    static unsigned long lastPrint = millis();

    // Print the menu
    if (!menuDisplayed) {
        Serial.println("[LOAD CELL]: Press 'q' to quit.");
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
        Serial.print("Left:");
        Serial.print(readLoadCell(loadcellLeft));
        Serial.print(" kg, Right:");
        Serial.print(readLoadCell(loadcellRight));
        Serial.println(" kg");
        lastPrint = millis();
    }

    return retVal;
}

bool readLimitSwitch() {
    static bool menuDisplayed = false;
    bool retVal = true;
    static unsigned long lastPrint = millis();

    // Print the menu
    if (!menuDisplayed) {
        Serial.println("[LIMIT SWITCH]: Press 'q' to quit.");
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
        Serial.print("Delivery:{Fwd: ");
        Serial.print(mtrPlunge.limitForward());
        Serial.print(", Rev: ");
        Serial.print(mtrPlunge.limitReverse());
        Serial.print("}, Collection:{Fwd: ");
        Serial.print(mtrPlunge.limitForward());
        Serial.print(", Rev: ");
        Serial.print(mtrPlunge.limitReverse());
        Serial.println("}");
        lastPrint = millis();
    }

    return retVal;
}

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
}

//=====setup================================================================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(5000);
    while (!Serial)
        ;  // wait for Serial

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
    SparkMc::addCanSender(canSend);
    SparkMc::addCanReceiver(CAN_INT_PIN, canReceive);

    // Clear faults
    for (int i = 0; i < numMotors; i++) {
        motors[i]->clearFaults();
    }

    // Initialize motors
    Serial.println("Initializing PWM motors...");
    mtrGate.init();
    mtrConveyor.init();

    // Initialize lights
    Serial.println("Initializing LEDs...");
    frontLeds.begin();
    backLeds.begin();
    frontLeds.clear();
    backLeds.clear();
    frontLeds.show();
    backLeds.show();

    // Load cells
    loadcellLeft.begin(LEFT_LC_DAT_PIN, LEFT_LC_CLK_PIN);
    loadcellLeft.set_scale(LOADCELL_SCALE);
    loadcellLeft.set_offset(LOADCELL_OFFSET);
    loadcellRight.begin(RIGHT_LC_DAT_PIN, RIGHT_LC_DAT_PIN);
    loadcellLeft.set_scale(LOADCELL_SCALE);
    loadcellLeft.set_offset(LOADCELL_OFFSET);

    // IMU
    Serial.println("Initializing IMU...");
    if (!IMU.begin()) {
        Serial.println("IMU initialize failed.");
    }

    // Status LED
    pinMode(STAT_LED, OUTPUT);
    digitalWrite(STAT_LED, LOW);

    Serial.println("Done! Starting main loop.");
}

//=====loop================================================================
void loop() {
    char readVal = 0;
    // Update at a fixed interval (<100ms I think)
    delay(80);
    SparkMc::sendKeepAlive();

    bool result = true;

    switch (controlState) {
        case NONE:
            Serial.println("ENTER CONTROL MODE:");
            Serial.println("\tm - Motor control");
            Serial.println("\tl - LED control");
            Serial.println("\tc - Load cell");
            Serial.println("\ts - Limit switches");
            controlState = WAITING;
            break;
        case MOTOR_CONTROL:
            result = controlMotor();
            break;
        case LED_CONTROL:
            result = controlLed();
            break;
        case LOAD_CELL:
            result = readLoadcell();
            break;
        case LIMIT_SWITCH:
            result = readLimitSwitch();
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
                case 'l':
                    controlState = LED_CONTROL;
                    break;
                case 'c':
                    controlState = LOAD_CELL;
                    break;
                case 's':
                    controlState = LIMIT_SWITCH;
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
