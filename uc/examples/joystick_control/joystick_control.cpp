/**
 * Example code for full bot control. This will display menus for actuator
 * control and reading sensor values.
 *
 * To Run:
 *      pio run -t upload -c examples.ini -e joystick_control
 */
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Arduino.h>
#include <Ps3Controller.h>

#include "MoonBot.h"
#include "globals.h"
#include "canFuncs.h"

MoonBot robot(UPDATE_INTERVAL / 1000.0);

#define JOY_DEAD_ZONE 0.1
#define JOY_MAX 200.0 // TODO: Not sure if this is right...

// status LED!
const int BLINK_PERIOD = 200; // ms between blinks
bool ledState = 0;
uint32_t prevTimeLED = 0;

// Some controller calibration values
bool calDone = false;
float joyLMiddle = 0.0;
float joyRMiddle = 0.0;

void onConnection()
{
    if (Ps3.isConnected())
    {
        // Calibrate the middle point
        if (!calDone)
        {
            joyLMiddle = Ps3.data.analog.stick.ly;
            joyRMiddle = Ps3.data.analog.stick.ry;
        }

        Serial.println("Connected!");
        Ps3.setPlayer(1);
    }
    else
    {
        Serial.println("Disconnected.");
    }
}

void setup()
{
    Serial.begin(115200);
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

    // Initialize the controller
    if (!Ps3.begin("00:1a:7d:da:71:13"))
    {
        Serial.println("Controller initialization failed.");
        return;
    }
    Serial.println("Controller initialization done!");
    Ps3.attachOnConnect(onConnection);

    // Finish robot initialization
    robot.addCanSender(canSend);
    robot.init();

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    static unsigned long lastTime = millis();
    static unsigned long lastDrive = 0;
    static float joyLY, joyRY;
    static float lastJoyLY = 0.0;
    static float lastJoyRY = 0.0;
    static unsigned long lastCycleTime = 0;
    static unsigned long releaseStart = 0;

    // Get the current values from Dabble
    if (Ps3.isConnected())
    {
        if (millis() - lastDrive > 25)
        {
            joyLY = -(Ps3.data.analog.stick.ly - joyLMiddle) / JOY_MAX;
            joyRY = -(Ps3.data.analog.stick.ry - joyRMiddle) / JOY_MAX;
            
            
            // If joystick getting released, skip this cycle
            float maxRate = 0.002 / max(millis() - lastCycleTime, 1ul);
            bool stickReleased = ((abs(joyLY) - abs(lastJoyLY)) < -maxRate || (abs(joyRY) - abs(lastJoyRY)) < -maxRate);
            lastJoyLY = joyLY;
            lastJoyRY = joyRY;
            lastCycleTime = millis();
            if (stickReleased)
            {
                releaseStart = millis();
            }
            if ((millis() - releaseStart) > 50)
            {
                robot.drive(joyLY, joyRY);
            }
            lastDrive = millis();
        }

        // Solid built-in LED
        digitalWrite(LED_BUILTIN, HIGH);
        if (millis() - lastTime > 500)
        {
            Serial.printf("ly: %.2f, ry: %.2f\n", joyLY, joyRY);
            Serial.flush();

            lastTime = millis();
        }
    }
    else
    {
        robot.stop();

        // Blinking built-in LED
        if (millis() > prevTimeLED + BLINK_PERIOD)
        {
            digitalWrite(LED_BUILTIN, ledState);
            ledState = !ledState;
            prevTimeLED = millis();
        }
    }

    robot.update();
}