#include <Arduino.h>
#define USE_USBCON // fix for ROS not communicating on the Arduino Nano 33 IOT
#include <ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/ColorRGBA.h"

#include <SPI.h>
#include "SparkMc.h"
#include "pins.h"
#include "PwmMotor.h"
#include <Adafruit_NeoPixel.h>


// pin numbers
constexpr int LED = 13;

// CAN keepalive variables
uint32_t canKeepAliveLastTime = 0;
constexpr uint8_t canKeepAlivePeriod = 40; // ms, should be under 100

// status LED blink
uint32_t blinkLastTime = 0;
uint16_t blinkPeriod = 500;
uint8_t blinkState = 0;

int collectionPlungeLowerEndstopPin = 38;
int collectionPlungeUpperEndstopPin = 40;
float collectionPlungeLastPower = 0;

#include <mcp_can.h>
#include "can-serial.h"
#include "mcp2515_can.h"
#include "mcp_can.h"

const int spiCSPin = 9;
mcp2515_can CAN(spiCSPin);  // Set CS pin

bool canInit() {
    return CAN.begin(CAN_1000KBPS, MCP_16MHz) == CAN_OK;   //! Make sure oscilator set to correct value!!!
}

void canSend(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *dat) {
    //sendMsgBuf(unsigned long id, byte ext, byte len, const byte *buf)
    CAN.sendMsgBuf(id, ext, len, dat);
}

unsigned canReceive(uint32_t *id, uint8_t *dat) {
    byte len = 0;
    CAN.readMsgBuf(&len, dat);
    *id = CAN.getCanId();
    return len;
}

// --------Motor control stuff-----------------
// SPARK motor definitions
SparkMc mtrFrontLeft(FL_MTR_ID);
SparkMc mtrFrontRight(FR_MTR_ID);
SparkMc mtrBackRight(BR_MTR_ID);
SparkMc mtrBackLeft(BL_MTR_ID);
SparkMc mtrBucket(DELIVERY_MTR_ID);
SparkMc mtrPlunge(PLUNGE_MTR_ID);

// this should include ALL motors on the robot
SparkMc* motors[] = {&mtrFrontLeft, &mtrFrontRight, &mtrBackRight, &mtrBackLeft, &mtrPlunge, &mtrBucket};
uint8_t numMotors = sizeof(motors) / sizeof(SparkMc*);

PwmMotor mtrGate(GATE_PIN, GATE_PWM_CHANNEL);
PwmMotor mtrConveyor(CONVEYOR_PIN, CONVEYOR_PWM_CHANNEL);


uint32_t lastUpdateTime = 0;
int updateTimeout = 1000;


// --------Pretty light stuff-----------------
Adafruit_NeoPixel frontLeds(5, FRONT_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel backLeds(5, BACK_LED_PIN, NEO_GRB + NEO_KHZ800);


// --------ROS stuff-----------------
ros::NodeHandle node_handle;

void leftCallback(const std_msgs::Float32& powerMsg)
{
  mtrFrontLeft.setVelocity(powerMsg.data);
  mtrBackLeft.setVelocity(powerMsg.data);
  lastUpdateTime = millis();
}

void rightCallback(const std_msgs::Float32& powerMsg)
{
  // these powers are negated since the motors are facing opposite the left motors
  mtrFrontRight.setVelocity(-powerMsg.data);
  mtrBackRight.setVelocity(-powerMsg.data);
  lastUpdateTime = millis();
}

void chainDriveCallback(const std_msgs::Float32& powerMsg)
{
  mtrConveyor.setPower(powerMsg.data);
  lastUpdateTime = millis();
}

void gateCallback(const std_msgs::Float32& powerMsg)
{
  mtrGate.setPower(powerMsg.data);
  lastUpdateTime = millis();
}

void bucketCallback(const std_msgs::Float32& powerMsg)
{
  mtrBucket.setPower(powerMsg.data);
  lastUpdateTime = millis();
}

void collectionPlungeCallback(const std_msgs::Float32& powerMsg)
{
  mtrPlunge.setPower(powerMsg.data);
  lastUpdateTime = millis();
}

// void frontLedCallback(const std_msgs::ColorRGBA& colorMsg)
// {
//   frontLeds.fill(Adafruit_NeoPixel::Color(colorMsg.r, colorMsg.g, colorMsg.b), NUM_FRONT_LEDS);
//   frontLeds.show();
//   lastUpdateTime = millis();
// }

// void backLedCallback(const std_msgs::ColorRGBA& colorMsg)
// {
//   backLeds.fill(Adafruit_NeoPixel::Color(colorMsg.r, colorMsg.g, colorMsg.b), NUM_BACK_LEDS);
//   backLeds.show();
//   lastUpdateTime = millis();
// }

// TODO Not entirely sure what this is...
// void collectionPlungeCallbackFake(float power)
// {
//   int powerInt = 1500;

//   if (power > 0 && digitalRead(collectionPlungeUpperEndstopPin))
//   {
//     powerInt = map(power * 1000, 0, 1000, 1500, 2000);
//   }
//   else if (power < 0 && digitalRead(collectionPlungeLowerEndstopPin))
//   {
//     powerInt = map(-power * 1000, 0, 1000, 1500, 1000);
//   }

//   collectionPlunge.writeMicroseconds(powerInt);
//   lastUpdateTime = millis();
// }

ros::Subscriber<std_msgs::Float32> subLeft("left_power", &leftCallback);
ros::Subscriber<std_msgs::Float32> subRight("right_power", &rightCallback);
ros::Subscriber<std_msgs::Float32> subCollectionChainDrive("chain_drive_power", &chainDriveCallback);
ros::Subscriber<std_msgs::Float32> subCollectionPlunge("collection_plunge_power", &collectionPlungeCallback);
ros::Subscriber<std_msgs::Float32> subBucket("bucket_power", &bucketCallback);
// ros::Subscriber<std_msgs::Float32> subGate("gate_power", &gateCallback);
// ros::Subscriber<std_msgs::ColorRGBA> subFrontLed("front_led", &frontLedCallback);
// ros::Subscriber<std_msgs::ColorRGBA> subBackLed("back_led", &frontLedCallback);

void setup()
{
  pinMode(LED, OUTPUT);
  int ledPattern[4] = {0, 0, 0, 1};
  int ledCounter = 0;
  
  // --- Pretty light startup --- 
  frontLeds.begin();
  backLeds.begin();
  frontLeds.fill(Adafruit_NeoPixel::Color(150, 0, 0), NUM_FRONT_LEDS);
  frontLeds.show();
  backLeds.fill(Adafruit_NeoPixel::Color(150, 0, 0), NUM_BACK_LEDS);
  backLeds.show();

  // --- Motor control startup ---
  //set the baudrate and let it know we are using the 16MHz oscilator on the CAN module
  if (!canInit()) {
    while (true) {
      delay(1000);
      digitalWrite(LED, ledPattern[ledCounter++ % 4]);
    }
  }

  // Add the sender for the CAN communications.
  SparkMc::addCanSender(canSend);
  SparkMc::addCanReceiver(CAN_INT_PIN, canReceive);

  // get the motors ready to go
  for (int i = 0; i < numMotors; i++)
  {
    motors[i]->clearFaults();
    motors[i]->setPower(0);
  }
  mtrConveyor.init();
  mtrGate.init();
  
  // --- ROS Startup ---
  node_handle.initNode();
  node_handle.subscribe(subLeft);
  node_handle.subscribe(subRight);
  node_handle.subscribe(subCollectionChainDrive);
  node_handle.subscribe(subCollectionPlunge);
  node_handle.subscribe(subBucket);
  // node_handle.subscribe(subGate);

  // Indicate we are done
  frontLeds.fill(Adafruit_NeoPixel::Color(0, 150, 0), NUM_FRONT_LEDS);
  frontLeds.show();
  backLeds.fill(Adafruit_NeoPixel::Color(0, 150, 0), NUM_BACK_LEDS);
  backLeds.show();
}

void loop()
{ 
  // Update at a fixed interval (<100ms I think)
  if (millis() - canKeepAliveLastTime > canKeepAlivePeriod)
  {
    canKeepAliveLastTime = millis();
    SparkMc::sendKeepAlive();
  }

  if (millis() - blinkLastTime > blinkPeriod)
  {
    blinkLastTime = millis();
    blinkState = !blinkState;
    digitalWrite(LED, blinkState);
  }

  // cut power to all motors if we haven't received anything in the past 100ms
  if (millis() - lastUpdateTime > updateTimeout)
  {
    for (int i = 0; i < numMotors; i++)
    {
      motors[i]->setPower(0);
    }
    
    mtrConveyor.setPower(0);
    mtrGate.setPower(0);
  }

  node_handle.spinOnce();
}
