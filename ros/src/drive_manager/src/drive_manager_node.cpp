#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "gamepad_msgs/Sticks.h"
#include "moon_msgs/TankDrivePowerControl.h"
#include "std_msgs/Float32.h"
#include <time.h>

// constexpr float scale = 0.2;
constexpr float scale = 1;
constexpr time_t timeout = 1; // seconds

class SticksCallbackHandler
{
public:
  static ros::Publisher *pubLeft, *pubRight;
  static time_t lastTime;

  SticksCallbackHandler(ros::Publisher *_pubLeft, ros::Publisher *_pubRight)
  {
    pubLeft = _pubLeft;
    pubRight = _pubRight;
  }

  static void sticksCallback(const gamepad_msgs::Sticks::ConstPtr &msg)
  {
    std_msgs::Float32 leftMsg, rightMsg;

    if (time(NULL) - lastTime > timeout)
    {
      // scale the stick values down so the bot drives a bit slower
      // also negate since negative values result from pushing up on the sticks
      leftMsg.data = -msg->ly * scale;
      rightMsg.data = -msg->rx * scale; // can change to rx to change steering;

      pubLeft->publish(leftMsg);
      pubRight->publish(rightMsg);
    }
  }

  static void autoDriveLeftCallback(const std_msgs::Float32::ConstPtr &msg)
  {
    std_msgs::Float32 leftMsg;

    leftMsg.data = msg->data;

    pubLeft->publish(leftMsg);

    lastTime = time(NULL);
  }

  static void autoDriveRightCallback(const std_msgs::Float32::ConstPtr &msg)
  {
    std_msgs::Float32 rightMsg;

    rightMsg.data = msg->data;

    pubRight->publish(rightMsg);

    lastTime = time(NULL);
  }
};

// must initialize static members outside of class, thanks C++
ros::Publisher *SticksCallbackHandler::pubLeft = nullptr;
ros::Publisher *SticksCallbackHandler::pubRight = nullptr;
time_t SticksCallbackHandler::lastTime = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_controller");
  ros::NodeHandle n;

  // set up the publisher, which will send out scaled stick y values and tank
  //  drive wheel powers
  ros::Publisher pubLeft = n.advertise<std_msgs::Float32>("left_power", 100);
  ros::Publisher pubRight = n.advertise<std_msgs::Float32>("right_power", 100);

  // initialize the callback handler for the subscriber, give it a reference
  //  to the publisher so it can send the modified message back out
  SticksCallbackHandler cbHandler(&pubLeft, &pubRight);

  // set up the subscriber, which will receive raw stick values
  ros::Subscriber sub = n.subscribe("sticks", 100, cbHandler.sticksCallback);
  ros::Subscriber autoDriveLeft = n.subscribe("autoDriveLeft", 100, cbHandler.autoDriveLeftCallback);
  ros::Subscriber autoDriveRight = n.subscribe("autoDriveRight", 100, cbHandler.autoDriveRightCallback);

  ros::spin();

  return 0;
}
