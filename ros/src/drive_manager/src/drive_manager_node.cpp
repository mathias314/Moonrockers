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
  static ros::Publisher *drivePub, *steerPub;
  static time_t lastTime;

  SticksCallbackHandler(ros::Publisher *_drivePub, ros::Publisher *_steerPub)
  {
    drivePub = _drivePub;
    steerPub = _steerPub;
  }

  static void sticksCallback(const gamepad_msgs::Sticks::ConstPtr &msg)
  {
    std_msgs::Float32 driveMsg, steerMsg;

    if (time(NULL) - lastTime > timeout)
    {
      // scale the stick values down so the bot drives a bit slower
      // also negate since negative values result from pushing up on the sticks
      driveMsg.data = -msg->ly * scale;
      steerMsg.data = -msg->rx * scale; // can change to rx to change steering;

      drivePub->publish(driveMsg);
      steerPub->publish(steerMsg);
    }
  }

  static void autoDriveCallback(const std_msgs::Float32::ConstPtr &msg)
  {
    std_msgs::Float32 driveMsg;

    driveMsg.data = msg->data;

    drivePub->publish(driveMsg);

    lastTime = time(NULL);
  }

  static void autoSteerCallback(const std_msgs::Float32::ConstPtr &msg)
  {
    std_msgs::Float32 steerMsg;

    steerMsg.data = msg->data;

    steerPub->publish(steerMsg);

    lastTime = time(NULL);
  }
};

// must initialize static members outside of class, thanks C++
ros::Publisher *SticksCallbackHandler::drivePub = nullptr;
ros::Publisher *SticksCallbackHandler::steerPub = nullptr;
time_t SticksCallbackHandler::lastTime = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drive_controller");
  ros::NodeHandle n;

  // set up the publisher, which will send out scaled stick y values and tank
  //  drive wheel powers
  ros::Publisher drivePub = n.advertise<std_msgs::Float32>("drive_power", 100);
  ros::Publisher steerPub = n.advertise<std_msgs::Float32>("steer_power", 100);

  // initialize the callback handler for the subscriber, give it a reference
  //  to the publisher so it can send the modified message back out
  SticksCallbackHandler cbHandler(&drivePub, &steerPub);

  // set up the subscriber, which will receive raw stick values
  ros::Subscriber sub = n.subscribe("sticks", 100, cbHandler.sticksCallback);
  ros::Subscriber autoDriveLeft = n.subscribe("autoDrive", 100, cbHandler.autoDriveCallback);
  ros::Subscriber autoDriveRight = n.subscribe("autoSteer", 100, cbHandler.autoSteerCallback);

  ros::spin();

  return 0;
}
