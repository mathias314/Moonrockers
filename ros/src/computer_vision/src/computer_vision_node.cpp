#include <ros/ros.h>

#include "moon_msgs/Pose.h"
#include "aruco_nano.h"

int main (int argc, char **argv)
{
  ros::init(argc, argv, "computer_vision");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<moon_msgs::Pose>("pose", 100);

  ros::Rate loop_rate(15);

  cv::VideoCapture cap(0);

  if (!cap.isOpened())
  {
    ROS_INFO("Failed to open camera!");
    return 1;
  }

  cv::namedWindow("preview");

  while (ros::ok())
  {
    cv::Mat frame;
    bool success = cap.read(frame);

    auto markers = aruconano::MarkerDetector::detect(frame);

    for (const auto &m : markers)
      m.draw(frame);

    cv::imshow("preview", frame);

    if (cv::waitKey(10) == 27)
    {
      break;
    }
  }

  return 0;
}
