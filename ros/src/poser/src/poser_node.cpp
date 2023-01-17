#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <aruco.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

using namespace std;

int main(int argc, char** argv) {
  cv::VideoCapture videoCapture;
  aruco::MarkerDetector markerDetector;
  vector<aruco::Marker> markers;
  // this will get more complicated when we're using more than one camera
  aruco::CameraParameters cameraParameters;
  float markerSize = 0.508;
  cv::Mat inputImage;

  // initialize ROS
  ros::init(argc, argv, "poser");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 1000);
  geometry_msgs::Pose pose;
  ros::Rate rate(10);

  // start up the camera capture
  videoCapture.open(0);

  videoCapture >> inputImage;
  cameraParameters.readFromXMLFile("dustin-calibration-2021-11-30.yml");
  //cout << "width: " << videoCapture.get(cv::CAP_PROP_FRAME_WIDTH) << " height: " <<
  //  videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
  cout << "width: " << inputImage.cols << " height: " <<
    inputImage.rows << std::endl;

  if (inputImage.empty()) cout << ">> Image empty!" << std::endl;
  if (!cameraParameters.isValid()) cout << "Camera parameters not valid!" << std::endl;

  while (ros::ok())
  {
    // take in an image and attempt to detect markers
    videoCapture.read(inputImage);
    //cv::imshow("Camera View", inputImage);
    if (inputImage.empty()) cout << "Image empty!" << std::endl;
    markers = markerDetector.detect(inputImage, cameraParameters, markerSize);

    // send out the pose according to the first detected marker
    if (markers.size())
    {
      markers[0].calculateExtrinsics(markerSize, cameraParameters);
      pose.position.x = markers[0].Tvec.at<float>(0);
      pose.position.y = markers[0].Tvec.at<float>(1);
      pose.position.z = markers[0].Tvec.at<float>(2);
      pose_pub.publish(pose);
    }
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
