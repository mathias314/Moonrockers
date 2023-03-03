#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <aruco/cvdrawingutils.h>
#include <aruco/dcf/dcfmarkermaptracker.h>
#include <aruco.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Core>

using namespace std;

int main(int argc, char** argv) {
  vector<aruco::Marker> markers;
  // this will get more complicated when we're using more than one camera
  aruco::CameraParameters cameraParameters;
  cv::VideoCapture videoCapture;
  cv::Mat inputImage;

  // initialize ROS
  ros::init(argc, argv, "poser");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 1000);
  geometry_msgs::Pose pose;
  ros::Rate rate(10);

  // get path prefix for camera calibration and ArUco board configuration
  std::string path = ros::package::getPath("poser");

  // start up the camera capture
  videoCapture.open(0);
  videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  cameraParameters.readFromXMLFile(path + "/dustin-calibration-2021-11-30.yml");

  // capture a frame to print the size to the console, should match the dimensions
  //  assigned above with videoCapture.set()
  videoCapture >> inputImage;
  cout << "width: " << inputImage.cols << " height: " <<
    inputImage.rows << std::endl;

  // configure the marker map tracker
  aruco::MarkerMap markerMap(path + "/Configuration.yml");
  aruco::DFCMarkerMapTracker tracker;
  tracker.setDictionary(markerMap.getDictionary()); 
  tracker.setParams(cameraParameters, markerMap);

  if (inputImage.empty()) cout << ">> Image empty!" << std::endl;
  if (!cameraParameters.isValid()) cout << "Camera parameters not valid!" << std::endl;

  while (ros::ok())
  {
    // take in an image and attempt to detect markers
    videoCapture.read(inputImage);
    if (inputImage.empty()) cout << "Image empty!" << std::endl;
    tracker.track(inputImage);
    bool estimatedPose = tracker.estimatePose();

    // send out the pose according to the first detected marker
    if (estimatedPose)
    {
      try
      {
	// show the identified tags
        //tracker.drawMarkers(inputImage);
        //cv::imshow("output", inputImage);
	//cv::waitKey(1);

        cv::Mat Tvec = tracker.getTvec();
        cv::Mat Rvec = tracker.getRvec();

        pose.position.x = Tvec.at<float>(0);
        pose.position.y = Tvec.at<float>(1);
        pose.position.z = Tvec.at<float>(2);
        // not actually sent as a quaternion, just euler. might fix later.
        pose.orientation.x = Rvec.at<float>(0);
        pose.orientation.y = Rvec.at<float>(1);
        pose.orientation.z = Rvec.at<float>(2);
        pose_pub.publish(pose);
      }
      catch (const std::exception& ex)
      {
        cerr << "drawing failed! " << __FILE__ << ":" << __LINE__ << endl;
	cerr << ex.what() << endl;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
