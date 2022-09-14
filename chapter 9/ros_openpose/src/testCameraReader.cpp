/**
 * testRosCameraReader.cpp: test file for CameraReader
 * Author: Ravi Joshi
 * Date: 2019/02/01
 */

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ros_openpose/cameraReader.hpp>

typedef std::shared_ptr<ros_openpose::CameraReader> sPtrCameraReader;

void show(sPtrCameraReader readers)
{
  ros::Rate loopRate(10);
  while (ros::ok())
  {
    auto colorImage = readers->getColorFrame();  //获取彩色图像
    auto depthImage = readers->getDepthFrame();  //获取深度图像

    if (!colorImage.empty())
      cv::imshow("color image", colorImage);
    else
      // display the error at most once per 10 seconds
      ROS_WARN_THROTTLE(10, "Empty color image frame detected. Ignoring...");

    if (!depthImage.empty())
      cv::imshow("depth image", depthImage);
    else
      // display the error at most once per 10 seconds
      ROS_WARN_THROTTLE(10, "Empty depth image frame detected. Ignoring...");  //输出深度图像

    int key = cv::waitKey(1) & 255;
    if (key == 27)  // escape key
      break;

    ros::spinOnce();
    loopRate.sleep();
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_camera_reader");
  ros::NodeHandle nh;

  const std::string colorTopic = "/camera/color/image_raw";                   //图像话题
  const std::string camInfoTopic = "/camera/color/camera_info";               //相机信息话题
  const std::string depthTopic = "/camera/aligned_depth_to_color/image_raw";  //深度图像话题
  const auto cameraReader = std::make_shared<ros_openpose::CameraReader>(nh, colorTopic, depthTopic, camInfoTopic);

  show(cameraReader);  //显示图像

  return 0;
}
