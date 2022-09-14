/*
 *   Author: Timon Homberger
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <darknet_ros/YoloObjectDetector.hpp>

class DarknetRosNodelet : public nodelet::Nodelet
{
public:
  DarknetRosNodelet() = default; //构造函数
  ~DarknetRosNodelet()
  {
    if (darknetRos_)
      delete darknetRos_;
  } //析构函数

private:
  virtual void onInit()
  {
    ros::NodeHandle NodeHandle("~");
    NodeHandle = getPrivateNodeHandle();                           //获取私有节点句柄
    darknetRos_ = new darknet_ros::YoloObjectDetector(NodeHandle); //创建YoloObjectDetector对象
  }

  darknet_ros::YoloObjectDetector *darknetRos_;
};

// Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(DarknetRosNodelet, nodelet::Nodelet);