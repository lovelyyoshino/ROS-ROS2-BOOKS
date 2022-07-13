/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <darknet_ros/YoloObjectDetector.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");                                //节点句柄
  darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle); //创建yolo对象

  ros::spin();
  return 0;
}
