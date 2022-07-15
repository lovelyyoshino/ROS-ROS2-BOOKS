#include <ros/ros.h>
#include <motion_decoder/image_converter.hpp>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

ImageConverter *ic;

void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
  ROS_INFO("In subscribe\n");
  // msg.id, msg.size, msg.pose
  static tf::TransformBroadcaster br; // 广播器
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg.detections[0].pose.pose.position.x, msg.detections[0].pose.pose.position.y, msg.detections[0].pose.pose.position.z)); // 广播位置
  tf::Quaternion quat_tf;
  quaternionMsgToTF(msg.detections[0].pose.pose.orientation, quat_tf); //四元数转换为tf::Quaternion
  transform.setRotation(quat_tf);                                      //设置旋转矩阵

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "april_tf")); // 旋转矩阵变换，转到当前的位置
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter"); //初始化节点

  ros::NodeHandle n;
  // TODO: Add a subscriber to get the AprilTag detections The callback function skelton is given.
  ros::Subscriber sub = n.subscribe("tag_detections", 1000, apriltag_detection_callback); //订阅apriltag_detection_callback函数

  ImageConverter converter; //创建ImageConverter类对象
  ic = &converter;          //将ImageConverter类对象的指针传给ic指针
  ros::Rate loop_rate(50);
  ROS_INFO("In main\n");
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
