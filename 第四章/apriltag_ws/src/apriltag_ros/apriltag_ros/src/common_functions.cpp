/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/common_functions.h"
#include "image_geometry/pinhole_camera_model.h"

#include "common/homography.h"
#include "tagStandard52h13.h"
#include "tagStandard41h12.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCustom48h12.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"

namespace apriltag_ros
{

  TagDetector::TagDetector(ros::NodeHandle pnh) : family_(getAprilTagOption<std::string>(pnh, "tag_family", "tag36h11")),
                                                  threads_(getAprilTagOption<int>(pnh, "tag_threads", 4)),
                                                  decimate_(getAprilTagOption<double>(pnh, "tag_decimate", 1.0)),
                                                  blur_(getAprilTagOption<double>(pnh, "tag_blur", 0.0)),
                                                  refine_edges_(getAprilTagOption<int>(pnh, "tag_refine_edges", 1)),
                                                  debug_(getAprilTagOption<int>(pnh, "tag_debug", 0)),
                                                  max_hamming_distance_(getAprilTagOption<int>(pnh, "max_hamming_dist", 2)),
                                                  publish_tf_(getAprilTagOption<bool>(pnh, "publish_tf", false))
  {
    // 解析用户指定的独立标签描述(存储在ROS参数服务器上)
    XmlRpc::XmlRpcValue standalone_tag_descriptions;                   //设置对象为单一标签描述符
    if (!pnh.getParam("standalone_tags", standalone_tag_descriptions)) //获取参数standalone_tags，如果没有，则返回false
    {
      ROS_WARN("No april tags specified");
    }
    else
    {
      try
      {
        standalone_tag_descriptions_ =
            parseStandaloneTags(standalone_tag_descriptions); //解析standalone_tag_descriptions，返回一个vector<TagDescriptor>
      }
      catch (XmlRpc::XmlRpcException e)
      {
        // in case any of the asserts in parseStandaloneTags() fail
        ROS_ERROR_STREAM("Error loading standalone tag descriptions: " << e.getMessage().c_str());
      }
    }

    // 解析用户指定的标签束描述(存储在ROS参数服务器上)
    XmlRpc::XmlRpcValue tag_bundle_descriptions; //设置对象为标签组描述符
    if (!pnh.getParam("tag_bundles", tag_bundle_descriptions))
    {
      ROS_WARN("No tag bundles specified");
    }
    else
    {
      try
      {
        tag_bundle_descriptions_ = parseTagBundles(tag_bundle_descriptions); //解析tag_bundle_descriptions，返回一个vector<TagBundleDescriptor>
      }
      catch (XmlRpc::XmlRpcException e)
      {
        // In case any of the asserts in parseStandaloneTags() fail
        ROS_ERROR_STREAM("Error loading tag bundle descriptions: " << e.getMessage().c_str());
      }
    }

    // Optionally remove duplicate detections in scene. Defaults to removing
    if (!pnh.getParam("remove_duplicates", remove_duplicates_)) //可选地移除场景中的重复检测。默认删除
    {
      ROS_WARN("remove_duplicates parameter not provided. Defaulting to true");
      remove_duplicates_ = true; //设置默认值为true
    }

    // 定义在相机图像中搜索其标签的标签族
    if (family_ == "tagStandard52h13")
    {
      tf_ = tagStandard52h13_create(); //创建标签族
    }
    else if (family_ == "tagStandard41h12")
    {
      tf_ = tagStandard41h12_create();
    }
    else if (family_ == "tag36h11")
    {
      tf_ = tag36h11_create();
    }
    else if (family_ == "tag25h9")
    {
      tf_ = tag25h9_create();
    }
    else if (family_ == "tag16h5")
    {
      tf_ = tag16h5_create();
    }
    else if (family_ == "tagCustom48h12")
    {
      tf_ = tagCustom48h12_create();
    }
    else if (family_ == "tagCircle21h7")
    {
      tf_ = tagCircle21h7_create();
    }
    else if (family_ == "tagCircle49h12")
    {
      tf_ = tagCircle49h12_create();
    }
    else
    {
      ROS_WARN("Invalid tag family specified! Aborting");
      exit(1);
    }

    // Create the AprilTag 2 detector
    td_ = apriltag_detector_create();                                   //创建apriltag_detector_t类型的对象
    apriltag_detector_add_family_bits(td_, tf_, max_hamming_distance_); //将标签族添加到apriltag_detector_t类型的对象中
    td_->quad_decimate = (float)decimate_;                              //设置apriltag_detector_t类型的对象的quad_decimate属性
    td_->quad_sigma = (float)blur_;                                     //设置apriltag_detector_t类型的对象的quad_sigma属性
    td_->nthreads = threads_;                                           //设置apriltag_detector_t类型的对象的nthreads属性
    td_->debug = debug_;
    td_->refine_edges = refine_edges_; //设置apriltag_detector_t类型的对象的refine_edges属性

    detections_ = NULL;
  }

  // 析构函数
  TagDetector::~TagDetector()
  {
    // 释放与标记检测器关联的内存
    apriltag_detector_destroy(td_);

    // 释放与标记检测数组关联的内存
    if (detections_)
    {
      apriltag_detections_destroy(detections_);
    }

    // 释放与标签族相关的内存
    if (family_ == "tagStandard52h13")
    {
      tagStandard52h13_destroy(tf_);
    }
    else if (family_ == "tagStandard41h12")
    {
      tagStandard41h12_destroy(tf_);
    }
    else if (family_ == "tag36h11")
    {
      tag36h11_destroy(tf_);
    }
    else if (family_ == "tag25h9")
    {
      tag25h9_destroy(tf_);
    }
    else if (family_ == "tag16h5")
    {
      tag16h5_destroy(tf_);
    }
    else if (family_ == "tagCustom48h12")
    {
      tagCustom48h12_destroy(tf_);
    }
    else if (family_ == "tagCircle21h7")
    {
      tagCircle21h7_destroy(tf_);
    }
    else if (family_ == "tagCircle49h12")
    {
      tagCircle49h12_destroy(tf_);
    }
  }

  AprilTagDetectionArray TagDetector::detectTags(
      const cv_bridge::CvImagePtr &image,
      const sensor_msgs::CameraInfoConstPtr &camera_info)
  {
    // 将图像转换为AprilTag代码的格式
    cv::Mat gray_image;
    if (image->image.channels() == 1) //如果图像的通道数为1，则不需要转换
    {
      gray_image = image->image;
    }
    else
    {
      cv::cvtColor(image->image, gray_image, CV_BGR2GRAY); //将图像转换为灰度图像
    }
    image_u8_t apriltag_image = {.width = gray_image.cols,
                                 .height = gray_image.rows,
                                 .stride = gray_image.cols,
                                 .buf = gray_image.data}; //将灰度图像转换为apriltag_image_u8_t类型的对象

    image_geometry::PinholeCameraModel camera_model; //定义一个PinholeCameraModel类型的对象
    camera_model.fromCameraInfo(camera_info);        //将camera_info中的信息赋值给camera_model对象

    // 得到校正图像的相机固有属性
    double fx = camera_model.fx(); // focal length in camera x-direction [px]
    double fy = camera_model.fy(); // focal length in camera y-direction [px]
    double cx = camera_model.cx(); // optical center x-coordinate [px]
    double cy = camera_model.cy(); // optical center y-coordinate [px]

    // 对图像运行AprilTag 2算法
    if (detections_)
    {
      apriltag_detections_destroy(detections_);
      detections_ = NULL;
    }
    detections_ = apriltag_detector_detect(td_, &apriltag_image);

    // 如果remove_dulpicates_设置为true，则不允许重复标记。因此，场景中可见的任何重复标签id必须包含至少一个错误检测。
    // 删除任何具有重复id的标记，以确保删除这些错误的检测
    if (remove_duplicates_)
    {
      removeDuplicates();
    }

    // 为每个检测到的标签分别计算估计的平移和旋转
    AprilTagDetectionArray tag_detection_array;
    std::vector<std::string> detection_names; //定义一个字符串类型的vector类型的对象，用于存储检测到的标签的名称
    tag_detection_array.header = image->header;
    std::map<std::string, std::vector<cv::Point3d>> bundleObjectPoints; //定义一个map类型的对象，用于存储标签的3D点的坐标
    std::map<std::string, std::vector<cv::Point2d>> bundleImagePoints;  //定义一个map类型的对象，用于存储标签的2D点的坐标
    for (int i = 0; i < zarray_size(detections_); i++)
    {
      // Get the i-th detected tag
      apriltag_detection_t *detection;        //定义一个apriltag_detection_t类型的对象，用于存储检测到的标签的信息
      zarray_get(detections_, i, &detection); //将第i个检测到的标签的信息存储到detection中

      // 进入for循环，在标签包中找到这个标签的描述。
      // 如果找到，将它的点添加到cv::solvePnP的对象中。
      // 不过，暂时不要运行cv::solvePnP，因为需要收集所有世界坐标与图像对应的点
      int tagID = detection->id;
      bool is_part_of_bundle = false;
      for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++)
      {
        // 遍历已注册包
        TagBundleDescription bundle = tag_bundle_descriptions_[j];

        if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end())
        {
          // 这个检测到的标记属于第j个标记包(它的ID在包描述中找到)
          is_part_of_bundle = true;
          std::string bundleName = bundle.name();

          //===== 世界坐标系中的角点
          double s = bundle.memberSize(tagID) / 2;
          addObjectPoints(s, bundle.memberT_oi(tagID),
                          bundleObjectPoints[bundleName]);

          //===== 图像帧坐标中的角点
          addImagePoints(detection, bundleImagePoints[bundleName]);
        }
      }

      // 当发现一个既不是包的一部分也不是独立的标签时，
      // 打印警告(因此它是用户没有指定描述的环境中的标签，或者Apriltags错误检测到标签(错误ID或假阳性))。
      StandaloneTagDescription *standaloneDescription;
      if (!findStandaloneTagDescription(tagID, standaloneDescription,
                                        !is_part_of_bundle))
      {
        continue;
      }

      //=================================================================
      // 这个for循环的其余部分与独立的标签姿势有关!
      double tag_size = standaloneDescription->size();

      // Get estimated tag pose in the camera frame.
      //
      // Note on frames:
      // The raw AprilTag 2 uses the following frames:
      //   - camera frame: looking from behind the camera (like a
      //     photographer), x is right, y is up and z is towards you
      //     (i.e. the back of camera)
      //   - tag frame: looking straight at the tag (oriented correctly),
      //     x is right, y is down and z is away from you (into the tag).
      // But we want:
      //   - camera frame: looking from behind the camera (like a
      //     photographer), x is right, y is down and z is straight
      //     ahead
      //   - tag frame: looking straight at the tag (oriented correctly),
      //     x is right, y is up and z is towards you (out of the tag).
      // Using these frames together with cv::solvePnP directly avoids
      // AprilTag 2's frames altogether.
      // TODO solvePnP[Ransac] better?
      std::vector<cv::Point3d> standaloneTagObjectPoints;                           //定义一个vector类型的对象，用于存储标签的3D点的坐标
      std::vector<cv::Point2d> standaloneTagImagePoints;                            //定义一个vector类型的对象，用于存储标签的2D点的坐标
      addObjectPoints(tag_size / 2, cv::Matx44d::eye(), standaloneTagObjectPoints); //添加标签的三维点
      addImagePoints(detection, standaloneTagImagePoints);                          //将检测到的标签的信息存储到standaloneTagImagePoints中
      Eigen::Matrix4d transform = getRelativeTransform(standaloneTagObjectPoints,
                                                       standaloneTagImagePoints,
                                                       fx, fy, cx, cy); //计算标签的相对姿势
      Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);                //拿到旋转量
      Eigen::Quaternion<double> rot_quaternion(rot);

      geometry_msgs::PoseWithCovarianceStamped tag_pose =
          makeTagPose(transform, rot_quaternion, image->header); //计算标签的姿势

      // Add the detection to the back of the tag detection array
      AprilTagDetection tag_detection;
      tag_detection.pose = tag_pose;
      tag_detection.id.push_back(detection->id);
      tag_detection.size.push_back(tag_size);
      tag_detection_array.detections.push_back(tag_detection);
      detection_names.push_back(standaloneDescription->frame_name()); //将标签的名字存储到detection_names中
    }

    //=================================================================
    // 对于检测到至少一个成员标签的每个tag_bundle，估计tag_bundle的起始姿态

    for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++)
    {
      // Get bundle name
      std::string bundleName = tag_bundle_descriptions_[j].name(); //获取bundle的名字

      std::map<std::string,
               std::vector<cv::Point3d>>::iterator it =
          bundleObjectPoints.find(bundleName); //查找bundleObjectPoints中是否有bundleName的键值对
      if (it != bundleObjectPoints.end())
      {
        // 检测到这个bundle的一些成员标签，得到这个bundle的位置
        TagBundleDescription &bundle = tag_bundle_descriptions_[j];

        Eigen::Matrix4d transform =
            getRelativeTransform(bundleObjectPoints[bundleName],
                                 bundleImagePoints[bundleName], fx, fy, cx, cy);
        Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
        Eigen::Quaternion<double> rot_quaternion(rot);

        geometry_msgs::PoseWithCovarianceStamped bundle_pose =
            makeTagPose(transform, rot_quaternion, image->header);

        // Add the detection to the back of the tag detection array
        AprilTagDetection tag_detection;
        tag_detection.pose = bundle_pose;
        tag_detection.id = bundle.bundleIds();
        tag_detection.size = bundle.bundleSizes();
        tag_detection_array.detections.push_back(tag_detection);
        detection_names.push_back(bundle.name());
      }
    }

    // 如果设置了，则发布transform /tf主题
    if (publish_tf_)
    {
      for (unsigned int i = 0; i < tag_detection_array.detections.size(); i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.pose = tag_detection_array.detections[i].pose.pose.pose;
        pose.header = tag_detection_array.detections[i].pose.header;
        tf::Stamped<tf::Transform> tag_transform;
        tf::poseStampedMsgToTF(pose, tag_transform);
        tf_pub_.sendTransform(tf::StampedTransform(tag_transform,
                                                   tag_transform.stamp_,
                                                   image->header.frame_id,
                                                   detection_names[i]));
      }
    }

    return tag_detection_array;
  }

  int TagDetector::idComparison(const void *first, const void *second)
  {
    int id1 = ((apriltag_detection_t *)first)->id; //获取第一个标签的id
    int id2 = ((apriltag_detection_t *)second)->id;
    return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1); //如果id1小于id2，返回-1，如果id1等于id2，返回0，如果id1大于id2，返回1
  }

  void TagDetector::removeDuplicates()
  {
    zarray_sort(detections_, &idComparison); //按照id排序
    int count = 0;
    bool duplicate_detected = false;
    while (true)
    {
      if (count > zarray_size(detections_) - 1) //如果count大于detections_的大小减1，则跳出循环
      {
        // The entire detection set was parsed
        return;
      }
      apriltag_detection_t *detection;
      zarray_get(detections_, count, &detection); //获取detections_中的第count个标签
      int id_current = detection->id;
      // Default id_next value of -1 ensures that if the last detection
      // is a duplicated tag ID, it will get removed
      int id_next = -1;
      if (count < zarray_size(detections_) - 1)
      {
        zarray_get(detections_, count + 1, &detection); //获取detections_中的第count+1个标签
        id_next = detection->id;                        //获取id_next的值
      }
      //如果id_current等于id_next或者id_current不等于id_next，并且duplicate_detected为true，则删除该标签
      if (id_current == id_next || (id_current != id_next && duplicate_detected))
      {
        duplicate_detected = true; //设置duplicate_detected为true
        // 从检测数组中删除当前的标记检测
        int shuffle = 0;
        zarray_remove_index(detections_, count, shuffle);
        if (id_current != id_next) //如果id_current不等于id_next
        {
          ROS_WARN_STREAM("Pruning tag ID " << id_current << " because it "
                                                             "appears more than once in the image.");
          duplicate_detected = false; // Reset
        }
        continue;
      }
      else
      {
        count++;
      }
    }
  }

  void TagDetector::addObjectPoints(
      double s, cv::Matx44d T_oi, std::vector<cv::Point3d> &objectPoints) const
  {
    // 将标签角坐标添加到对象点向量中，从左下角开始逆时针方向
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, -s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, -s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, s, 0, 1));
    objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, s, 0, 1));
  }

  void TagDetector::addImagePoints(
      apriltag_detection_t *detection,
      std::vector<cv::Point2d> &imagePoints) const
  {
    // 将图像框中的标签角添加到图像点矢量中,从左下角开始逆时针方向
    double tag_x[4] = {-1, 1, 1, -1};
    double tag_y[4] = {1, 1, -1, -1}; // 取反，因为AprilTag标签局部帧的y轴指向下，而我们使用的标签局部帧的y轴指向上
    for (int i = 0; i < 4; i++)
    {
      // 单应投影取标签局部帧坐标到图像像素
      double im_x, im_y;
      homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
      imagePoints.push_back(cv::Point2d(im_x, im_y));
    }
  }

  Eigen::Matrix4d TagDetector::getRelativeTransform(
      std::vector<cv::Point3d> objectPoints,
      std::vector<cv::Point2d> imagePoints,
      double fx, double fy, double cx, double cy) const
  {
    // 利用上述3D-2D点对应进行视角n点相机姿态估计
    cv::Mat rvec, tvec;
    cv::Matx33d cameraMatrix(fx, 0, cx,
                             0, fy, cy,
                             0, 0, 1);
    cv::Vec4f distCoeffs(0, 0, 0, 0); // 畸变系数
    // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
    // need to first check WHAT is a bottleneck in this code, and only
    // do this if PnP solution is the bottleneck.
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec); //视角n点相机姿态估计
    cv::Matx33d R;
    cv::Rodrigues(rvec, R); //求旋转矩阵
    Eigen::Matrix3d wRo;
    wRo << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);

    Eigen::Matrix4d T; // homogeneous transformation matrix
    T.topLeftCorner(3, 3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2); //设置平移向量
    T.row(3) << 0, 0, 0, 1;
    return T;
  }

  geometry_msgs::PoseWithCovarianceStamped TagDetector::makeTagPose(
      const Eigen::Matrix4d &transform,
      const Eigen::Quaternion<double> rot_quaternion,
      const std_msgs::Header &header)
  {
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = header;
    //===== Position and orientation
    pose.pose.pose.position.x = transform(0, 3);
    pose.pose.pose.position.y = transform(1, 3);
    pose.pose.pose.position.z = transform(2, 3);
    pose.pose.pose.orientation.x = rot_quaternion.x();
    pose.pose.pose.orientation.y = rot_quaternion.y();
    pose.pose.pose.orientation.z = rot_quaternion.z();
    pose.pose.pose.orientation.w = rot_quaternion.w();
    return pose;
  }

  void TagDetector::drawDetections(cv_bridge::CvImagePtr image)
  {
    for (int i = 0; i < zarray_size(detections_); i++)
    {
      apriltag_detection_t *det;
      zarray_get(detections_, i, &det); //获取第i个标签的检测结果

      // Check if this ID is present in config/tags.yaml
      // Check if is part of a tag bundle
      int tagID = det->id;
      bool is_part_of_bundle = false;
      for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++)
      {
        TagBundleDescription bundle = tag_bundle_descriptions_[j];
        if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end()) //如果在bundle中找到了这个标签ID
        {
          is_part_of_bundle = true; //是一个标签组的一部分
          // break;
        }
      }
      // 如果不是bundle的一部分，请检查是否定义为独立标记
      // StandaloneTagDescription *standaloneDescription;
      // if (!is_part_of_bundle &&
      //     !findStandaloneTagDescription(tagID, standaloneDescription, false)) //如果不是bundle的一部分，请检查是否定义为独立标记
      // {
      // Neither a standalone tag nor part of a bundle, so this is a "rogue"
      // tag, skip it.
      // continue;
      // }

      // Draw tag outline with edge colors green, blue, blue, red
      // (going counter-clockwise, starting from lower-left corner in
      // tag coords). cv::Scalar(Blue, Green, Red) format for the edge
      // colors!
      // std::cout<<"draw lines"<<std::endl;
      // line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
      //      cv::Point((int)det->p[1][0], (int)det->p[1][1]),
      //      cv::Scalar(0, 0xff, 0)); // green
      // line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
      //      cv::Point((int)det->p[3][0], (int)det->p[3][1]),
      //      cv::Scalar(0, 0, 0xff)); // red
      // line(image->image, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
      //      cv::Point((int)det->p[2][0], (int)det->p[2][1]),
      //      cv::Scalar(0xff, 0, 0)); // blue
      // line(image->image, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
      //      cv::Point((int)det->p[3][0], (int)det->p[3][1]),
      //      cv::Scalar(0xff, 0, 0)); // blue

      // 在标签中间打印标签ID
      std::stringstream ss;
      ss << det->id;
      cv::String text = ss.str();
      int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
      double fontscale = 0.5;
      int baseline;
      cv::Size textsize = cv::getTextSize(text, fontface,
                                          fontscale, 2, &baseline); //获取文字的大小
      cv::putText(image->image, text,
                  cv::Point((int)(det->c[0] - textsize.width / 2),
                            (int)(det->c[1] + textsize.height / 2)),
                  fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 4);
    }
  }

  // 解析独立标签描述
  std::map<int, StandaloneTagDescription> TagDetector::parseStandaloneTags(
      XmlRpc::XmlRpcValue &standalone_tags)
  {
    // 创建将由函数填充并在最后返回的映射
    std::map<int, StandaloneTagDescription> descriptions;
    // 确保类型正确
    ROS_ASSERT(standalone_tags.getType() == XmlRpc::XmlRpcValue::TypeArray);
    // 遍历所有标签描述
    for (int32_t i = 0; i < standalone_tags.size(); i++)
    {

      // 第i个标签描述
      XmlRpc::XmlRpcValue &tag_description = standalone_tags[i];

      // 断言标签描述是一个结构体
      ROS_ASSERT(tag_description.getType() ==
                 XmlRpc::XmlRpcValue::TypeStruct);
      // 字段“id”的断言类型是一个int
      ROS_ASSERT(tag_description["id"].getType() ==
                 XmlRpc::XmlRpcValue::TypeInt);
      // 字段“size”的断言类型是双精度类型
      ROS_ASSERT(tag_description["size"].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);

      int id = (int)tag_description["id"]; // tag id
      // Tag size (square, side length in meters)
      double size = (double)tag_description["size"];

      // 自定义帧名称，如果该标记存在这样的字段
      std::string frame_name;
      if (tag_description.hasMember("name"))
      {
        // Assert type of field "name" is a string
        ROS_ASSERT(tag_description["name"].getType() ==
                   XmlRpc::XmlRpcValue::TypeString);
        frame_name = (std::string)tag_description["name"];
      }
      else
      {
        std::stringstream frame_name_stream;
        frame_name_stream << "tag_" << id;
        frame_name = frame_name_stream.str();
      }

      StandaloneTagDescription description(id, size, frame_name); //标签描述类的构造函数
      ROS_INFO_STREAM("Loaded tag config: " << id << ", size: " << size << ", frame_name: " << frame_name.c_str());
      // Add this tag's description to map of descriptions
      descriptions.insert(std::make_pair(id, description)); //将标签描述插入到映射中
    }

    return descriptions;
  }

  // 解析标记包描述
  std::vector<TagBundleDescription> TagDetector::parseTagBundles(
      XmlRpc::XmlRpcValue &tag_bundles)
  {
    std::vector<TagBundleDescription> descriptions;
    ROS_ASSERT(tag_bundles.getType() == XmlRpc::XmlRpcValue::TypeArray); //断言类型是数组

    // 遍历所有标记包描述
    for (int32_t i = 0; i < tag_bundles.size(); i++)
    {
      ROS_ASSERT(tag_bundles[i].getType() == XmlRpc::XmlRpcValue::TypeStruct); //断言类型是结构体
      // i-th tag bundle description
      XmlRpc::XmlRpcValue &bundle_description = tag_bundles[i];

      std::string bundleName;
      if (bundle_description.hasMember("name")) //如果存在这样的字段
      {
        ROS_ASSERT(bundle_description["name"].getType() ==
                   XmlRpc::XmlRpcValue::TypeString);          //断言类型是字符串
        bundleName = (std::string)bundle_description["name"]; //获取字段的值
      }
      else
      {
        std::stringstream bundle_name_stream;
        bundle_name_stream << "bundle_" << i;
        bundleName = bundle_name_stream.str();
      }
      TagBundleDescription bundle_i(bundleName); //标记包描述类的构造函数
      ROS_INFO("Loading tag bundle '%s'", bundle_i.name().c_str());

      ROS_ASSERT(bundle_description["layout"].getType() ==
                 XmlRpc::XmlRpcValue::TypeArray);
      XmlRpc::XmlRpcValue &member_tags = bundle_description["layout"]; //获取字段的值

      // 遍历所有标记包中的标记
      for (int32_t j = 0; j < member_tags.size(); j++)
      {
        ROS_ASSERT(member_tags[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        XmlRpc::XmlRpcValue &tag = member_tags[j];

        ROS_ASSERT(tag["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        int id = tag["id"];

        ROS_ASSERT(tag["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double size = tag["size"];

        // 自定义帧名称，如果该标记存在这样的字段
        StandaloneTagDescription *standaloneDescription;
        if (findStandaloneTagDescription(id, standaloneDescription, false))
        {
          ROS_ASSERT(size == standaloneDescription->size());
        }

        //获取这个标签相对于bundle原点的姿态
        double x = xmlRpcGetDoubleWithDefault(tag, "x", 0.);
        double y = xmlRpcGetDoubleWithDefault(tag, "y", 0.);
        double z = xmlRpcGetDoubleWithDefault(tag, "z", 0.);
        double qw = xmlRpcGetDoubleWithDefault(tag, "qw", 1.);
        double qx = xmlRpcGetDoubleWithDefault(tag, "qx", 0.);
        double qy = xmlRpcGetDoubleWithDefault(tag, "qy", 0.);
        double qz = xmlRpcGetDoubleWithDefault(tag, "qz", 0.);
        Eigen::Quaterniond q_tag(qw, qx, qy, qz);
        q_tag.normalize();
        Eigen::Matrix3d R_oi = q_tag.toRotationMatrix();

        // 构建从tag_j到束原点的刚性转换
        cv::Matx44d T_mj(R_oi(0, 0), R_oi(0, 1), R_oi(0, 2), x,
                         R_oi(1, 0), R_oi(1, 1), R_oi(1, 2), y,
                         R_oi(2, 0), R_oi(2, 1), R_oi(2, 2), z,
                         0, 0, 0, 1);

        // 注册标记成员
        bundle_i.addMemberTag(id, size, T_mj);
        ROS_INFO_STREAM(" " << j << ") id: " << id << ", size: " << size << ", "
                            << "p = [" << x << "," << y << "," << z << "], "
                            << "q = [" << qw << "," << qx << "," << qy << ","
                            << qz << "]");
      }
      descriptions.push_back(bundle_i);
    }
    return descriptions;
  }

  double TagDetector::xmlRpcGetDouble(XmlRpc::XmlRpcValue &xmlValue,
                                      std::string field) const
  {
    ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
               (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
    if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int tmp = xmlValue[field];
      return (double)tmp;
    }
    else
    {
      return xmlValue[field];
    }
  }

  double TagDetector::xmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue &xmlValue,
                                                 std::string field,
                                                 double defaultValue) const
  {
    if (xmlValue.hasMember(field))
    {
      ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
                 (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
      if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        int tmp = xmlValue[field];
        return (double)tmp;
      }
      else
      {
        return xmlValue[field];
      }
    }
    else
    {
      return defaultValue;
    }
  }
  //查找自定义标记描述类
  bool TagDetector::findStandaloneTagDescription(
      int id, StandaloneTagDescription *&descriptionContainer, bool printWarning)
  {
    std::map<int, StandaloneTagDescription>::iterator description_itr =
        standalone_tag_descriptions_.find(id);                 //查找自定义标记描述类
    if (description_itr == standalone_tag_descriptions_.end()) //如果没有找到
    {
      if (printWarning)
      {
        ROS_WARN_THROTTLE(10.0, "Requested description of standalone tag ID [%d],"
                                " but no description was found...",
                          id);
      }
      return false;
    }
    descriptionContainer = &(description_itr->second); //获取自定义标记描述类
    return true;
  }

} // namespace apriltag_ros
