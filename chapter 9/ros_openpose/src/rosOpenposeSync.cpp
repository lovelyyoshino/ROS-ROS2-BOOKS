#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>
#include <ros_openpose/Frame.h>
#include <ros_openpose/BodyPart.h>

// define a macro for compatibility with older versions
#define OPENPOSE1POINT6_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 6
#define OPENPOSE1POINT7POINT1_OR_HIGHER                                                                                \
  OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 7 && OpenPose_VERSION_PATCH >= 1

using namespace sensor_msgs;
class rosOpenPose
{
private:
  op::Wrapper* _op_wrapper;  //定义一个op::Wrapper类的指针,来打包数据
  ros::NodeHandle* _nh;      // Node信息
  ros::Publisher _pub;
  message_filters::Subscriber<Image> _color_sub;  //订阅彩色图像信息
  message_filters::Subscriber<Image> _depth_sub;  //订阅深度图像信息
  typedef message_filters::sync_policies::ApproximateTime<Image, Image> ColorDepthSyncPolicy;  //定义一个同步策略类型
  typedef message_filters::Synchronizer<ColorDepthSyncPolicy> ColorDepthSync;  //定义一个同步类型
  std::shared_ptr<ColorDepthSync> _sync;                                       //定义一个同步指针

  bool _no_depth;
  float _fx, _fy, _cx, _cy;  //相机内参
  float _mm_to_m;            //毫米转化为米

  cv::Mat _color_img, _depth_img;  //定义一个cv::Mat类型的变量,用于存储彩色图像和深度图像
  ros_openpose::Frame _frame_msg;  //定义一个ros_openpose::Frame类型的变量,用于存储OpenPose返回的数据

public:
  rosOpenPose(ros::NodeHandle* nh, op::Wrapper* op_wrapper, const std::string& color_topic,
              const std::string& depth_topic, const std::string& cam_info_topic, const std::string& pub_topic,
              const std::string& frame_id, const bool& no_depth)
    : _nh(nh), _op_wrapper(op_wrapper), _no_depth(no_depth)
  {
    _frame_msg.header.frame_id = frame_id;  //设置帧frame_id

    // Populate camera intrinsic matrix values.
    auto cam_info = ros::topic::waitForMessage<CameraInfo>(cam_info_topic);  //等待相机信息信息
    _fx = cam_info->K.at(0);                                                 //获取相机内参的fx值
    _fy = cam_info->K.at(4);                                                 //获取相机内参的fy值
    _cx = cam_info->K.at(2);                                                 //获取相机内参的cx值
    _cy = cam_info->K.at(5);                                                 //获取相机内参的cy值

    // Obtain depth encoding.
    auto depth_encoding = ros::topic::waitForMessage<Image>(depth_topic)->encoding;  //等待深度图像信息
    _mm_to_m = (depth_encoding == image_encodings::TYPE_16UC1) ?
                   0.001 :
                   1.;  //判断深度图像的编码类型,如果是16UC1,则设置毫米转化为米的值为0.001,否则设置为1.

    // Initialize frame publisher
    _pub = _nh->advertise<ros_openpose::Frame>(pub_topic, 10);  //初始化发布器

    // Start color & depth subscribers.
    _color_sub.subscribe(*_nh, color_topic, 1);
    _depth_sub.subscribe(*_nh, depth_topic, 1);
    _sync.reset(new ColorDepthSync(ColorDepthSync(10), _color_sub, _depth_sub));  //初始化同步类型
    _sync->registerCallback(boost::bind(&rosOpenPose::callback, this, _1, _2));   //注册回调函数
  }

  template <typename key_points>
  void assign_msg_vals(ros_openpose::BodyPart& part, const key_points& kp, const int& i)
  {
    // 从Openpose分配像素位置和评分。
    float u = kp[i], v = kp[i + 1], s = kp[i + 2];  //获取像素位置和评分
    part.pixel.x = u;                               //设置像素位置x值
    part.pixel.y = v;                               //设置像素位置y值
    part.score = s;                                 //设置评分

    // 如果深度提供,计算三维姿态
    if (!_no_depth)
    {
      auto depth = _depth_img.at<float>(static_cast<int>(v), static_cast<int>(u)) * _mm_to_m;  //获取深度图像的像素值
      if (depth <= 0)
        return;
      part.point.x = (depth / _fx) * (u - _cx);  //计算三维位置x值
      part.point.y = (depth / _fy) * (v - _cy);  //计算三维位置y值
      part.point.z = depth;                      //计算三维位置z值
    }
  }

  void callback(const ImageConstPtr& color_msg, const ImageConstPtr& depth_msg)
  {
    _frame_msg.header.stamp = ros::Time::now();
    _frame_msg.persons.clear();

    _color_img = cv_bridge::toCvShare(color_msg, image_encodings::BGR8)->image;        //获取彩色图像信息
    _depth_img = cv_bridge::toCvShare(depth_msg, image_encodings::TYPE_32FC1)->image;  //获取深度图像信息

    // Fill datum
#if OPENPOSE1POINT6_OR_HIGHER                                                   //如果是OpenPose1.6版本以上版本
    auto datum_ptr = _op_wrapper->emplaceAndPop(OP_CV2OPCONSTMAT(_color_img));  //获取OpenPose返回的数据
#else
    auto datum_ptr = _op_wrapper->emplaceAndPop(_color_img);  //获取OpenPose返回的数据
#endif

    const auto& pose_kp = datum_ptr->at(0)->poseKeypoints;  //获取OpenPose返回的数据中的人体关键点信息
    const auto& hand_kp = datum_ptr->at(0)->handKeypoints;  //获取OpenPose返回的数据中的手体关键点信息

    // get the size
    const auto num_persons = pose_kp.getSize(0);         //获取行人的个数
    const auto body_part_count = pose_kp.getSize(1);     //获取人体关键点信息
    const auto hand_part_count = hand_kp[0].getSize(1);  //获取手关键点信息

    _frame_msg.persons.resize(num_persons);  //设置行人的个数
    int i;
    for (auto p = 0; p < num_persons; p++)
    {
      auto& curr_person = _frame_msg.persons[p];  //当前的行人信息

      curr_person.bodyParts.resize(body_part_count);       //设置人体关键点信息
      curr_person.leftHandParts.resize(hand_part_count);   //设置左手关键点信息
      curr_person.rightHandParts.resize(hand_part_count);  //设置右手关键点信息

      // Fill body parts
      for (auto bp = 0; bp < body_part_count; bp++)  //获取人体关键点信息
      {
        auto& curr_body_part = curr_person.bodyParts[bp];     //获取当前的人体关键点信息
        i = pose_kp.getSize(2) * (p * body_part_count + bp);  //获取当前人体关键点信息的索引
        assign_msg_vals(curr_body_part, pose_kp, i);          //设置人体关键点信息
      }

      // Fill left and right hands
      for (auto hp = 0; hp < hand_part_count; hp++)  //获取左手关键点信息
      {
        i = hand_kp[0].getSize(2) * (p * hand_part_count + hp);  //获取当前左手关键点信息的索引

        // Left Hand
        auto& curr_left_hand = curr_person.leftHandParts[hp];  //获取当前左手关键点信息
        assign_msg_vals(curr_left_hand, hand_kp[0], i);        //设置左手关键点信息

        // Right Hand
        auto& curr_right_hand = curr_person.rightHandParts[hp];  //获取当前右手关键点信息
        assign_msg_vals(curr_right_hand, hand_kp[1], i);         //设置右手关键点信息
      }
    }
    _pub.publish(_frame_msg);
  }
};

void configureOpenPose(op::Wrapper& opWrapper)  //配置Openpose
{
  try
  {
#if OPENPOSE1POINT6_OR_HIGHER
    op::checkBool(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#else
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#endif
                  "Wrong logging_level value.", __LINE__, __FUNCTION__, __FILE__);  //检查日志等级是否正确

    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);  //设置日志等级
    op::Profiler::setDefaultX(FLAGS_profile_speed);                             //设置默认的时间间隔

// Applying user defined configuration - GFlags to program variables
// outputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
#else
    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");  //获取输出的尺寸
#endif

// netInputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
#else
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");  //获取网络输入的尺寸
#endif

// faceNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
#else
    const auto faceNetInputSize =
        op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");  //获取人脸网络输入的尺寸
#endif

// handNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
    const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
#else
    const auto handNetInputSize =
        op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");  //获取手网络输入的尺寸
#endif

    // poseMode
    const auto poseMode = op::flagsToPoseMode(FLAGS_body);  //获取人体模式

// poseModel
#if OPENPOSE1POINT6_OR_HIGHER
    const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
#else
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);                 //获取人体模型
#endif

    // JSON saving
    if (!FLAGS_write_keypoint.empty())
      ROS_INFO(
          "Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json` instead.");  //输出提示信息

    // keypointScaleMode
    const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);  //获取关键点尺寸模式

    // heatmaps to add
    const auto heatMapTypes =
        op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg, FLAGS_heatmaps_add_PAFs);  //获取热图类型

    const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);  //获取热图尺寸模式

    // >1 camera view?
    // const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
    const auto multipleView = false;  //多视角

    // Face and hand detectors
    const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);  //获取人脸检测器
    const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);  //获取手检测器

    // Enabling Google Logging
    const bool enableGoogleLogging = true;  //启用Google日志

    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose
    {
      poseMode, netInputSize,  //获取人体模式和网络输入尺寸
#if OPENPOSE1POINT7POINT1_OR_HIGHER
          FLAGS_net_resolution_dynamic, outputSize,  //获取网络输出尺寸
#else
          outputSize,
#endif
          keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start, FLAGS_scale_number,
          (float)FLAGS_scale_gap,  //获取关键点尺寸模式、GPU数量、GPU起始索引、尺寸缩放数量和尺寸缩放间隔
          op::flagsToRenderMode(FLAGS_render_pose, multipleView), poseModel,
          !FLAGS_disable_blending,  //获取渲染模式、人体模型、是否启用混合模式
          (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
          FLAGS_part_to_show,  //获取透明度、热图透明度、显示的关键点
#if OPENPOSE1POINT6_OR_HIGHER
          op::String(FLAGS_model_folder),
#else
          FLAGS_model_folder,                                                //获取模型文件夹
#endif
          heatMapTypes, heatMapScaleMode, FLAGS_part_candidates, (float)FLAGS_render_threshold, FLAGS_number_people_max,
          FLAGS_maximize_positives,
          FLAGS_fps_max,  //获取热图类型、热图尺寸模式、关键点候选、渲染阈值、最大人数、最大匹配程度、最大帧率
#if OPENPOSE1POINT6_OR_HIGHER
          op::String(FLAGS_prototxt_path), op::String(FLAGS_caffemodel_path),
#else
          FLAGS_prototxt_path, FLAGS_caffemodel_path,                        //获取prototxt路径、caffemodel路径
#endif
          (float)FLAGS_upsampling_ratio, enableGoogleLogging  //获取上采样比例、是否启用Google日志
    };
    opWrapper.configure(wrapperStructPose);

    // Face configuration (use op::WrapperStructFace{} to disable it)
    const op::WrapperStructFace wrapperStructFace{
        FLAGS_face,        //是否启用人脸检测
        faceDetector,      //人脸检测器
        faceNetInputSize,  //人脸网络输入尺寸
        op::flagsToRenderMode(FLAGS_face_render, multipleView,
                              FLAGS_render_pose),  //获取渲染模式、是否多视角、是否渲染人体姿态
        (float)FLAGS_face_alpha_pose,
        (float)FLAGS_face_alpha_heatmap,
        (float)FLAGS_face_render_threshold};  //获取人脸渲染模式、人脸透明度、热图透明度、渲染阈值
    opWrapper.configure(wrapperStructFace);  //配置人脸检测器

    // Hand configuration (use op::WrapperStructHand{} to disable it)
    const op::WrapperStructHand wrapperStructHand{
        FLAGS_hand,                     //是否启用手检测
        handDetector,                   //手检测器
        handNetInputSize,               //手网络输入尺寸
        FLAGS_hand_scale_number,        //手尺寸缩放数量
        (float)FLAGS_hand_scale_range,  //手尺寸缩放范围
        op::flagsToRenderMode(FLAGS_hand_render, multipleView,
                              FLAGS_render_pose),  //获取渲染模式、是否多视角、是否渲染人体姿态
        (float)FLAGS_hand_alpha_pose,              //手透明度
        (float)FLAGS_hand_alpha_heatmap,           //热图透明度
        (float)FLAGS_hand_render_threshold};       //渲染阈值
    opWrapper.configure(wrapperStructHand);

    // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
    const op::WrapperStructExtra wrapperStructExtra{
        FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking,
        FLAGS_ik_threads};  //获取3D检测、最小视角数、是否启用识别、是否启用跟踪、IK线程数
    opWrapper.configure(wrapperStructExtra);  //配置额外功能

    // Output (comment or use default argument to disable any output)
    const op::WrapperStructOutput wrapperStructOutput
    {
      FLAGS_cli_verbose,  //获取是否显示详细信息
#if OPENPOSE1POINT6_OR_HIGHER
          op::String(FLAGS_write_keypoint),
#else
          FLAGS_write_keypoint,                                              //获取是否保存关键点
#endif
          op::stringToDataFormat(FLAGS_write_keypoint_format),  //获取关键点保存格式
#if OPENPOSE1POINT6_OR_HIGHER
          op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json),
#else
          FLAGS_write_json, FLAGS_write_coco_json,                           //获取是否保存json、coco json
#endif
          FLAGS_write_coco_json_variants, FLAGS_write_coco_json_variant,  //获取coco json变体、变体类型
#if OPENPOSE1POINT6_OR_HIGHER
          op::String(FLAGS_write_images), op::String(FLAGS_write_images_format), op::String(FLAGS_write_video),
#else
          FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video,  //获取是否保存图片、图片格式、是否保存视频
#endif
          FLAGS_write_video_fps, FLAGS_write_video_with_audio,  //获取视频帧率、是否保存音频
#if OPENPOSE1POINT6_OR_HIGHER
          op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
          op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
          op::String(FLAGS_udp_port)
    };
#else
          FLAGS_write_heatmaps, FLAGS_write_heatmaps_format, FLAGS_write_video_3d, FLAGS_write_video_adam,
          FLAGS_write_bvh, FLAGS_udp_host,
          FLAGS_udp_port  //获取是否保存热图、热图格式、是否保存3D视频、是否保存Adam视频、是否保存BVH、UDP主机、UDP端口
    };
#endif
    opWrapper.configure(wrapperStructOutput);  //配置输出

    // GUI (comment or use default argument to disable any visual output)
    const op::WrapperStructGui wrapperStructGui{op::flagsToDisplayMode(FLAGS_display, FLAGS_3d), !FLAGS_no_gui_verbose,
                                                FLAGS_fullscreen};  //获取显示模式、是否显示详细信息、是否全屏
    opWrapper.configure(wrapperStructGui);                          //配置GUI
    // clang-format on

    // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
    if (FLAGS_disable_multi_thread)       //获取是否禁用多线程
      opWrapper.disableMultiThreading();  //禁用多线程
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_openpose_synchronous");  // ros节点名称
  ros::NodeHandle nh("~");

  // Get params
  bool no_depth;
  std::string color_topic, depth_topic, cam_info_topic, pub_topic, frame_id;
  nh.getParam("color_topic", color_topic);        //获取rgb图像信息
  nh.getParam("depth_topic", depth_topic);        //获取深度信息
  nh.getParam("cam_info_topic", cam_info_topic);  //获取相机信息
  nh.getParam("pub_topic", pub_topic);            //获取发布的topic名称
  nh.getParam("frame_id", frame_id);              //获取发布的frame_id名称
  nh.param("no_depth", no_depth, false);          //是否有深度图像，一般是有的 default value is false

  // Parse Openpose Args
  gflags::ParseCommandLineFlags(&argc, &argv, true);  //解析命令行参数

  try
  {
    ROS_INFO("Starting ros_openpose...");

    // Initialize Openpose wrapper
    op::Wrapper op_wrapper{op::ThreadManagerMode::Asynchronous};  //异步模式
    configureOpenPose(op_wrapper);                                //配置OpenPose
    op_wrapper.start();                                           //开始OpenPose

    // Start ROS wrapper
    rosOpenPose rop(&nh, &op_wrapper, color_topic, depth_topic, cam_info_topic, pub_topic, frame_id,
                    no_depth);  //初始化ros_openpose类

    ros::spin();

    ROS_INFO("Exiting ros_openpose...");

    op_wrapper.stop();

    return 0;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
    return -1;
  }
}