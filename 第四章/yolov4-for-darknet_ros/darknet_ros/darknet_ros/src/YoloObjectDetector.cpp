/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

// yolo object detector
#include "darknet_ros/YoloObjectDetector.hpp"

// Check for xServer
#include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros
{

  char *cfg;
  char *weights;
  char *data;
  char **detectionNames;

  YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
      : nodeHandle_(nh), imageTransport_(nodeHandle_), numClasses_(0), classLabels_(0), rosBoxes_(0), rosBoxCounter_(0) //初始化
  {
    ROS_INFO("[YoloObjectDetector] Node started.");

    // Read parameters from config file.
    if (!readParameters())
    {
      ros::requestShutdown(); //关闭节点
    }

    init();
  }

  YoloObjectDetector::~YoloObjectDetector()
  {
    {
      boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_); //加锁
      isNodeRunning_ = false;
    }
    yoloThread_.join();
  }

  bool YoloObjectDetector::readParameters() //读取参数
  {
    // Load common parameters.
    nodeHandle_.param("image_view/enable_opencv", viewImage_, true);                    //是否显示图像
    nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);                   //等待时间
    nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false); //是否显示控制台输出

    // Check if Xserver is running on Linux.
    if (XOpenDisplay(NULL))
    {
      // Do nothing!
      ROS_INFO("[YoloObjectDetector] Xserver is running.");
    }
    else
    {
      ROS_INFO("[YoloObjectDetector] Xserver is not running.");
      viewImage_ = false;
    }

    // Set vector sizes.
    nodeHandle_.param("yolo_model/detection_classes/names", classLabels_, std::vector<std::string>(0)); //类别名称
    numClasses_ = classLabels_.size();                                                                  //类别数量
    rosBoxes_ = std::vector<std::vector<RosBox_>>(numClasses_);                                         //每个类别的框
    rosBoxCounter_ = std::vector<int>(numClasses_);                                                     //每个类别的框数量

    return true;
  }

  void YoloObjectDetector::init()
  {
    ROS_INFO("[YoloObjectDetector] init().");

    // Initialize deep network of darknet.
    std::string weightsPath;
    std::string configPath;
    std::string dataPath;
    std::string configModel;
    std::string weightsModel;

    // Threshold of object detection.
    float thresh;
    nodeHandle_.param("yolo_model/threshold/value", thresh, (float)0.3); //阈值

    // Path to weights file.
    nodeHandle_.param("yolo_model/weight_file/name", weightsModel, std::string("yolov2-tiny.weights")); //模型文件名称
    nodeHandle_.param("weights_path", weightsPath, std::string("/default"));                            //模型文件路径
    weightsPath += "/" + weightsModel;
    weights = new char[weightsPath.length() + 1];
    strcpy(weights, weightsPath.c_str()); //模型文件路径

    // Path to config file.
    nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("yolov2-tiny.cfg")); //配置文件名称
    nodeHandle_.param("config_path", configPath, std::string("/default"));                         //配置文件路径
    configPath += "/" + configModel;
    cfg = new char[configPath.length() + 1]; //配置文件路径
    strcpy(cfg, configPath.c_str());         // 配置文件路径

    // Path to data folder.
    dataPath = darknetFilePath_; //模型文件路径
    dataPath += "/data";
    data = new char[dataPath.length() + 1]; //模型文件路径
    strcpy(data, dataPath.c_str());         //模型文件路径

    // Get classes.
    detectionNames = (char **)realloc((void *)detectionNames, (numClasses_ + 1) * sizeof(char *)); //类别名称数组
    for (int i = 0; i < numClasses_; i++)
    {
      detectionNames[i] = new char[classLabels_[i].length() + 1]; //类别名称数组
      strcpy(detectionNames[i], classLabels_[i].c_str());         //类别名称数组
    }

    // Load network.
    setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_, 0, 0, 1, 0.5, 0, 0, 0, 0); //加载模型
    yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);                                      //创建线程

    // Initialize publisher and subscriber.
    std::string cameraTopicName;         //相机话题名称
    int cameraQueueSize;                 //相机队列大小
    std::string objectDetectorTopicName; //检测器话题名称
    int objectDetectorQueueSize;         //检测器队列大小
    bool objectDetectorLatch;            // 检测器是否阻塞
    std::string boundingBoxesTopicName;  //框话题名称
    int boundingBoxesQueueSize;          //框队列大小
    bool boundingBoxesLatch;             //框是否阻塞
    std::string detectionImageTopicName; //图像话题名称
    int detectionImageQueueSize;         //图像队列大小
    bool detectionImageLatch;            //图像是否阻塞

    nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/camera/image_raw"));       //订阅图像topic
    nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);                                 //订阅图像队列大小
    nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName, std::string("found_object"));    //发布检测结果topic
    nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);                         //发布检测结果队列大小
    nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);                              //发布检测结果是否阻塞
    nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName, std::string("bounding_boxes"));    //发布框话题名称
    nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);                           //发布框队列大小
    nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);                                //发布框是否阻塞
    nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName, std::string("detection_image")); //发布图像话题名称
    nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);                         //发布图像队列大小
    nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);                               //发布图像是否阻塞

    imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize, &YoloObjectDetector::cameraCallback, this);
    objectPublisher_ =
        nodeHandle_.advertise<darknet_ros_msgs::ObjectCount>(objectDetectorTopicName, objectDetectorQueueSize, objectDetectorLatch);
    boundingBoxesPublisher_ =
        nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
    detectionImagePublisher_ =
        nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName, detectionImageQueueSize, detectionImageLatch);

    // Action servers.
    std::string checkForObjectsActionName;
    nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName, std::string("check_for_objects"));
    checkForObjectsActionServer_.reset(new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));            //创建action服务器
    checkForObjectsActionServer_->registerGoalCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));       //注册action服务器的目标回调函数
    checkForObjectsActionServer_->registerPreemptCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this)); //注册action服务器的抢占回调函数
    checkForObjectsActionServer_->start();
  }

  void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr &msg) // 相机回调函数
  {
    ROS_DEBUG("[YoloObjectDetector] USB image received.");

    cv_bridge::CvImagePtr cam_image;

    try
    {
      cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //将图像转换为OpenCV格式
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cam_image)
    {
      {
        boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_); //加锁
        imageHeader_ = msg->header;                                                     //获取图像头信息
        camImageCopy_ = cam_image->image.clone();                                       //复制图像
      }
      {
        boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_); //加锁
        imageStatus_ = true;                                                        //图像已经接收到
      }
      frameWidth_ = cam_image->image.size().width;   //获取图像宽度
      frameHeight_ = cam_image->image.size().height; //获取图像高度
    }
    return;
  }
  //检测图像是否接收到
  void YoloObjectDetector::checkForObjectsActionGoalCB()
  {
    ROS_DEBUG("[YoloObjectDetector] Start check for objects action.");

    boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> imageActionPtr = checkForObjectsActionServer_->acceptNewGoal(); //接收action服务器的目标
    sensor_msgs::Image imageAction = imageActionPtr->image;                                                                        //获取图像信息

    cv_bridge::CvImagePtr cam_image;

    try
    {
      cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8); //将图像转换为OpenCV格式
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cam_image)
    {
      {
        boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_); //加锁
        camImageCopy_ = cam_image->image.clone();                                       //复制图像
      }
      {
        boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexActionStatus_); //加锁
        actionId_ = imageActionPtr->id;                                                //获取action id
      }
      {
        boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_); //加锁
        imageStatus_ = true;                                                        //图像已经接收到
      }
      frameWidth_ = cam_image->image.size().width;
      frameHeight_ = cam_image->image.size().height;
    }
    return;
  }
  //抢占回调函数
  void YoloObjectDetector::checkForObjectsActionPreemptCB()
  {
    ROS_DEBUG("[YoloObjectDetector] Preempt check for objects action.");
    checkForObjectsActionServer_->setPreempted(); //抢占回调函数
  }

  bool YoloObjectDetector::isCheckingForObjects() const //检测图像是否接收到
  {
    return (ros::ok() && checkForObjectsActionServer_->isActive() && !checkForObjectsActionServer_->isPreemptRequested()); //检测图像是否接收到
  }
  bool YoloObjectDetector::publishDetectionImage(const cv::Mat &detectionImage) //发布检测到的图像
  {
    if (detectionImagePublisher_.getNumSubscribers() < 1) //如果没有订阅者
      return false;
    cv_bridge::CvImage cvImage;
    cvImage.header.stamp = ros::Time::now();
    cvImage.header.frame_id = "detection_image";             //设置图像的frame_id
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;   //设置图像编码格式
    cvImage.image = detectionImage;                          //设置图像数据
    detectionImagePublisher_.publish(*cvImage.toImageMsg()); //发布图像
    ROS_DEBUG("Detection image has been published.");
    return true;
  }

  // double YoloObjectDetector::getWallTime()
  // {
  //   struct timeval time;
  //   if (gettimeofday(&time, NULL)) {
  //     return 0;
  //   }
  //   return (double) time.tv_sec + (double) time.tv_usec * .000001;
  // }

  int YoloObjectDetector::sizeNetwork(network *net) //获取网络的大小
  {
    int i;
    int count = 0;
    for (i = 0; i < net->n; ++i) //获取网络中的层数
    {
      layer l = net->layers[i];                                      //获取层
      if (l.type == YOLO || l.type == REGION || l.type == DETECTION) //如果是YOLO，REGION，DETECTION类型
      {
        count += l.outputs; //获取输出层的个数
      }
    }
    return count;
  }

  void YoloObjectDetector::rememberNetwork(network *net) //存储网络
  {
    int i;
    int count = 0;
    for (i = 0; i < net->n; ++i)
    {
      layer l = net->layers[i];
      if (l.type == YOLO || l.type == REGION || l.type == DETECTION) //如果是YOLO，REGION，DETECTION类型
      {
        memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs); //存储网络
        count += l.outputs;
      }
    }
  }

  detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes) //获取平均预测值
  {
    int i, j;
    int count = 0;
    fill_cpu(demoTotal_, 0, avg_, 1); //填充平均值
    for (j = 0; j < demoFrame_; ++j)
    {
      axpy_cpu(demoTotal_, 1. / demoFrame_, predictions_[j], 1, avg_, 1); //计算平均值
    }
    for (i = 0; i < net->n; ++i)
    {
      layer l = net->layers[i];
      if (l.type == YOLO || l.type == REGION || l.type == DETECTION) //如果是YOLO，REGION，DETECTION类型
      {
        memcpy(l.output, avg_ + count, sizeof(float) * l.outputs); //复制平均值
        count += l.outputs;
      }
    }
    // detection* dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
    detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes, 1); //获取网络的检测结果
    return dets;
  }

  void *YoloObjectDetector::detectInThread() //线程函数
  {
    running_ = 1;   //设置线程运行状态
    float nms = .4; //设置NMS的阈值

    layer l = net_->layers[net_->n - 1];               //获取最后一层
    float *X = buffLetter_[(buffIndex_ + 2) % 3].data; //  获取缓存的数据
    float *prediction = network_predict(*net_, X);     //预测网络

    rememberNetwork(net_);                //存储网络
    detection *dets = 0;                  //检测结果
    int nboxes = 0;                       //检测结果的个数
    dets = avgPredictions(net_, &nboxes); //获取平均预测值

    if (nms > 0)                                //如果NMS的阈值大于0
      do_nms_obj(dets, nboxes, l.classes, nms); //进行NMS操作

    if (enableConsoleOutput_) //如果启用控制台输出
    {
      printf("\033[2J");
      printf("\033[1;1H");
      printf("\nFPS:%.1f\n", fps_);
      printf("Objects:\n\n");
    }
    image display = buff_[(buffIndex_ + 2) % 3]; //获取缓存的图像
    // draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_, 1);
    draw_detections_v3(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_, 1); //绘制检测结果

    // extract the bounding boxes and send them to ROS
    int i, j;
    int count = 0;
    for (i = 0; i < nboxes; ++i)
    {
      float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.; //获取检测结果的xmin
      float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.; //获取检测结果的xmax
      float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.; //获取检测结果的ymin
      float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.; //获取检测结果的ymax

      if (xmin < 0)
        xmin = 0;
      if (ymin < 0)
        ymin = 0;
      if (xmax > 1)
        xmax = 1;
      if (ymax > 1)
        ymax = 1;

      // iterate through possible boxes and collect the bounding boxes
      for (j = 0; j < demoClasses_; ++j) //遍历所有类别
      {
        if (dets[i].prob[j]) //如果检测结果的概率大于0
        {
          float x_center = (xmin + xmax) / 2;     //获取检测结果的中心x坐标
          float y_center = (ymin + ymax) / 2;     //获取检测结果的中心y坐标
          float BoundingBox_width = xmax - xmin;  //获取检测结果的宽度
          float BoundingBox_height = ymax - ymin; //获取检测结果的高度

          // define bounding box
          // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
          if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) // BoundingBox的大小
          {
            roiBoxes_[count].x = x_center;
            roiBoxes_[count].y = y_center;
            roiBoxes_[count].w = BoundingBox_width;
            roiBoxes_[count].h = BoundingBox_height;
            roiBoxes_[count].Class = j;              //类别
            roiBoxes_[count].prob = dets[i].prob[j]; //概率
            count++;
          }
        }
      }
    }

    // create array to store found bounding boxes
    // if no object detected, make sure that ROS knows that num = 0
    if (count == 0)
    {
      roiBoxes_[0].num = 0;
    }
    else
    {
      roiBoxes_[0].num = count;
    }

    free_detections(dets, nboxes);
    demoIndex_ = (demoIndex_ + 1) % demoFrame_;
    running_ = 0;
    return 0;
  }

  void *YoloObjectDetector::fetchInThread() //线程函数
  {
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_); //获取读锁
      IplImageWithHeader_ imageAndHeader = getIplImageWithHeader();      //获取图像和头信息
      IplImage *ROS_img = imageAndHeader.image;                          //获取图像
      ipl_into_image(ROS_img, buff_[buffIndex_]);                        //将图像转换为网络的输入图像
      headerBuff_[buffIndex_] = imageAndHeader.header;                   //获取头信息
      buffId_[buffIndex_] = actionId_;                                   //获取动作id
    }
    rgbgr_image(buff_[buffIndex_]);                                                     //转换图像的颜色空间
    letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]); //将图像转换为网络的输入图像
    return 0;
  }

  void *YoloObjectDetector::displayInThread(void *ptr) //显示结果
  {
    show_image_cv(buff_[(buffIndex_ + 1) % 3], "YOLO V4"); //显示图像
    int c = cv::waitKey(waitKeyDelay_);                    //等待按键
    if (c != -1)
      c = c % 256;
    if (c == 27)
    {
      demoDone_ = 1;
      return 0;
    }
    else if (c == 82)
    {
      demoThresh_ += .02;
    }
    else if (c == 84)
    {
      demoThresh_ -= .02;
      if (demoThresh_ <= .02)
        demoThresh_ = .02;
    }
    else if (c == 83)
    {
      demoHier_ += .02;
    }
    else if (c == 81)
    {
      demoHier_ -= .02;
      if (demoHier_ <= .0)
        demoHier_ = .0;
    }
    return 0;
  }

  void *YoloObjectDetector::displayLoop(void *ptr) //结果显示循环
  {
    while (1)
    {
      displayInThread(0);
    }
  }

  void *YoloObjectDetector::detectLoop(void *ptr) //检测循环
  {
    while (1)
    {
      detectInThread();
    }
  }

  void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh, char **names, int classes, int delay,
                                        char *prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen) //设置YOLO V4网络
  {
    demoPrefix_ = prefix;                                 //获取前缀
    demoDelay_ = delay;                                   //获取延迟
    demoFrame_ = avg_frames;                              //获取平均帧数
    image **alphabet = load_alphabet_with_file(datafile); //加载字母表
    demoNames_ = names;                                   //获取名字
    demoAlphabet_ = alphabet;                             //获取字母表
    demoClasses_ = classes;                               //获取类别数量
    demoThresh_ = thresh;                                 //获取阈值
    demoHier_ = hier;                                     //获取层数
    fullScreen_ = fullscreen;                             //获取是否全屏
    printf("YOLO V4\n");
    net_ = load_network(cfgfile, weightfile, 0); //加载网络
    set_batch_network(net_, 1);                  //设置网络的批处理数量
  }

  void YoloObjectDetector::yolo() // yolo网络模型
  {
    const auto wait_duration = std::chrono::milliseconds(2000); //设置等待时间
    while (!getImageStatus())                                   //如果没有图像
    {
      printf("Waiting for image.\n");
      if (!isNodeRunning()) //如果节点停止
      {
        return;
      }
      std::this_thread::sleep_for(wait_duration); //等待
    }

    std::thread detect_thread; //创建检测线程
    std::thread fetch_thread;  //创建获取图像线程

    srand(2222222); //设置随机数种子

    int i;
    demoTotal_ = sizeNetwork(net_);                               //获取网络的总层数
    predictions_ = (float **)calloc(demoFrame_, sizeof(float *)); //分配内存
    for (i = 0; i < demoFrame_; ++i)                              //分配内存
    {
      predictions_[i] = (float *)calloc(demoTotal_, sizeof(float)); //分配内存,每个层的预测结果
    }
    avg_ = (float *)calloc(demoTotal_, sizeof(float)); //分配内存,平均值

    layer l = net_->layers[net_->n - 1];                                                       //获取最后一层
    roiBoxes_ = (darknet_ros::RosBox_ *)calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_)); //分配内存,每个目标的信息
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_); //获取互斥锁
      IplImageWithHeader_ imageAndHeader = getIplImageWithHeader();      //获取图像和头信息
      IplImage *ROS_img = imageAndHeader.image;                          //获取图像
      buff_[0] = ipl_to_image(ROS_img);                                  //转换图像
      headerBuff_[0] = imageAndHeader.header;                            //获取头信息
    }
    buff_[1] = copy_image(buff_[0]); //复制图像
    buff_[2] = copy_image(buff_[0]);
    headerBuff_[1] = headerBuff_[0]; //复制头信息
    headerBuff_[2] = headerBuff_[0];
    buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h); //缩放图像
    buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
    buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
    ipl_ = cvCreateImage(cvSize(buff_[0].w, buff_[0].h), IPL_DEPTH_8U, buff_[0].c); //创建图像

    int count = 0;

    if (!demoPrefix_ && viewImage_)
    {
      cv::namedWindow("YOLO V4", cv::WINDOW_NORMAL); //创建窗口
      if (fullScreen_)
      {
        cv::setWindowProperty("YOLO V4", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN); //设置窗口全屏
      }
      else
      {
        cv::moveWindow("YOLO V4", 0, 0);
        cv::resizeWindow("YOLO V4", 640, 480);
      }
    }

    demoTime_ = what_time_is_it_now(); //获取当前时间

    while (!demoDone_)
    {
      buffIndex_ = (buffIndex_ + 1) % 3;
      fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);   //创建获取图像线程
      detect_thread = std::thread(&YoloObjectDetector::detectInThread, this); //创建检测线程
      if (!demoPrefix_)
      {
        fps_ = 1. / (what_time_is_it_now() - demoTime_);
        demoTime_ = what_time_is_it_now();
        if (viewImage_)
        {
          displayInThread(0);
        }
        else
        {
          generate_image(buff_[(buffIndex_ + 1) % 3], ipl_);
        }
        publishInThread();
      }
      else
      {
        char name[256];
        sprintf(name, "%s_%08d", demoPrefix_, count);
        save_image(buff_[(buffIndex_ + 1) % 3], name);
      }
      fetch_thread.join();
      detect_thread.join();
      ++count;
      if (!isNodeRunning())
      {
        demoDone_ = true;
      }
    }
  }

  IplImageWithHeader_ YoloObjectDetector::getIplImageWithHeader() //获取图像和头信息
  {
    IplImage *ROS_img = new IplImage(camImageCopy_);                         //获取图像
    IplImageWithHeader_ header = {.image = ROS_img, .header = imageHeader_}; //获取头信息
    return header;
  }

  bool YoloObjectDetector::getImageStatus(void) //获取图像状态
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_); //获取互斥锁
    return imageStatus_;
  }

  bool YoloObjectDetector::isNodeRunning(void) //获取节点状态
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_); //获取互斥锁
    return nodeRunning_;
  }

  void *YoloObjectDetector::publishInThread() //发布图像线程
  {
    // Publish image.
    cv::Mat cvImage = cv::cvarrToMat(ipl_);       // 转换图像
    if (!publishDetectionImage(cv::Mat(cvImage))) //发布图像
    {
      ROS_DEBUG("Detection image has not been broadcasted.");
    }

    // Publish bounding boxes and detection result.
    int num = roiBoxes_[0].num; //获取目标数量
    if (num > 0 && num <= 100)  //如果目标数量大于0小于等于100
    {
      for (int i = 0; i < num; i++) //遍历目标
      {
        for (int j = 0; j < numClasses_; j++) //遍历类别
        {
          if (roiBoxes_[i].Class == j) //如果类别相等
          {
            rosBoxes_[j].push_back(roiBoxes_[i]); //添加目标
            rosBoxCounter_[j]++;
          }
        }
      }

      darknet_ros_msgs::ObjectCount msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "detection";
      msg.count = num;
      objectPublisher_.publish(msg); //发布目标数量

      for (int i = 0; i < numClasses_; i++)
      {
        if (rosBoxCounter_[i] > 0) //如果目标数量大于0
        {
          darknet_ros_msgs::BoundingBox boundingBox; //创建目标框

          for (int j = 0; j < rosBoxCounter_[i]; j++) //遍历目标
          {
            int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;  //获取目标框左上角x坐标
            int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_; //获取目标框左上角y坐标
            int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;  //获取目标框右下角x坐标
            int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_; //获取目标框右下角y坐标

            boundingBox.Class = classLabels_[i];                         //获取类别
            boundingBox.id = i;                                          //获取类别编号
            boundingBox.probability = rosBoxes_[i][j].prob;              //获取目标概率
            boundingBox.xmin = xmin;                                     //获取目标框左上角x坐标
            boundingBox.ymin = ymin;                                     //获取目标框左上角y坐标
            boundingBox.xmax = xmax;                                     //获取目标框右下角x坐标
            boundingBox.ymax = ymax;                                     //获取目标框右下角y坐标
            boundingBoxesResults_.bounding_boxes.push_back(boundingBox); //添加目标框
          }
        }
      }
      boundingBoxesResults_.header.stamp = ros::Time::now();                  //获取时间戳
      boundingBoxesResults_.header.frame_id = "detection";                    //获取图像帧id
      boundingBoxesResults_.image_header = headerBuff_[(buffIndex_ + 1) % 3]; //获取图像头信息
      boundingBoxesPublisher_.publish(boundingBoxesResults_);                 //发布目标框
    }
    else
    {
      darknet_ros_msgs::ObjectCount msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "detection";
      msg.count = 0;
      objectPublisher_.publish(msg);
    }
    if (isCheckingForObjects()) //如果检测目标
    {
      ROS_DEBUG("[YoloObjectDetector] check for objects in image.");
      darknet_ros_msgs::CheckForObjectsResult objectsActionResult;                             //创建检测目标结果
      objectsActionResult.id = buffId_[0];                                                     //获取图像id
      objectsActionResult.bounding_boxes = boundingBoxesResults_;                              //获取目标框
      checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes."); //发布检测目标结果
    }
    boundingBoxesResults_.bounding_boxes.clear(); //清空目标框
    for (int i = 0; i < numClasses_; i++)
    {
      rosBoxes_[i].clear();
      rosBoxCounter_[i] = 0;
    }

    return 0;
  }

} /* namespace darknet_ros*/
