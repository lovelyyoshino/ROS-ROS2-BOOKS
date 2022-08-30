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

  // yolo 识别的构造函数
  YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
      : nodeHandle_(nh), imageTransport_(nodeHandle_), numClasses_(0), classLabels_(0), rosBoxes_(0), rosBoxCounter_(0)
  {
    ROS_INFO("[YoloObjectDetector] Node started.");

    // Read parameters from config file.
    if (!readParameters())
    {
      ros::requestShutdown(); //读取参数失败，关闭节点
    }

    init();
  }

  YoloObjectDetector::~YoloObjectDetector()
  {
    {
      boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_); //加锁
      isNodeRunning_ = false;
    }
    yoloThread_.join(); //等待线程结束
  }

  bool YoloObjectDetector::readParameters()
  {
    // Load common parameters.
    nodeHandle_.param("image_view/enable_opencv", viewImage_, true);                    //是否显示图像
    nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);                   //等待时间
    nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false); //是否显示控制台输出

    // Check if Xserver is running on Linux.
    if (XOpenDisplay(NULL)) //如果返回非空，则表示Xserver已经启动
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
    nodeHandle_.param("yolo_model/detection_classes/names", classLabels_, std::vector<std::string>(0)); //读取类别名称
    numClasses_ = classLabels_.size();                                                                  //类别数量
    rosBoxes_ = std::vector<std::vector<RosBox_>>(numClasses_);                                         //每个类别的检测框
    rosBoxCounter_ = std::vector<int>(numClasses_);                                                     //计数器

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
    nodeHandle_.param("yolo_model/weight_file/name", weightsModel, std::string("yolov2-tiny.weights")); //权重文件名称
    nodeHandle_.param("weights_path", weightsPath, std::string("/default"));                            //权重文件路径
    weightsPath += "/" + weightsModel;
    weights = new char[weightsPath.length() + 1];
    strcpy(weights, weightsPath.c_str()); //将字符串复制到char*

    // Path to config file.
    nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("yolov2-tiny.cfg")); //配置文件名称
    nodeHandle_.param("config_path", configPath, std::string("/default"));                         //配置文件路径
    configPath += "/" + configModel;
    cfg = new char[configPath.length() + 1];
    strcpy(cfg, configPath.c_str());

    // Path to data folder.
    dataPath = darknetFilePath_;
    dataPath += "/data";
    data = new char[dataPath.length() + 1];
    strcpy(data, dataPath.c_str()); //将字符串复制到char*

    // Get classes.
    detectionNames = (char **)realloc((void *)detectionNames, (numClasses_ + 1) * sizeof(char *)); //重新分配内存
    for (int i = 0; i < numClasses_; i++)
    {
      detectionNames[i] = new char[classLabels_[i].length() + 1]; //分配内存
      strcpy(detectionNames[i], classLabels_[i].c_str());
    }

    // Load network.
    setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_, 0, 0, 1, 0.5, 0, 0, 0, 0); //加载网络
    yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);                                      //创建线程

    // Initialize publisher and subscriber.
    std::string cameraTopicName;
    int cameraQueueSize;
    std::string objectDetectorTopicName;
    int objectDetectorQueueSize;
    bool objectDetectorLatch;
    std::string boundingBoxesTopicName;
    int boundingBoxesQueueSize;
    bool boundingBoxesLatch;
    std::string detectionImageTopicName;
    int detectionImageQueueSize;
    bool detectionImageLatch;

    nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/camera/image_raw"));
    nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
    nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName, std::string("found_object"));
    nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
    nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);
    nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName, std::string("bounding_boxes"));
    nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
    nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);
    nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName, std::string("detection_image"));
    nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
    nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);

    imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize, &YoloObjectDetector::cameraCallback, this);
    objectPublisher_ =
        nodeHandle_.advertise<darknet_ros_msgs::ObjectCount>(objectDetectorTopicName, objectDetectorQueueSize, objectDetectorLatch);
    boundingBoxesPublisher_ =
        nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
    detectionImagePublisher_ =
        nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName, detectionImageQueueSize, detectionImageLatch);

    // Action servers.
    std::string checkForObjectsActionName;
    nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName, std::string("check_for_objects"));                //检测目标的action名称
    checkForObjectsActionServer_.reset(new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));            //创建action服务器
    checkForObjectsActionServer_->registerGoalCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));       //注册目标回调函数
    checkForObjectsActionServer_->registerPreemptCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this)); //注册取消目标回调函数
    checkForObjectsActionServer_->start();
  }

  void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    ROS_DEBUG("[YoloObjectDetector] USB image received.");

    cv_bridge::CvImagePtr cam_image;

    try
    {
      if (msg->encoding == "mono8" || msg->encoding == "bgr8" || msg->encoding == "rgb8") //如果是灰度图像
      {
        cam_image = cv_bridge::toCvCopy(msg, msg->encoding); //将图像转换成cv_bridge格式
      }
      else if (msg->encoding == "bgra8")
      {
        cam_image = cv_bridge::toCvCopy(msg, "bgr8"); //将图像转换成cv_bridge格式
      }
      else if (msg->encoding == "rgba8")
      {
        cam_image = cv_bridge::toCvCopy(msg, "rgb8"); //将图像转换成cv_bridge格式
      }
      else if (msg->encoding == "mono16")
      {
        ROS_WARN_ONCE("Converting mono16 images to mono8");
        cam_image = cv_bridge::toCvCopy(msg, "mono8"); //将图像转换成cv_bridge格式
      }
      else
      {
        ROS_ERROR("Image message encoding provided is not mono8, mono16, bgr8, bgra8, rgb8 or rgba8.");
      }
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
        imageHeader_ = msg->header;
        camImageCopy_ = cam_image->image.clone();
      }
      {
        boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_); //加锁
        imageStatus_ = true;
      }
      frameWidth_ = cam_image->image.size().width;
      frameHeight_ = cam_image->image.size().height;
    }
    return;
  }

  void YoloObjectDetector::checkForObjectsActionGoalCB()
  {
    ROS_DEBUG("[YoloObjectDetector] Start check for objects action.");

    boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> imageActionPtr = checkForObjectsActionServer_->acceptNewGoal(); //接受新的目标
    sensor_msgs::Image imageAction = imageActionPtr->image;                                                                        //获取图像

    cv_bridge::CvImagePtr cam_image;

    try
    {
      cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8); //将图像转换成cv_bridge格式
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
        boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexActionStatus_);
        actionId_ = imageActionPtr->id;
      }
      {
        boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
        imageStatus_ = true;
      }
      frameWidth_ = cam_image->image.size().width;   //获取图像的宽度
      frameHeight_ = cam_image->image.size().height; //获取图像的高度
    }
    return;
  }

  void YoloObjectDetector::checkForObjectsActionPreemptCB()
  {
    ROS_DEBUG("[YoloObjectDetector] Preempt check for objects action.");
    checkForObjectsActionServer_->setPreempted(); //取消目标
  }

  bool YoloObjectDetector::isCheckingForObjects() const
  {
    return (ros::ok() && checkForObjectsActionServer_->isActive() && !checkForObjectsActionServer_->isPreemptRequested()); //判断是否在检测目标
  }

  bool YoloObjectDetector::publishDetectionImage(const cv::Mat &detectionImage)
  {
    if (detectionImagePublisher_.getNumSubscribers() < 1) //如果没有订阅者
      return false;
    cv_bridge::CvImage cvImage;
    cvImage.header.stamp = ros::Time::now();
    cvImage.header.frame_id = "detection_image";
    cvImage.encoding = sensor_msgs::image_encodings::BGR8; //设置图像编码格式
    cvImage.image = detectionImage;                        //设置图像
    detectionImagePublisher_.publish(*cvImage.toImageMsg());
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
    for (i = 0; i < net->n; ++i)
    {
      layer l = net->layers[i];
      if (l.type == YOLO || l.type == REGION || l.type == DETECTION) //如果是YOLO或者REGION或者DETECTION
      {
        count += l.outputs; //获取输出层的大小
      }
    }
    return count;
  }

  void YoloObjectDetector::rememberNetwork(network *net) //记忆网络
  {
    int i;
    int count = 0;
    for (i = 0; i < net->n; ++i)
    {
      layer l = net->layers[i];                                      //获取层
      if (l.type == YOLO || l.type == REGION || l.type == DETECTION) //如果是YOLO或者REGION或者DETECTION
      {
        memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs); //复制输出层的数据
        count += l.outputs;                                                                         //计数器加1
      }
    }
  }

  detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes) //获取平均预测
  {
    int i, j;
    int count = 0;
    fill_cpu(demoTotal_, 0, avg_, 1); //填充avg_数组
    for (j = 0; j < demoFrame_; ++j)
    {
      axpy_cpu(demoTotal_, 1. / demoFrame_, predictions_[j], 1, avg_, 1); //计算平均值
    }
    for (i = 0; i < net->n; ++i)
    {
      layer l = net->layers[i];
      if (l.type == YOLO || l.type == REGION || l.type == DETECTION) //如果是YOLO或者REGION或者DETECTION
      {
        memcpy(l.output, avg_ + count, sizeof(float) * l.outputs); //复制平均值到输出层
        count += l.outputs;                                        //计数器加1
      }
    }
    detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes); //获取网络的boxes
    return dets;
  }

  void *YoloObjectDetector::detectInThread() //线程函数
  {
    running_ = 1;
    float nms = .4;

    layer l = net_->layers[net_->n - 1];               //获取最后一层
    float *X = buffLetter_[(buffIndex_ + 2) % 3].data; //获取缓存的数据
    float *prediction = network_predict(net_, X);      //预测网络

    rememberNetwork(net_); //记忆网络
    detection *dets = 0;
    int nboxes = 0;
    dets = avgPredictions(net_, &nboxes); //获取平均预测

    if (nms > 0)
      do_nms_obj(dets, nboxes, l.classes, nms); //进行NMS

    if (enableConsoleOutput_)
    {
      printf("\033[2J");
      printf("\033[1;1H");
      printf("\nFPS:%.1f\n", fps_);
      printf("Objects:\n\n");
    }
    image display = buff_[(buffIndex_ + 2) % 3];                                                  //获取缓存的图像
    draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_); //绘制结果

    // extract the bounding boxes and send them to ROS
    int i, j;
    int count = 0;
    for (i = 0; i < nboxes; ++i)
    {
      float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.; //获取xmin
      float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.; //获取xmax
      float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.; //获取ymin
      float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.; //获取ymax

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
        if (dets[i].prob[j]) //如果概率大于阈值
        {
          float x_center = (xmin + xmax) / 2;
          float y_center = (ymin + ymax) / 2;
          float BoundingBox_width = xmax - xmin;  //获取宽度
          float BoundingBox_height = ymax - ymin; //获取高度

          // define bounding box
          // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
          if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) //如果宽度和高度大于0.01
          {
            roiBoxes_[count].x = x_center;           //记录x坐标
            roiBoxes_[count].y = y_center;           //记录y坐标
            roiBoxes_[count].w = BoundingBox_width;  //记录宽度
            roiBoxes_[count].h = BoundingBox_height; //记录高度
            roiBoxes_[count].Class = j;              //记录类别
            roiBoxes_[count].prob = dets[i].prob[j]; //记录概率
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

    free_detections(dets, nboxes);              //释放内存
    demoIndex_ = (demoIndex_ + 1) % demoFrame_; //计数器加1
    running_ = 0;
    return 0;
  }

  void *YoloObjectDetector::fetchInThread()
  {
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_); //获取读锁
      CvMatWithHeader_ imageAndHeader = getCvMatWithHeader();            //获取图像和头信息
      free_image(buff_[buffIndex_]);                                     //释放内存
      buff_[buffIndex_] = mat_to_image(imageAndHeader.image);            //转换成图像
      headerBuff_[buffIndex_] = imageAndHeader.header;                   //转换成头信息
      buffId_[buffIndex_] = actionId_;                                   //记录actionId
    }
    rgbgr_image(buff_[buffIndex_]);                                                     //转换成BGR格式
    letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]); //调整图像大小
    return 0;
  }

  void *YoloObjectDetector::displayInThread(void *ptr)
  {
    int c = show_image(buff_[(buffIndex_ + 1) % 3], "YOLO", 1); //显示图像
    if (c != -1)
      c = c % 256; //获取键盘输入
    if (c == 27)
    {
      demoDone_ = 1; //退出
      return 0;
    }
    else if (c == 82)
    {
      demoThresh_ += .02; //阈值加0.02
    }
    else if (c == 84)
    {
      demoThresh_ -= .02; //阈值减0.02
      if (demoThresh_ <= .02)
        demoThresh_ = .02;
    }
    else if (c == 83)
    {
      demoHier_ += .02; //层数加0.02
    }
    else if (c == 81)
    {
      demoHier_ -= .02; //层数减0.02
      if (demoHier_ <= .0)
        demoHier_ = .0;
    }
    return 0;
  }

  void *YoloObjectDetector::displayLoop(void *ptr)
  {
    while (1)
    {
      displayInThread(0); //显示图像
    }
  }

  void *YoloObjectDetector::detectLoop(void *ptr)
  {
    while (1)
    {
      detectInThread(); //检测图像
    }
  }
  //设置网络模型启动参数
  void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh, char **names, int classes, int delay,
                                        char *prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen)
  {
    demoPrefix_ = prefix;                                 //设置前缀
    demoDelay_ = delay;                                   //设置延迟
    demoFrame_ = avg_frames;                              //设置帧数
    image **alphabet = load_alphabet_with_file(datafile); //加载字符集
    demoNames_ = names;                                   //设置类别名称
    demoAlphabet_ = alphabet;                             //设置字符集
    demoClasses_ = classes;                               //设置类别数量
    demoThresh_ = thresh;                                 //设置阈值
    demoHier_ = hier;                                     //设置层数
    fullScreen_ = fullscreen;                             //设置是否全屏
    printf("YOLO\n");
    net_ = (cfgfile, weightfile, 0); //加载网络模型
    set_batch_network(net_, 1);      //设置网络模型的batch为1
  }

  void YoloObjectDetector::yolo()
  {
    const auto wait_duration = std::chrono::milliseconds(2000);
    while (!getImageStatus())
    {
      printf("Waiting for image.\n");
      if (!isNodeRunning())
      {
        return;
      }
      std::this_thread::sleep_for(wait_duration);
    }

    std::thread detect_thread; //检测线程
    std::thread fetch_thread;  //获取线程

    srand(2222222); //设置随机数种子

    int i;
    demoTotal_ = sizeNetwork(net_);                               //获取网络模型的层数
    predictions_ = (float **)calloc(demoFrame_, sizeof(float *)); //分配内存
    for (i = 0; i < demoFrame_; ++i)
    {
      predictions_[i] = (float *)calloc(demoTotal_, sizeof(float)); //分配内存
    }
    avg_ = (float *)calloc(demoTotal_, sizeof(float)); //分配内存

    layer l = net_->layers[net_->n - 1];
    roiBoxes_ = (darknet_ros::RosBox_ *)calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_)); //分配内存

    {
      boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_); //获取读锁
      CvMatWithHeader_ imageAndHeader = getCvMatWithHeader();            //获取图像和头信息
      buff_[0] = mat_to_image(imageAndHeader.image);                     //转换成图像格式
      headerBuff_[0] = imageAndHeader.header;                            //设置头信息
    }
    buff_[1] = copy_image(buff_[0]); //复制图像
    buff_[2] = copy_image(buff_[0]);
    headerBuff_[1] = headerBuff_[0]; //复制头信息
    headerBuff_[2] = headerBuff_[0];
    buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h); //设置图像大小
    buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
    buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
    disp_ = image_to_mat(buff_[0]); //转换成mat格式

    int count = 0;
    if (!demoPrefix_ && viewImage_) //如果没有前缀，并且显示图像
    {
      cv::namedWindow("YOLO", cv::WINDOW_NORMAL);
      if (fullScreen_)
      {
        cv::setWindowProperty("YOLO", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN); //设置全屏
      }
      else
      {
        cv::moveWindow("YOLO", 0, 0);
        cv::resizeWindow("YOLO", 640, 480);
      }
    }

    demoTime_ = what_time_is_it_now(); //获取当前时间

    while (!demoDone_) //如果没有处理结果
    {
      buffIndex_ = (buffIndex_ + 1) % 3;                                      //设置缓存索引
      fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);   //设置获取线程
      detect_thread = std::thread(&YoloObjectDetector::detectInThread, this); //设置检测线程
      if (!demoPrefix_)                                                       //如果没有前缀
      {
        fps_ = 1. / (what_time_is_it_now() - demoTime_); //计算帧率
        demoTime_ = what_time_is_it_now();               //设置当前时间
        if (viewImage_)
        {
          displayInThread(0);
        }
        else
        {
          generate_image(buff_[(buffIndex_ + 1) % 3], disp_); //生成图像
        }
        publishInThread();
      }
      else
      {
        char name[256];
        sprintf(name, "%s_%08d", demoPrefix_, count);
        save_image(buff_[(buffIndex_ + 1) % 3], name); //保存图像
      }
      fetch_thread.join();  //等待获取线程结束
      detect_thread.join(); //等待检测线程结束
      ++count;
      if (!isNodeRunning())
      {
        demoDone_ = true;
      }
    }
  }

  CvMatWithHeader_ YoloObjectDetector::getCvMatWithHeader() //获取图像和头信息
  {
    CvMatWithHeader_ header = {.image = camImageCopy_, .header = imageHeader_}; //设置头信息
    return header;
  }

  bool YoloObjectDetector::getImageStatus(void)
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_); //获取读锁
    return imageStatus_;
  }

  bool YoloObjectDetector::isNodeRunning(void)
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
    return isNodeRunning_;
  }

  void *YoloObjectDetector::publishInThread()
  {
    // Publish image.
    cv::Mat cvImage = disp_;
    if (!publishDetectionImage(cv::Mat(cvImage))) //发布检测图像
    {
      ROS_DEBUG("Detection image has not been broadcasted.");
    }

    // Publish bounding boxes and detection result.
    int num = roiBoxes_[0].num; //获取检测结果数量
    if (num > 0 && num <= 100)
    {
      for (int i = 0; i < num; i++)
      {
        for (int j = 0; j < numClasses_; j++)
        {
          if (roiBoxes_[i].Class == j)
          {
            rosBoxes_[j].push_back(roiBoxes_[i]); //设置检测结果
            rosBoxCounter_[j]++;
          }
        }
      }

      darknet_ros_msgs::ObjectCount msg;   //设置检测结果消息
      msg.header.stamp = ros::Time::now(); //设置时间戳
      msg.header.frame_id = "detection";   //设置帧id
      msg.count = num;
      objectPublisher_.publish(msg); //发布检测结果消息

      for (int i = 0; i < numClasses_; i++)
      {
        if (rosBoxCounter_[i] > 0)
        {
          darknet_ros_msgs::BoundingBox boundingBox; //设置检测结果消息

          for (int j = 0; j < rosBoxCounter_[i]; j++)
          {
            int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
            int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
            int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
            int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;

            boundingBox.Class = classLabels_[i];
            boundingBox.id = i;
            boundingBox.probability = rosBoxes_[i][j].prob;
            boundingBox.xmin = xmin;
            boundingBox.ymin = ymin;
            boundingBox.xmax = xmax;
            boundingBox.ymax = ymax;
            boundingBoxesResults_.bounding_boxes.push_back(boundingBox); //检测结果消息发布
          }
        }
      }
      boundingBoxesResults_.header.stamp = ros::Time::now();
      boundingBoxesResults_.header.frame_id = "detection";                    //设置帧id
      boundingBoxesResults_.image_header = headerBuff_[(buffIndex_ + 1) % 3]; //设置图像头信息
      boundingBoxesPublisher_.publish(boundingBoxesResults_);                 //发布检测结果消息
    }
    else
    {
      darknet_ros_msgs::ObjectCount msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "detection";
      msg.count = 0;
      objectPublisher_.publish(msg);
    }
    if (isCheckingForObjects())
    {
      ROS_DEBUG("[YoloObjectDetector] check for objects in image.");
      darknet_ros_msgs::CheckForObjectsResult objectsActionResult;                             //设置检测结果消息
      objectsActionResult.id = buffId_[0];                                                     //设置id
      objectsActionResult.bounding_boxes = boundingBoxesResults_;                              //设置检测结果消息
      checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes."); //发布检测结果消息
    }
    boundingBoxesResults_.bounding_boxes.clear(); //清空检测结果消息
    for (int i = 0; i < numClasses_; i++)
    {
      rosBoxes_[i].clear();
      rosBoxCounter_[i] = 0;
    }

    return 0;
  }

} /* namespace darknet_ros*/
