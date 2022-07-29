#include <ros/ros.h>

// opencv image processing libraries
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"

// to facilitate the processing of the images in ROS
#include <image_transport/image_transport.h> // for publishing and subscribing to images in ROS
#include <cv_bridge/cv_bridge.h>             // to convert between ROS and OpenCV Image formats
#include <sensor_msgs/image_encodings.h>

#include <hog_haar_person_detection/Faces.h>       //用于发布人脸检测结果
#include <hog_haar_person_detection/Pedestrians.h> //用于发布行人检测结果
#include <hog_haar_person_detection/BoundingBox.h> //用于发布bounding box结果

static const std::string OPENCV_WINDOW = "Image window";

class HogHaarPersonDetection
{
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber im_sub_;
  image_transport::Publisher im_pub_;

  ros::Publisher faces_pub_;
  ros::Publisher pedestrians_pub_;

  cv::CascadeClassifier face_cascade_; //人脸检测级联分类器

  cv::HOGDescriptor hog_; // HOG检测器

  HogHaarPersonDetection()
      : it_(nh_)
  {
    // Get ROS params from launch file
    std::string image_topic; //图像输入的topic
    if (!nh_.getParam("image_topic", image_topic))
      ROS_ERROR("Could not get image_topic");

    std::string face_cascade_name_std; //人脸检测级联分类器的名称
    if (!nh_.getParam("face_cascade_name", face_cascade_name_std))
      ROS_ERROR("Could not get face_cascade_name");
    cv::String face_cascade_name = face_cascade_name_std;

    // Load the hog descriptor
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector()); //设置HOG检测器的检测器

    // Load face detector HAAR cascades
    if (!face_cascade_.load(face_cascade_name)) //加载人脸检测级联分类器
      ROS_ERROR("--(!)Error loading face detector cascade \n");

    // Subscrive to input video feed and publish output video feed
    im_sub_ = it_.subscribe(image_topic, 1, &HogHaarPersonDetection::imageCallback, this); //订阅图像输入的topic
    im_pub_ = it_.advertise("/camera_person_tracker/output_video", 1);                     //发布图像输出的topic

    // Publish detected faces and pedestrians.
    faces_pub_ = nh_.advertise<hog_haar_person_detection::Faces>("/person_detection/faces", 1000);
    pedestrians_pub_ = nh_.advertise<hog_haar_person_detection::Pedestrians>("/person_detection/pedestrians", 1000);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~HogHaarPersonDetection()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    // Make image processable in ROS
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //将ROS图像转换为OpenCV图像
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat im_bgr = cv_ptr->image; //获取OpenCV图像

    // HAAR Face detector
    std::vector<cv::Rect> detected_faces;
    cv::Mat im_gray;
    cv::cvtColor(im_bgr, im_gray, CV_BGR2GRAY);                                                                     //将BGR图像转换为灰度图像
    cv::equalizeHist(im_gray, im_gray);                                                                             //直方图均衡化
    face_cascade_.detectMultiScale(im_gray, detected_faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30)); //人脸检测

    // Publish message of location and confidence of detected faces.
    // Also, draw detected faces to screen as circles.
    hog_haar_person_detection::Faces faces_msg;
    for (unsigned i = 0; i < detected_faces.size(); i++)
    {
      // Draw on screen.
      cv::Point center(detected_faces[i].x + detected_faces[i].width * 0.5, detected_faces[i].y + detected_faces[i].height * 0.5);                       //计算人脸中心点
      cv::ellipse(im_bgr, center, cv::Size(detected_faces[i].width * 0.5, detected_faces[i].height * 0.5), 0, 0, 360, cv::Scalar(255, 0, 255), 4, 8, 0); //画出人脸框

      // Add to published message.
      hog_haar_person_detection::BoundingBox face;                        //用于发布bounding box结果
      face.center.x = detected_faces[i].x - detected_faces[i].width / 2;  //计算bounding box的中心点
      face.center.y = detected_faces[i].y - detected_faces[i].height / 2; //计算bounding box的中心点
      face.width = detected_faces[i].width;                               //计算bounding box的宽度
      face.height = detected_faces[i].height;                             //计算bounding box的高度
      faces_msg.faces.push_back(face);                                    //将bounding box加入faces_msg中
    }
    faces_pub_.publish(faces_msg); //发布faces_msg

    // HOG pedestrian detector
    std::vector<cv::Rect> detected_pedestrian;
    hog_.detectMultiScale(im_gray, detected_pedestrian, 0.0, cv::Size(8, 8), cv::Size(0, 0), 1.05, 4); // HOG检测器检测人脸

    // Publish message of location and confident of detected pedestrians.
    // Draw detections from HOG to the screen.
    hog_haar_person_detection::Pedestrians pedestrians_msg; //用于发布pedestrians结果
    for (unsigned i = 0; i < detected_pedestrian.size(); i++)
    {
      // Draw on screen.
      cv::rectangle(im_bgr, detected_pedestrian[i], cv::Scalar(255)); //画出人脸框

      // Add to published message.
      hog_haar_person_detection::BoundingBox pedestrian; //用于发布bounding box结果
      pedestrian.center.x = detected_pedestrian[i].x - detected_pedestrian[i].width / 2;
      pedestrian.center.y = detected_pedestrian[i].y - detected_pedestrian[i].height / 2;
      pedestrian.width = detected_pedestrian[i].width;
      pedestrian.height = detected_pedestrian[i].height;
      pedestrians_msg.pedestrians.push_back(pedestrian); //将bounding box加入pedestrians_msg中
    }
    pedestrians_pub_.publish(pedestrians_msg);

    // Publish image to screen.

    sensor_msgs::ImagePtr projected_img =
        cv_bridge::CvImage(msg->header, "bgr8", im_bgr).toImageMsg(); //将OpenCV图像转换为ROS图像
    im_pub_.publish(projected_img);                                   //发布图像输出的topic

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, im_bgr);
    // cv::waitKey(3);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hog_haar_person_detection");
  HogHaarPersonDetection hhpd;
  ros::spin();
  return 0;
}
