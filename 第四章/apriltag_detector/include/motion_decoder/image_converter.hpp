#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
      : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
                               &ImageConverter::imageCb, this);     //订阅图像信息
    image_pub_ = it_.advertise("/image_converter/output_video", 1); //发布图像信息
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void setTagLocations(float x_det, float y_det, float z_det)
  {
    // TODO: Update tag locations
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr; // cv_bridge提供了一个CvImagePtr类，用于将ROS图像转换为OpenCV图像。
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //将图像信息转换为OpenCV格式
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // TODO: Draw circles at tag locations on image.

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image); //显示图像
    cv::waitKey(3);

    // TODO:Output modified video stream
    // Convert the modified frames into sensor_msgs::Image message and publish it using image_pub
  }

private:
  float x_loc, y_loc;
  std::vector<float> x_arr;
  std::vector<float> y_arr;
};
