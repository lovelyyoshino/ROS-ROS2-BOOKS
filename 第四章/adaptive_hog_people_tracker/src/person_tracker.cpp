#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <vector>
#include <boost/thread.hpp>

#include "LinearSVM.hpp"
#include "FrameSize.hpp"
#include "HOGDetector.hpp"
#include "BoundingBox.h"

#define MIN_POSITIVES 5
#define MIN_NEGATIVES (2 * MIN_POSITIVES)
#define MAX_POSITIVES 20
#define MAX_NEGATIVES (2 * MAX_POSITIVES)
#define MIN_TRAIN (MIN_POSITIVES + MIN_NEGATIVES * 10)
#define MAX_TRAIN (MAX_POSITIVES + MAX_NEGATIVES * 10)

#define MIN_THRESHOLD 0.1
#define MAX_THRESHOLD 0.65

#define POSITIVE 1
#define NEGATIVE -1

#define SKIP_ADD_SAMPLES 1

const std::string windowTitle = "Person Tracker";

void imageCallback(const sensor_msgs::ImageConstPtr &); //图像回调函数

void hogTraining(); // HOG模型训练函数

// Computes the features of positive/negative image
void computeFeatures(cv::Mat, cv::Rect); //计算特征函数
void computeFeatureVector(cv::Mat, int); //计算特征向量函数

void enlargeSearchRoi();		 //放大搜索区域函数
void printFunction(std::string); //打印函数

void showHelpMessage(std::string); //显示帮助信息函数

HOGDetector detectionHog; // HOG检测器

HOGDetector trainingHog; // HOG训练器

int frameNumber = 0; //当前帧数

bool isTraining = false; //训练标志

cv::Rect searchRoi(0, 0, frameSize.width, frameSize.height); //搜索区域

boost::mutex hogMutex;		// HOG锁
boost::mutex featuresMutex; //特征锁

LinearSVM svm; //线性SVM模型

ros::Publisher p;

cv::Mat trainData, trainLabels;

bool detected = false; //检测标志
bool print = true;	   //打印标志

// Kalman filter
int stateSize = 6;
int measSize = 4;
int contrSize = 0;
cv::KalmanFilter kf(stateSize, measSize, contrSize); //卡尔曼滤波器

cv::Mat state(stateSize, 1, CV_32F);
cv::Mat meas(measSize, 1, CV_32F);

double ticks = 0;	   //时间计数
int notFoundCount = 0; //未找到计数

double searchRoiRatio;					   //搜索区域比例
double detectionThreshold = MIN_THRESHOLD; //检测阈值

int main(int argc, char **argv)
{
	ros::init(argc, argv, "person_tracker");

	ros::NodeHandle nh;

	std::string camera_t, bounding_box_t, detector_type;
	int verbosity_level;
	ros::param::param<std::string>("~/camera_topic", camera_t, "camera/image_raw");
	ros::param::param<std::string>("~/bounding_box_topic", bounding_box_t, "bounding_box");
	ros::param::param<std::string>("~/detector", detector_type, "full");
	ros::param::param<int>("~/verbosity_level", verbosity_level, 0);

	image_transport::ImageTransport it(nh); //初始化图像传输类

	image_transport::Subscriber sub = it.subscribe(camera_t, 1, imageCallback); //订阅图像信息

	p = nh.advertise<adaptive_hog_people_tracker::BoundingBox>(bounding_box_t, 1); //发布bounding_box信息

	if (verbosity_level == 0)
	{
		print = false;
	}
	else if (verbosity_level == 1)
	{
		print = true;
	}
	else
	{
		ROS_ERROR("Param verbosity_level must be 0 (no output) or 1 (full output)!");
		return 1;
	}

	if (detector_type == "full") //如果检测器类型为full
	{
		detectionHog = HOGDetector(HOGDetector::FULL); //初始化HOG检测器
		trainingHog = HOGDetector(HOGDetector::FULL);  //初始化HOG训练器
	}
	else if (detector_type == "torso")
	{
		detectionHog = HOGDetector(HOGDetector::TORSO); //初始化HOG检测器
		trainingHog = HOGDetector(HOGDetector::TORSO);	//初始化HOG训练器
	}

	searchRoiRatio = detectionHog.getSearchRoiRatio(); //获取搜索区域比例

	cv::startWindowThread(); //初始化窗口线程

	cv::namedWindow(windowTitle); //初始化窗口

	cv::setIdentity(kf.transitionMatrix); //初始化卡尔曼滤波器矩阵

	kf.errorCovPre.at<float>(0) = 1; //初始化卡尔曼滤波器误差矩阵
	kf.errorCovPre.at<float>(7) = 1;
	kf.errorCovPre.at<float>(14) = 1;
	kf.errorCovPre.at<float>(21) = 1;
	kf.errorCovPre.at<float>(28) = 1;
	kf.errorCovPre.at<float>(35) = 1;

	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
	kf.measurementMatrix.at<float>(0) = 1.0f; //初始化卡尔曼滤波器测量矩阵
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(16) = 1.0f;
	kf.measurementMatrix.at<float>(23) = 1.0f;

	kf.processNoiseCov.at<float>(0) = 1e-2; //初始化卡尔曼滤波器过程噪声矩阵
	kf.processNoiseCov.at<float>(7) = 1e-4;
	kf.processNoiseCov.at<float>(14) = 1e-3;
	kf.processNoiseCov.at<float>(21) = 1e-10;
	kf.processNoiseCov.at<float>(28) = 1e-4;
	kf.processNoiseCov.at<float>(35) = 1e-4;

	state.at<float>(0) = frameSize.width / 2; //初始化卡尔曼滤波器状态向量
	state.at<float>(1) = frameSize.height / 2;

	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-2)); //将矩阵中对角线上的元素设为1e-2

	while (ros::ok())
	{
		if (!cvGetWindowHandle(windowTitle.c_str()))
		{
			break;
		}

		ros::spinOnce();
	}

	cv::destroyAllWindows();
	ros::shutdown();
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	cv::Mat image;

	try
	{
		image = cv_bridge::toCvShare(msg, "bgr8")->image; //将图像转换为OpenCV格式
		if (image.empty())
		{
			return;
		}
	}
	catch (cv_bridge::Exception &e)
	{
		// Conversion failed
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str()); //转换失败
		return;
	}

	cv::resize(image, image, frameSize); //将图像缩放到指定大小

	double precTick = ticks; //上一时刻赋值

	ticks = (double)cv::getTickCount(); //获取当前时间

	double dT = (ticks - precTick) / cv::getTickFrequency(); //计算时间差

	cv::Mat temp(image.size(), CV_8UC3); //初始化临时图像
	temp.setTo(cv::Scalar::all(0));		 //将临时图像设置为黑色

	adaptive_hog_people_tracker::BoundingBox par; //初始化boundingBox结构体

	if (detected)
	{
		// >>>> Matrix A
		kf.transitionMatrix.at<float>(2) = dT;
		kf.transitionMatrix.at<float>(9) = dT;
		// <<<< Matrix A

		state = kf.predict(); //预测状态向量

		cv::Rect predRect;
		predRect.width = state.at<float>(4);
		predRect.height = state.at<float>(5);
		predRect.x = state.at<float>(0) - predRect.width / 2;
		predRect.y = state.at<float>(1) - predRect.height / 2;

		searchRoi.width = predRect.width * searchRoiRatio; //计算搜索区域宽度
		searchRoi.height = predRect.height + (searchRoi.width - predRect.width);
		searchRoi.x = predRect.x - ((searchRoi.width - predRect.width) / 2);
		searchRoi.y = predRect.y - ((searchRoi.width - predRect.width) / 2);

		if (searchRoi.x < 0)
		{
			searchRoi.x = 0;
		}
		if (searchRoi.y < 0)
		{
			searchRoi.y = 0;
		}
		if (searchRoi.br().x > frameSize.width)
		{
			searchRoi.width -= searchRoi.br().x - frameSize.width;
		}
		if (searchRoi.br().y > frameSize.height)
		{
			searchRoi.height -= searchRoi.br().y - frameSize.height;
		}

		cv::Point center;
		center.x = state.at<float>(0);
		center.y = state.at<float>(1);

		par.x = center.x; //设置boundingBox结构体的x坐标
		par.y = center.y;
		par.width = predRect.width; //设置boundingBox结构体的宽度
		par.height = predRect.height;

		cv::rectangle(temp, predRect, cv::Scalar(0, 0, 255), 2); //画出预测矩形
		cv::circle(temp, center, 1, cv::Scalar(0, 0, 255), 2);	 //画出预测中心点
	}
	else
	{
		par.x = -1;
		par.y = -1;
		par.width = -1;
		par.height = -1;
	}

	p.publish(par);

	bool foundAtLeastOne = false;

	std::vector<cv::Rect> found;
	std::vector<double> confidence;

	if (frameNumber > 10) //如果帧数大于10，则开始检测,防止开始的黑屏
	{
		hogMutex.lock();
		detectionHog.detectMultiScale(image(searchRoi), found, confidence, detectionThreshold, cv::Size(), cv::Size(), 1.05, 1, false); //检测行人
		hogMutex.unlock();
	}

	cv::Rect currArea;
	cv::Point currDetection;

	double distance = cv::norm(cv::Point(0, 0) - cv::Point(frameSize.width, frameSize.height)); //计算两点之间的距离

	for (int i = 0; i < found.size(); i++) //遍历所有检测到的人
	{
		cv::Rect r = found[i]; //获取检测到的人的矩形框

		int j;

		for (j = 0; j < found.size(); j++)
		{
			if (j != i && (r & found[j]) == r) //如果检测到的人的矩形框与其他人的矩形框有重叠，则跳出循环
			{
				break;
			}
		}

		if (j == found.size()) //如果检测到的人的矩形框与其他人的矩形框没有重叠，则认为是一个独立的人
		{
			//----da roi a immagine----
			r.x += searchRoi.x; //将矩形框的x坐标加上搜索区域的x坐标
			r.y += searchRoi.y;

			if (r.x < 0) //如果矩形框的x坐标小于0，则将其设置为0
			{
				r.x = 0;
			}
			if (r.y < 0) //如果矩形框的y坐标小于0，则将其设置为0
			{
				r.y = 0;
			}
			if (r.br().x > frameSize.width) //如果矩形框的右下角x坐标大于图像的宽度，则将其设置为图像的宽度
			{
				r.width -= r.br().x - frameSize.width;
			}
			if (r.br().y > frameSize.height) //如果矩形框的右下角y坐标大于图像的高度，则将其设置为图像的高度
			{
				r.height -= r.br().y - frameSize.height;
			}

			cv::Point r_center;

			r_center.x = r.x + r.width / 2;	 //计算矩形框的中心点x坐标
			r_center.y = r.y + r.height / 2; //计算矩形框的中心点y坐标

			cv::rectangle(temp, r, cv::Scalar(255, 0, 0), 2);

			if (cv::norm(r_center - cv::Point(state.at<float>(0), state.at<float>(1))) < distance) //如果检测到的矩形中心点与预测的中心点距离小于预测的宽度，则认为是同一个人
			{
				distance = cv::norm(r_center - cv::Point(state.at<float>(0), state.at<float>(1))); //更新距离
				currDetection = cv::Point(r_center);											   //更新检测到的中心点
				currArea = cv::Rect(r);															   //更新检测到的矩形

				foundAtLeastOne = true; //设置为true，表示检测到了人

				detectionThreshold = confidence[i] * 0.6; //更新检测阈值
			}
		}
	}

	if (foundAtLeastOne) //如果检测到了人
	{
		//-------Collecting Samples--------
		if (frameNumber % SKIP_ADD_SAMPLES == 0)
		{
			if (currArea.width > detectionHog.winSize.width / 2 && currArea.height > detectionHog.winSize.height / 2)
			{
				boost::thread t(computeFeatures, image, currArea);
				t.detach();

				//----Automatic Start Training
				if (trainData.rows >= MIN_TRAIN && !isTraining)
				{
					boost::thread t(hogTraining); //开启线程
					t.detach();
				}
			}
		}

		notFoundCount = 0; //如果检测到了人，则将没有检测到人的计数器置0

		meas.at<float>(0) = currDetection.x; //更新状态量
		meas.at<float>(1) = currDetection.y;
		meas.at<float>(2) = (float)currArea.width;
		meas.at<float>(3) = (float)currArea.height;

		if (!detected)
		{
			detected = true; //如果没有检测到人，则设置为true

			state.at<float>(0) = currDetection.x; //更新状态量
			state.at<float>(1) = currDetection.y;
			state.at<float>(2) = 0;
			state.at<float>(3) = 0;
			state.at<float>(4) = (float)currArea.width;
			state.at<float>(5) = (float)currArea.height;

			kf.statePost = state;
		}
		else
		{
			kf.correct(meas);
		}
	}
	else
	{
		notFoundCount++; //如果没有检测到人，则计数器加1

		detectionThreshold -= notFoundCount * 0.01; //更新检测阈值
		enlargeSearchRoi();							//更新搜索区域

		if (notFoundCount >= 30) //如果没有检测到人30帧，则重置状态量
		{
			detected = false;

			state.at<float>(0) = frameSize.width / 2;
			state.at<float>(1) = frameSize.height / 2;
		}
		else
		{
			kf.statePost = state;
		}
	}

	cv::rectangle(temp, searchRoi, cv::Scalar(0, 255, 0)); //画出搜索区域
	cv::scaleAdd(temp, 0.95, image, image);				   //将搜索区域画到图像上

	frameNumber++;

	detectionThreshold = std::min(detectionThreshold, MAX_THRESHOLD);
	detectionThreshold = std::max(detectionThreshold, MIN_THRESHOLD);

	if (cvGetWindowHandle(windowTitle.c_str()))
	{
		cv::imshow(windowTitle, image);
	}

	printFunction("Frame time: " + boost::to_string(dT * 1000) + "ms.");
}

//------------hog-training---------------------------------------

void hogTraining() //训练HOG模型
{
	isTraining = true; //设置为true，表示正在训练

	double t = (double)cv::getTickCount(); //获取时钟数

	std::vector<float> model;

	featuresMutex.lock();
	cv::Mat data = trainData.clone();	  //复制训练数据
	cv::Mat labels = trainLabels.clone(); //复制训练标签
	featuresMutex.unlock();

	model = svm.trainModel(data, labels, detectionHog.getDefaultModel()); //训练模型

	hogMutex.lock();					//锁定HOG模型
	detectionHog.setSVMDetector(model); //设置模型
	hogMutex.unlock();

	t -= (double)cv::getTickCount();

	printFunction("Training time = " + boost::to_string(-(t * 1000 / cv::getTickFrequency())) + "ms.");

	isTraining = false; //设置为false，表示训练完成
}

void computeFeatures(cv::Mat image, cv::Rect selection) //提取特征
{
	try
	{
		cv::Mat resized(trainingHog.winSize, CV_8UC3); //创建缩放图像

		resize(image(selection), resized, trainingHog.winSize, cv::INTER_CUBIC); //缩放图像

		cv::Rect roi1(0, 0, selection.x, image.rows);
		cv::Rect roi2(selection.br().x, 0, image.cols - selection.br().x, image.rows);

		featuresMutex.lock();

		computeFeatureVector(resized, POSITIVE); //提取正样本特征

		if (roi1.width > trainingHog.winSize.width) //如果左边区域宽度大于窗口宽度，则提取特征
		{
			computeFeatureVector(image(roi1), NEGATIVE); //提取负样本特征
		}

		if (roi2.width > trainingHog.winSize.width) //如果右边区域宽度大于窗口宽度，则提取特征
		{
			computeFeatureVector(image(roi2), NEGATIVE); //提取负样本特征
		}

		if (trainData.rows > MAX_TRAIN) //训练数据
		{
			trainData = trainData.rowRange(trainData.rows - MAX_TRAIN, trainData.rows).clone();			//训练数据
			trainLabels = trainLabels.rowRange(trainLabels.rows - MAX_TRAIN, trainLabels.rows).clone(); //训练标签
		}

		featuresMutex.unlock();
	}
	catch (ros::Exception &e)
	{
		ROS_ERROR("%s\n", e.what());
	}
	catch (std::exception &e)
	{
		ROS_ERROR("%s\n", e.what());
	}
}

void computeFeatureVector(cv::Mat image, int label) //提取特征
{
	if (label == POSITIVE)
	{
		//提取正样本特征
		cv::Mat scale = image(cv::Rect(image.cols / 2 - trainingHog.winSize.width / 2, image.rows / 2 - trainingHog.winSize.height / 2, trainingHog.winSize.width, trainingHog.winSize.height)).clone();

		std::vector<float> desc; //特征向量

		trainingHog.compute(scale, desc); //计算特征向量

		cv::Mat data(1, desc.size(), CV_32FC1, desc.data());

		trainData.push_back(data); //训练数据

		trainLabels.push_back(POSITIVE); //训练标签
	}
	else if (label == NEGATIVE)
	{
		srand(time(NULL)); //随机数种子

		// take 10 random windows of each negative image
		for (int j = 0; j < 10; j++)
		{
			try
			{
				cv::Point pt;
				cv::Size sc;

				// random width and height
				sc.height = rand() % (image.rows - trainingHog.winSize.height) + trainingHog.winSize.height - 1; //随机高度

				sc.width = cvRound((double)sc.height / (double)trainingHog.winSize.height * (double)trainingHog.winSize.width); //随机宽度

				if (sc.width > image.cols)
				{
					sc.width = image.cols;
					sc.height = cvRound((double)sc.width / (double)trainingHog.winSize.width * (double)trainingHog.winSize.height); //随机高度
				}

				// random top-left point
				pt.x = rand() % (image.cols - sc.width + 1) - 1; //随机x坐标

				if (pt.x < 0)
				{
					pt.x = 0;
				}

				pt.y = rand() % (image.rows - sc.height + 1) - 1;

				if (pt.y < 0)
				{
					pt.y = 0;
				}

				cv::Mat scale;

				resize(image(cv::Rect(pt, sc)), scale, trainingHog.winSize); //缩放图像

				std::vector<float> desc;

				trainingHog.compute(scale, desc); //计算特征向量

				cv::Mat data(1, desc.size(), CV_32FC1, desc.data()); //训练数据

				trainData.push_back(data); //训练数据

				trainLabels.push_back(NEGATIVE); //训练标签
			}
			catch (std::exception &e)
			{
				ROS_ERROR("%s\n", e.what());
			}
		}
	}
}

void enlargeSearchRoi()
{
	searchRoi.width += 6 * notFoundCount; //搜索区域宽度
	searchRoi.height += 6 * notFoundCount;
	searchRoi.x -= 3 * notFoundCount; //搜索区域x坐标
	searchRoi.y -= 3 * notFoundCount;

	if (searchRoi.x < 0) //搜索区域x坐标
	{
		searchRoi.x = 0;
	}
	if (searchRoi.y < 0) //搜索区域y坐标
	{
		searchRoi.y = 0;
	}
	if (searchRoi.br().x > frameSize.width) //搜索区域右边界x坐标
	{
		searchRoi.width -= searchRoi.br().x - frameSize.width;
	}
	if (searchRoi.br().y > frameSize.height) //搜索区域右边界y坐标
	{
		searchRoi.height -= searchRoi.br().y - frameSize.height;
	}
}

void printFunction(std::string message)
{
	if (print)
	{
		ROS_INFO(message.c_str());
	}
}
//--------------------------------------------------
