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

//ROS Callback
void imageCallback(const sensor_msgs::ImageConstPtr&);

//HOG model training function
void hogTraining();

//Computes the features of positive/negative image
void computeFeatures(cv::Mat, cv::Rect);
void computeFeatureVector(cv::Mat, int);

void enlargeSearchRoi();
void printFunction(std::string);

void showHelpMessage(std::string);

//HOGDescriptor for detection
HOGDetector detectionHog;

//HOGDescriptor for training
HOGDetector trainingHog;

//Current frame
int frameNumber = 0;

//Training flag
bool isTraining = false;

//Region of interest
cv::Rect searchRoi(0, 0, frameSize.width, frameSize.height);

//Mutex variables to handle concurrency
boost::mutex hogMutex;
boost::mutex featuresMutex;

//SVM for learning
LinearSVM svm;

ros::Publisher p;

cv::Mat trainData, trainLabels;

bool detected = false;
bool print = true;

//Kalman filter
int stateSize = 6;
int measSize = 4;
int contrSize = 0;
cv::KalmanFilter kf(stateSize, measSize, contrSize);

cv::Mat state(stateSize, 1, CV_32F);
cv::Mat meas(measSize, 1, CV_32F);

double ticks = 0;
int notFoundCount = 0;

double searchRoiRatio;
double detectionThreshold = MIN_THRESHOLD;

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

	image_transport::ImageTransport it(nh);

	image_transport::Subscriber sub = it.subscribe(camera_t, 1, imageCallback);

	p = nh.advertise<adaptive_hog_people_tracker::BoundingBox>(bounding_box_t, 1);

	if(verbosity_level == 0)
	{
		print = false;
	}
	else if(verbosity_level == 1)
	{
		print = true;
	}
	else
	{
		ROS_ERROR("Param verbosity_level must be 0 (no output) or 1 (full output)!");
		return 1;
	}

	if(detector_type == "full")
	{
		detectionHog = HOGDetector(HOGDetector::FULL);
		trainingHog = HOGDetector(HOGDetector::FULL);
	}
	else if(detector_type == "torso")
	{
		detectionHog = HOGDetector(HOGDetector::TORSO);
		trainingHog = HOGDetector(HOGDetector::TORSO);
	}

	searchRoiRatio = detectionHog.getSearchRoiRatio();

    cv::startWindowThread();

	cv::namedWindow(windowTitle);

    cv::setIdentity(kf.transitionMatrix);

	kf.errorCovPre.at<float>(0) = 1;
	kf.errorCovPre.at<float>(7) = 1;
	kf.errorCovPre.at<float>(14) = 1;
	kf.errorCovPre.at<float>(21) = 1;
	kf.errorCovPre.at<float>(28) = 1;
	kf.errorCovPre.at<float>(35) = 1;

	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(16) = 1.0f;
	kf.measurementMatrix.at<float>(23) = 1.0f;

	kf.processNoiseCov.at<float>(0) = 1e-2;
	kf.processNoiseCov.at<float>(7) = 1e-4;
	kf.processNoiseCov.at<float>(14) = 1e-3;
	kf.processNoiseCov.at<float>(21) = 1e-10;
	kf.processNoiseCov.at<float>(28) = 1e-4;
	kf.processNoiseCov.at<float>(35) = 1e-4;

	state.at<float>(0) = frameSize.width / 2;
	state.at<float>(1) = frameSize.height / 2;

	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-2));

	while(ros::ok())
	{
		if(!cvGetWindowHandle(windowTitle.c_str()))
		{
			break;
		}

		ros::spinOnce();
	}

	cv::destroyAllWindows();
	ros::shutdown();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv::Mat image;

	try
	{
		image = cv_bridge::toCvShare(msg, "bgr8")->image;

		if (image.empty())
		{
			return;
		}
	}
	catch (cv_bridge::Exception& e)
	{
		//Conversion failed
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

		return;
	}

	cv::resize(image, image, frameSize);

	double precTick = ticks;

	ticks = (double) cv::getTickCount();

	double dT = (ticks - precTick) / cv::getTickFrequency();

	// temp image for drawing
	cv::Mat temp(image.size(), CV_8UC3);
	temp.setTo(cv::Scalar::all(0));

	adaptive_hog_people_tracker::BoundingBox par;

	if(detected)
	{
		// >>>> Matrix A
		kf.transitionMatrix.at<float>(2) = dT;
		kf.transitionMatrix.at<float>(9) = dT;
		// <<<< Matrix A

		state = kf.predict();

		cv::Rect predRect;
		predRect.width = state.at<float>(4);
		predRect.height = state.at<float>(5);
		predRect.x = state.at<float>(0) - predRect.width / 2;
		predRect.y = state.at<float>(1) - predRect.height / 2;

		searchRoi.width = predRect.width * searchRoiRatio;
		searchRoi.height = predRect.height + (searchRoi.width - predRect.width);
		searchRoi.x = predRect.x - ((searchRoi.width - predRect.width) / 2);
		searchRoi.y = predRect.y - ((searchRoi.width - predRect.width) / 2);

		if(searchRoi.x < 0)
		{
			searchRoi.x = 0;
		}
		if(searchRoi.y < 0)
		{
			searchRoi.y = 0;
		}
		if(searchRoi.br().x > frameSize.width)
		{
			searchRoi.width -= searchRoi.br().x - frameSize.width;
		}
		if(searchRoi.br().y > frameSize.height)
		{
			searchRoi.height -= searchRoi.br().y - frameSize.height;
		}

		cv::Point center;
		center.x = state.at<float>(0);
		center.y = state.at<float>(1);

		par.x = center.x;
		par.y = center.y;
		par.width = predRect.width;
		par.height = predRect.height;

		cv::rectangle(temp, predRect, cv::Scalar(0, 0, 255), 2);
		cv::circle(temp, center, 1, cv::Scalar(0, 0, 255), 2);
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

	if(frameNumber > 10)
	{
		hogMutex.lock();
		detectionHog.detectMultiScale(image(searchRoi), found, confidence, detectionThreshold, cv::Size(), cv::Size(), 1.05, 1, false);
		hogMutex.unlock();
	}

	cv::Rect currArea;
	cv::Point currDetection;

	double distance = cv::norm(cv::Point(0, 0) - cv::Point(frameSize.width, frameSize.height));

	for(int i = 0; i < found.size(); i++)
	{
		cv::Rect r = found[i];

		int j;

		for(j = 0; j < found.size(); j++)
		{
			if(j != i && (r & found[j]) == r)
			{
				break;
			}
		}

		if(j == found.size())
		{
			//----da roi a immagine----
			r.x += searchRoi.x;
			r.y += searchRoi.y;

			if(r.x < 0)
			{
				r.x = 0;
			}
			if(r.y < 0)
			{
				r.y = 0;
			}
			if(r.br().x > frameSize.width)
			{
				r.width -= r.br().x - frameSize.width;
			}
			if(r.br().y > frameSize.height)
			{
				r.height -= r.br().y - frameSize.height;
			}

			cv::Point r_center;

			r_center.x = r.x + r.width / 2;
			r_center.y = r.y + r.height / 2;

			cv::rectangle(temp, r, cv::Scalar(255, 0, 0), 2);

			if(cv::norm(r_center - cv::Point(state.at<float>(0), state.at<float>(1))) < distance)
			{
				distance = cv::norm(r_center - cv::Point(state.at<float>(0), state.at<float>(1)));
				currDetection = cv::Point(r_center);
				currArea = cv::Rect(r);

				foundAtLeastOne = true;

				detectionThreshold = confidence[i] * 0.6;
			}
		}
	}

	if(foundAtLeastOne)
	{
		//-------Collecting Samples--------
		if(frameNumber % SKIP_ADD_SAMPLES == 0)
		{
			if(currArea.width > detectionHog.winSize.width / 2 && currArea.height > detectionHog.winSize.height / 2)
			{
				boost::thread t(computeFeatures, image, currArea);
				t.detach();

				//----Automatic Start Training
				if(trainData.rows >= MIN_TRAIN && !isTraining)
				{
					boost::thread t(hogTraining);
					t.detach();
				}
			}
		}

		notFoundCount = 0;

		meas.at<float>(0) = currDetection.x;
		meas.at<float>(1) = currDetection.y;
		meas.at<float>(2) = (float)currArea.width;
		meas.at<float>(3) = (float)currArea.height;

		if(!detected)
		{
			detected = true;

			state.at<float>(0) = currDetection.x;
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
		notFoundCount++;

		detectionThreshold -= notFoundCount * 0.01;
		enlargeSearchRoi();

		if(notFoundCount >= 30)
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

	cv::rectangle(temp, searchRoi, cv::Scalar(0, 255, 0));
	cv::scaleAdd(temp, 0.95, image, image);

	frameNumber++;

	detectionThreshold = std::min(detectionThreshold, MAX_THRESHOLD);
	detectionThreshold = std::max(detectionThreshold, MIN_THRESHOLD);

	if(cvGetWindowHandle(windowTitle.c_str()))
	{
		cv::imshow(windowTitle, image);
	}

	printFunction("Frame time: " + boost::to_string(dT * 1000) + "ms.");
}

//------------hog-training---------------------------------------

void hogTraining()
{
	isTraining = true;

	double t = (double)cv::getTickCount();

	std::vector<float> model;

	featuresMutex.lock();
	cv::Mat data = trainData.clone();
	cv::Mat labels = trainLabels.clone();
	featuresMutex.unlock();

	model = svm.trainModel(data, labels, detectionHog.getDefaultModel());

	hogMutex.lock();
	detectionHog.setSVMDetector(model);
	hogMutex.unlock();

	t -= (double)cv::getTickCount();

	printFunction("Training time = " + boost::to_string(-(t * 1000 / cv::getTickFrequency())) + "ms.");

	isTraining = false;
}

void computeFeatures(cv::Mat image, cv::Rect selection)
{
	try
	{
		cv::Mat resized(trainingHog.winSize, CV_8UC3);

		resize(image(selection), resized, trainingHog.winSize, cv::INTER_CUBIC);

		cv::Rect roi1(0, 0, selection.x, image.rows);
		cv::Rect roi2(selection.br().x, 0, image.cols - selection.br().x, image.rows);

		featuresMutex.lock();

		computeFeatureVector(resized, POSITIVE);

		if(roi1.width > trainingHog.winSize.width)
		{
			computeFeatureVector(image(roi1), NEGATIVE);
		}

		if(roi2.width > trainingHog.winSize.width)
		{
			computeFeatureVector(image(roi2), NEGATIVE);
		}

		if(trainData.rows > MAX_TRAIN)
		{
			trainData = trainData.rowRange(trainData.rows - MAX_TRAIN, trainData.rows).clone();
			trainLabels = trainLabels.rowRange(trainLabels.rows - MAX_TRAIN, trainLabels.rows).clone();
		}

		featuresMutex.unlock();
	}
	catch(ros::Exception& e)
	{
		ROS_ERROR("%s\n", e.what());
	}
	catch (std::exception& e)
	{
		ROS_ERROR("%s\n", e.what());
	}
}

void computeFeatureVector(cv::Mat image, int label)
{
	if(label == POSITIVE)
	{
		cv::Mat scale =  image(cv::Rect(image.cols / 2 - trainingHog.winSize.width / 2, image.rows / 2 - trainingHog.winSize.height / 2, trainingHog.winSize.width, trainingHog.winSize.height)).clone();

		std::vector<float> desc;

		//compute feature vector
		trainingHog.compute(scale, desc);

		cv::Mat data(1, desc.size(), CV_32FC1, desc.data());

		trainData.push_back(data);

		trainLabels.push_back(POSITIVE);
	}
	else if(label == NEGATIVE)
	{
		srand(time(NULL));

		//take 10 random windows of each negative image
		for(int j = 0; j < 10; j++)
		{
			try
			{
				cv::Point pt;
				cv::Size sc;

				//random width and height
				sc.height = rand() % (image.rows - trainingHog.winSize.height) + trainingHog.winSize.height - 1;

				sc.width = cvRound((double)sc.height / (double)trainingHog.winSize.height * (double)trainingHog.winSize.width);

				if(sc.width > image.cols)
				{
					sc.width = image.cols;
					sc.height = cvRound((double)sc.width / (double)trainingHog.winSize.width * (double)trainingHog.winSize.height);
				}

				//random top-left point
				pt.x = rand() % (image.cols - sc.width + 1) - 1;

				if(pt.x < 0)
				{
					pt.x = 0;
				}

				pt.y = rand() % (image.rows - sc.height + 1) - 1;

				if(pt.y < 0)
				{
					pt.y = 0;
				}

				cv::Mat scale;

				resize(image(cv::Rect(pt, sc)), scale, trainingHog.winSize);

				std::vector<float> desc;

				trainingHog.compute(scale, desc);

				cv::Mat data(1, desc.size(), CV_32FC1, desc.data());

				trainData.push_back(data);

				trainLabels.push_back(NEGATIVE);
			}
			catch(std::exception& e)
			{
				ROS_ERROR("%s\n", e.what());
			}
		}
	}
}

void enlargeSearchRoi()
{
	searchRoi.width += 6 * notFoundCount;
	searchRoi.height += 6 * notFoundCount;
	searchRoi.x -= 3 * notFoundCount;
	searchRoi.y -= 3 * notFoundCount;

	if(searchRoi.x < 0)
	{
		searchRoi.x = 0;
	}
	if(searchRoi.y < 0)
	{
		searchRoi.y = 0;
	}
	if(searchRoi.br().x > frameSize.width)
	{
		searchRoi.width -= searchRoi.br().x - frameSize.width;
	}
	if(searchRoi.br().y > frameSize.height)
	{
		searchRoi.height -= searchRoi.br().y - frameSize.height;
	}
}

void printFunction(std::string message)
{
	if(print)
	{
		ROS_INFO(message.c_str());
	}
}
//--------------------------------------------------
