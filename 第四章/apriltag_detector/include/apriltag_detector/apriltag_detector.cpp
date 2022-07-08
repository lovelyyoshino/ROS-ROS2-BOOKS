#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <queue>
#include "apriltag_detector/DetectionTag.h"

using namespace std;

#define PI 3.141592
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(X) ((X)*PI/180.)

std::queue<string> tagQue; // store detecting avable tag repeatly
boost::mutex que_mutex; // for worker thread checking Queue mutex

// publish detection tag using thread pool
marvel_core::DetectionTag tag_msgs;
ros::Publisher tag_pub;
boost::mutex msg_mutex;

// thread workers childs, parent -> have reader writer problem
boost::mutex rc_mutex;
int r_count = 0;
boost::mutex rw_mutex;

tf2_ros::Buffer tfBuffer; // threads use tfBuffer Only Reader. not Necessary mutex 

class TagDetector
{
public:
	TagDetector(ros::NodeHandle& nh);
	~TagDetector();

private:
	void ParamGet();
	void ParamPrint();
	static void TagPublisher(string* Available_Tag_,string baseFrame,int tag_num_,const int worker_num);
	static void WorkerThread(string baseFrame);

	ros::NodeHandle node_;

	// Parameter Concerned with AprilTag
	std::vector<string> tag_;
	string* Available_Tag_;
	int tag_num_;

	// tag detector thread ( managing thread pool)
	boost::thread tag_thr_;
	int worker_num_;
}

TagDetector::TagDetector(ros::NodeHandle& nh)
: tfListener_(tfBuffer), node_(nh)
{
	ParamGet();
	tag_pub = node_.advertise<marvel_core::DetectionTag>("tag_detector",10);
	tag_thr_ = boost::thread(&TagDetector::TagPublisher, Available_Tag_,baseFrame_, tag_num_,worker_num_);
}
TagDetector::~TagDetector()
{
	tag_thr_.join();
	delete[] Available_Tag_;
}
void TagDetector::TagPublisher(string* Available_Tag_,string baseFrame,int tag_num,const int worker_num)
{
	// make worker thread
	boost::thread worker[worker_num];
	for(int i=0; i<worker_num ; i++)
		worker[i] = boost::thread(&TagSlam::WorkerThread,baseFrame);

	que_mutex.lock();
	for(int i=0;i<tag_num;i++)
		tagQue.push(Available_Tag_[i]);
	que_mutex.unlock();

	ros::Rate rate(10.0);
	while(ros::ok())
	{
		rw_mutex.lock();
		que_mutex.lock();
		if(tagQue.empty())
		{
			cout << "tagQueue : ";
			for(int i=0;i<tag_num;i++)
			{
				tagQue.push(Available_Tag_[i]);
			}
			cout << "parent thread queue fulling" << endl;
			// que_mutex.unlock();

			// publish detection tag msg
			msg_mutex.lock();
			tag_pub.publish(tag_msgs);
			tag_msgs.tag.clear();
			msg_mutex.unlock();
		}
		que_mutex.unlock();
		rw_mutex.unlock();

		rate.sleep();
	}
	for(int i=0; i<worker_num ; i++)
		worker[i].join();

	printf(" end of parent thread \n");
}
void TagSlam::WorkerThread(string baseFrame)
{
	ros::Rate rate(10);
	while(ros::ok())
	{

		rc_mutex.lock();
		r_count++;
		if(r_count == 1)
			rw_mutex.lock();
		rc_mutex.unlock();

		que_mutex.lock();
		if(!tagQue.empty())
		{
			string tagFrame = tagQue.front();
			tagQue.pop();
			que_mutex.unlock();
			cout << "checking frame : "<< tagFrame << endl;
			try
			{
				tfBuffer.lookupTransform(baseFrame,tagFrame,
				ros::Time::now(),ros::Duration(0.2));
			}
			catch(tf2::TransformException &ex)
			{
				goto skip;
			}
			msg_mutex.lock();
			tag_msgs.tag.push_back(tagFrame);
			msg_mutex.unlock();	
		}
		else
		{
			que_mutex.unlock();
		}
		skip:
		rc_mutex.lock();
		r_count--;
		if(r_count == 0)
			rw_mutex.unlock();
		rc_mutex.unlock();
		
		
		rate.sleep();
	}
	printf(" end of worker Thread\n");
	return ;
}

// bring related data by Config file
void TagDetector::ParamGet()
{
	if(!node_.getParam("BaseFrame",baseFrame_))
		baseFrame_ = "base_footprint";
	if(!node_.getParam("Tag",tag_)){
		tag_.assign(1,"tag_0");
		ROS_WARN("Please Describe 'Tag : ' You Want to Detect tag name in Config File");
	}
	tag_num_ = (int)tag_.size();
	Available_Tag_ = new string[tag_num_];
	for(int i=0;i<tag_num_;i++)
		Available_Tag_[i] = tag_[i].c_str();

	if(!node_.getParam("WorkerThread",worker_num_))
		worker_num_ = 4;
}
void TagDetector::ParamPrint()
{
	printf("==============Tag Parameter Check=====================\n");
	printf("BaseFrame : %s \n",baseFrame_.c_str());
	// printf("Detection Available tag_number : %d\n",tag_num_);
	printf("Detection Available Tag : ");
	for(int i=0;i<tag_num_;i++)
		printf("%s ",tag_[i].c_str());
	printf("\n");
}
