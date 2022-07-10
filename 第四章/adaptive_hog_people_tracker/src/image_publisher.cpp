#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <FrameSize.hpp>

int main(int argc, char **argv)
{
	/* The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "image_publisher");

	/* NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	/* The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	image_transport::Publisher pub = it.advertise("camera/image", 1);

	cv::VideoCapture cap;

	if(argc > 1)
	{
		//Start the capture from main camera (use 1 for secondary camera)
		cap.open(argv[1]);
	}
	else
	{
		cap.open(0);
	}

	cap.set(CV_CAP_PROP_FRAME_WIDTH, frameSize.width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, frameSize.height);

	//Check if capture is opened
	if(!cap.isOpened())
	{
		return 1;
	}
	else
	{
		//Create an image to store the video screen grab
		cv::Mat frame;

		//This is a message object. You stuff it with data, and then publish it.
		sensor_msgs::ImagePtr msg;

		ros::Rate loop_rate(25);

		//Create a loop to update the image with video camera image capture
		while(nh.ok())
		{
			//Grab a frame from the video camera
			cap >> frame;

			//Check if camera is actually capturing frames
			if(!frame.empty())
			{
				//Convert the frame from cv::Mat to ros::sensor_msg
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

				/* The publish() function is how you send messages. The parameter
				 * is the message object. The type of this object must agree with the type
				 * given as a template parameter to the advertise<>() call, as was done
				 * in the constructor above.
				 */
				pub.publish(msg);
			}
			else
			{
				//Capture is opened but is not receiving frames
				cap.release();
				return 2;
			}

			loop_rate.sleep();
		}

		//Release the capture
		cap.release();

		return 0;
	}
}
