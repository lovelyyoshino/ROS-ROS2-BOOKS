hog_haar_person_detection
=============================

A ROS Indigo package implementing the [OpenCV HOG pedestrian detector](http://docs.opencv.org/modules/gpu/doc/object_detection.html)[1] and [HAAR face detector](http://docs.opencv.org/trunk/doc/py_tutorials/py_objdetect/py_face_detection/py_face_detection.html).
Subscribes to an image being published, detects faces and standing pedestrians in the image,
draws the detections on a copy of the image and publishes results as a rostopic. Also displays
detections on an OpenCV window.

## Usage
[Install ROS](http://wiki.ros.org/ROS/Installation)

* $ cd [your indigo catkin src folder]
* $ git clone https://github.com/angusleigh/hog_haar_person_detection.git
* $ cd ..
* $ catkin_make
* $ roslaunch hog_haar_person_detection hog_haar_person_detection.launch
* Launch your camera node and have it publish to the "image_topic" parameter sepecified in hog_haar_person_detection.launch. Check out the [open-ni](http://wiki.ros.org/openni_launch) node for the Kinect or [USB-Cam](http://wiki.ros.org/usb_cam) node for USB cameras.
* Detected faces and pedestrians should be published to /person_detection/faces and /person_detection/pedestrians topics.

[1] Link is for GPU documentation. I'm not aware of any documentation for the CPU implementaion (which is the one used for this repo).

## Similar packages

* [spencer people tracking](https://github.com/spencer-project/spencer_people_tracking)
* [cob_people_detection](http://wiki.ros.org/cob_people_detection)
* [face_detector](http://wiki.ros.org/face_detector)
