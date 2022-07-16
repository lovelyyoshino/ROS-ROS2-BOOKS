#!/usr/bin/env python
# -*- coding: utf-8 -*-

# visualizer.py: rviz visualizer
# Author: Ravi Joshi
# Date: 2019/10/01

# import modules
import math
import rospy
from ros_openpose.msg import Frame
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray


class RealtimeVisualization():
    def __init__(self, ns, frame_topic, skeleton_frame, id_text_size, id_text_offset, skeleton_hands, skeleton_line_width):
        self.ns = ns
        self.skeleton_frame = skeleton_frame
        self.id_text_size = id_text_size
        self.id_text_offset = id_text_offset
        self.skeleton_hands = skeleton_hands
        self.skeleton_line_width = skeleton_line_width

        # define a few colors we are going to use later on
        self.colors = [ColorRGBA(0.12, 0.63, 0.42, 1.00),
                       ColorRGBA(0.98, 0.30, 0.30, 1.00),
                       ColorRGBA(0.26, 0.09, 0.91, 1.00),
                       ColorRGBA(0.77, 0.44, 0.14, 1.00),
                       ColorRGBA(0.92, 0.73, 0.14, 1.00),
                       ColorRGBA(0.00, 0.61, 0.88, 1.00),
                       ColorRGBA(1.00, 0.65, 0.60, 1.00),
                       ColorRGBA(0.59, 0.00, 0.56, 1.00)]# 设置不同的颜色用于骨骼点显示

        '''
        The skeleton is considered as a combination of line strips.
        Hence, the skeleton is decomposed into 3 LINE_STRIP as following:
            1) upper_body : from nose to mid hip
            2) hands : from left-hand wrist to right-hand wrist
            3) legs : from left foot toe to right foot toe

        See the link below to get the id of each joint as defined in Kinect v2
        src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#keypoint-ordering
        Result for BODY_25 (25 body parts consisting of COCO + foot)
        const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS {
            { 0,      "Nose"},    {13,      "LKnee"}
            { 1,      "Neck"},    {14,     "LAnkle"}
            { 2, "RShoulder"},    {15,       "REye"}
            { 3,    "RElbow"},    {16,       "LEye"}
            { 4,    "RWrist"},    {17,       "REar"}
            { 5, "LShoulder"},    {18,       "LEar"}
            { 6,    "LElbow"},    {19,    "LBigToe"}
            { 7,    "LWrist"},    {20,  "LSmallToe"}
            { 8,    "MidHip"},    {21,      "LHeel"}
            { 9,      "RHip"},    {22,    "RBigToe"}
            {10,     "RKnee"},    {23,  "RSmallToe"}
            {11,    "RAnkle"},    {24,      "RHeel"}
            {12,      "LHip"},    {25, "Background"}


        hand output ordering
        src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/raw/master/doc/media/keypoints_hand.png
        We are using 5 LINE_STRIP to draw a hand
        '''

        self.upper_body_ids = [0, 1, 8]# 设置身体的骨骼点索引
        self.hands_ids = [4, 3, 2, 1, 5, 6, 7]# 设置手的骨骼点索引
        self.legs_ids = [22, 11, 10, 9, 8, 12, 13, 14, 19]# 设置腿的骨骼点索引
        self.body_parts = [self.upper_body_ids, self.hands_ids, self.legs_ids]# 设置整个身体的骨骼点索引

        # number of fingers in a hand
        self.fingers = 5# 设置手的骨骼点个数

        # number of keypoints to denote a finger
        self.count_keypoints_one_finger = 5# 设置手指的骨骼点个数

        self.total_finger_kepoints = self.fingers * self.count_keypoints_one_finger# 设置整个手的骨骼点个数

        # write person id on the top of his head
        self.nose_id = 0# 设置鼻子的骨骼点索引

        # define a publisher to publish the 3D skeleton of multiple people
        self.skeleton_pub = rospy.Publisher(self.ns, MarkerArray, queue_size=1)# 定义一个发布器，用于发布骨骼点的信息

        # define a subscriber to retrive tracked bodies
        rospy.Subscriber(frame_topic, Frame, self.frame_callback)


    def spin(self):
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        rospy.spin()


    def create_marker(self, index, color, marker_type, size, time):
        '''
        Function to create a visualization marker which is used inside RViz
        '''
        marker = Marker()# 创建一个标记点
        marker.id = index  # 设置标记点的id
        marker.ns = self.ns# 设置标记点的命名空间
        marker.color = color# 设置标记点的颜色
        marker.action = Marker.ADD# 设置标记点的操作类型
        marker.type = marker_type# 设置标记点的类型
        marker.scale = Vector3(size, size, size)# 设置标记点的大小
        marker.pose.orientation.w = 1# 设置标记点的方向
        marker.header.stamp = time
        marker.header.frame_id = self.skeleton_frame
        marker.lifetime = rospy.Duration(1)  # 1 second
        return marker


    def isValid(self, bodyPart):
        '''
        何时我们应该将主体部分视为有效实体?  

        我们确保分数和z坐标是一个正数。  

        注意, z坐标表示所处对象的距离  

        在镜头前。 因此它总是一个正数。 
        '''
        return bodyPart.score > 0 and not math.isnan(bodyPart.point.x) and not math.isnan(bodyPart.point.y) and not math.isnan(bodyPart.point.z) and bodyPart.point.z > 0


    def frame_callback(self, data):
        '''
        每当订阅者接收到消息时，将调用此函数 
        '''
        marker_counter = 0
        person_counter = 0
        marker_array = MarkerArray()

        for person in data.persons:# 循环遍历每一个人
            now = rospy.Time.now()
            marker_color = self.colors[person_counter % len(self.colors)]# 设置标记点的颜色

            # body_marker包含前面提到的三个标记  
            # # 1. 上半身 2. 手 3. 腿 
            body_marker = [self.create_marker(marker_counter + idx, marker_color, Marker.LINE_STRIP, self.skeleton_line_width, now) for idx in range(len(self.body_parts))]
            marker_counter += len(self.body_parts)# 标记点的索引加1

            # 为每个身体部位分配3D位置  
            # 确保只考虑身体的有效部位 
            for index, body_part in enumerate(self.body_parts):
                body_marker[index].points = [person.bodyParts[idx].point for idx in body_part if self.isValid(person.bodyParts[idx])]# 设置标记点的位置

            marker_array.markers.extend(body_marker)# 将标记点添加到标记点数组中

            if self.skeleton_hands:
                left_hand = [self.create_marker(marker_counter + idx, marker_color, Marker.LINE_STRIP, self.skeleton_line_width, now) for idx in range(self.fingers)]# 创建左手的标记点
                marker_counter += self.fingers# 标记点的索引加1

                right_hand = [self.create_marker(marker_counter + idx, marker_color, Marker.LINE_STRIP, self.skeleton_line_width, now) for idx in range(self.fingers)]# 创建右手的标记点
                marker_counter += self.fingers# 标记点的索引加1

                keypoint_counter = 0
                for idx in range(self.total_finger_kepoints):
                    strip_id = idx / self.count_keypoints_one_finger# 计算标记点的除数
                    temp_id = idx % self.count_keypoints_one_finger# 计算标记点的余数
                    if temp_id == 0:#如果被整除
                        point_id = temp_id# 设置标记点的id
                    else:
                        keypoint_counter += 1# 标记点的索引加1
                        point_id = keypoint_counter# 设置标记点的id

                    leftHandPart = person.leftHandParts[point_id]# 获取左手的标记点
                    rightHandPart = person.rightHandParts[point_id]# 获取右手的标记点
                    if self.isValid(leftHandPart):
                        left_hand[strip_id].points.append(leftHandPart.point)# 将标记点的位置添加到标记点的位置列表中

                    if self.isValid(rightHandPart):
                        right_hand[strip_id].points.append(rightHandPart.point)# 将标记点的位置添加到标记点的位置列表中
                marker_array.markers.extend(left_hand)
                marker_array.markers.extend(right_hand)

            person_id = self.create_marker(marker_counter, marker_color, Marker.TEXT_VIEW_FACING, self.id_text_size, now)# 创建标记点
            marker_counter += 1
            # assign person id and 3D position
            person_id.text = str(person_counter)# 设置标记点的文本
            nose = person.bodyParts[self.nose_id]
            if self.isValid(nose):
                person_id.pose.position = Point(nose.point.x, nose.point.y + self.id_text_offset, nose.point.z)# 设置标记点的位置
                marker_array.markers.append(person_id)# 将标记点添加到标记点数组中

            # update the counter
            person_counter += 1# 标记点的索引加1

        # publish the markers
        self.skeleton_pub.publish(marker_array)# 发布标记点数组


if __name__ == '__main__':
    # define some constants
    ns = 'visualization'

    # initialize ros node
    rospy.init_node('visualizer_node', anonymous=False)

    # read the parameters from ROS parameter server
    frame_topic = rospy.get_param('~pub_topic')
    skeleton_frame = rospy.get_param('~frame_id')
    id_text_size = rospy.get_param('~id_text_size')
    id_text_offset = rospy.get_param('~id_text_offset')
    skeleton_hands = rospy.get_param('~skeleton_hands')
    skeleton_line_width = rospy.get_param('~skeleton_line_width')

    # instantiate the RealtimeVisualization class
    visualization = RealtimeVisualization(ns, frame_topic, skeleton_frame, id_text_size, id_text_offset, skeleton_hands, skeleton_line_width)
    visualization.spin()
