import rospy
from mavros_msgs.msg import State, PositionTarget
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from sensor_msgs.msg import CameraInfo, RegionOfInterest
import math
from image_geometry import PinholeCameraModel
import time
from std_msgs.msg import String, Header


class OffbPosCtl:
    curr_pose = PoseStamped()#保存当前的位置
    waypointIndex = 0#起降点索引
    distThreshold = 0.4#位置容忍距离为0.4m
    des_pose = PoseStamped()#目标位置
    saved_location = None#是否保存定位
    isReadyToFly = False#是否准备起飞
    hover_loc = [4, 0, 10, 0, 0, 0, 0]
    mode = "SURVEY"
    KP = 0.008#PID参数
    KD = 0.0004
    KI = 0.00005
    x_sum_error = 0#x的累计误差
    y_sum_error = 0
    x_prev_error = 0#x上一帧的误差
    y_prev_error = 0
    x_change = 1
    y_change = 1
    camera = PinholeCameraModel()#相机为针孔相机模型，计算相机下的点的坐标

    vel_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)# 发布定位信息
    pub = rospy.Publisher('/data', String, queue_size=10)
    tag_pt = None
    detection_count = 0
    missing_count = 0

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)#初始化节点
        self.loc = []
        mocap_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.mocap_cb)#订阅mavros给出的相对位置
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_cb)#订阅mavros的状态
        state = rospy.Subscriber('/dreams/state', String, callback=self.update_state_cb)#订阅飞机飞行模式
        tag_detect = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback=self.tag_detections)#订阅apriltag的位置
        decision = rospy.Subscriber('/data', String, callback=self.planner)#订阅规划结果
        self.camera.fromCameraInfo(self.camera_info())#根据相机信息设置相机模型
        self.controller()

    def copy_pose(self, pose):#复制位姿
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def camera_info(self):#获取相机信息
        msg_header = Header()
        msg_header.frame_id = "f450/robot_camera_down"#相机的名称
        msg_roi = RegionOfInterest()#相机的ROI
        msg_roi.x_offset = 0
        msg_roi.y_offset = 0
        msg_roi.height = 0
        msg_roi.width = 0
        msg_roi.do_rectify = 0

        msg = CameraInfo()
        msg.header = msg_header#相机的信息
        msg.height = 480#相机的高度
        msg.width = 640#相机的宽度
        msg.distortion_model = 'plumb_bob'#相机的模型
        msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]#相机的畸变参数
        msg.K = [1.0, 0.0, 320.5, 0.0, 1.0, 240.5, 0.0, 0.0, 1.0]#相机的内参数
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]#相机的外参数
        msg.P = [1.0, 0.0, 320.5, -0.0, 0.0, 1.0, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]#相机的投影矩阵
        msg.binning_x = 0#相机的x方向的binning
        msg.binning_y = 0#相机的y方向的binning
        msg.roi = msg_roi#相机的ROI
        return msg

    def mocap_cb(self, msg):#订阅mavros给出的相对位置
        self.curr_pose = msg

    def state_cb(self, msg):#订阅mavros的状态
        if msg.mode == 'OFFBOARD':#如果是自动驾驶状态
            self.isReadyToFly = True
        else:
            print (msg.mode)

    def update_state_cb(self, data):#订阅飞机飞行模式
        self.mode = data.data
        print (self.mode)

    def tag_detections(self, msgs):#订阅apriltag的位置
        rate = rospy.Rate(30)#设置订阅的频率
        if len(msgs.detections) > 0:#如果有apriltag
            msg = msgs.detections[0].pose#获取apriltag的位置
            self.tag_pt = msg.pose.pose.position
            self.pub.publish("FOUND UAV")#发布找到UAV的信息
        else:
            if self.mode == "DESCENT":#如果是降落模式
                self.pub.publish("MISSING UAV")#发布不要接受UAV的信息

    def get_descent(self, x, y, z):#获取降落的位置
        des_vel = PositionTarget()
        des_vel.header.frame_id = "world"
        des_vel.header.stamp = rospy.Time.from_sec(time.time())#获取当前时间
        des_vel.coordinate_frame = 8#设置坐标系
        des_vel.type_mask = 3527#设置位置类型
        des_vel.velocity.x = x#设置x方向的速度
        des_vel.velocity.y = y
        des_vel.velocity.z = z
        return des_vel

    def lawnmover(self, rect_x, rect_y, height, offset, offset_x):#获取移动的位置
        """ lawnmover pattern over a rectangle of size rect_x and rect_y with increasing step size in x.
            offset - offset to where drone begins search (in x)
            offset_x is the value by which drone moves forward in x direction after each left to right movements
        """
        if len(self.loc) == 0:
            takeoff = [self.curr_pose.pose.position.x, self.curr_pose.pose.position.y, height, 0, 0, 0, 0]
            move_to_offset = [self.curr_pose.pose.position.x + offset,
                              self.curr_pose.pose.position.y - rect_y/2, height, 0, 0, 0, 0]
            self.loc.append(takeoff)
            self.loc.append(move_to_offset)
            left = True
            while True:
                if left:
                    x = self.loc[len(self.loc) - 1][0]
                    y = self.loc[len(self.loc) - 1][1] + rect_y/3
                    z = self.loc[len(self.loc) - 1][2]
                    self.loc.append([x, y, z, 0, 0, 0, 0])
                    x = self.loc[len(self.loc) - 1][0]
                    y = self.loc[len(self.loc) - 1][1] + rect_y/3
                    z = self.loc[len(self.loc) - 1][2]
                    self.loc.append([x, y, z, 0, 0, 0, 0])
                    x = self.loc[len(self.loc) - 1][0]
                    y = self.loc[len(self.loc) - 1][1] + rect_y/3
                    z = self.loc[len(self.loc) - 1][2]
                    self.loc.append([x, y, z, 0, 0, 0, 0])
                    left = False
                    x = self.loc[len(self.loc) - 1][0] + offset_x
                    y = self.loc[len(self.loc) - 1][1]
                    z = self.loc[len(self.loc) - 1][2]
                    self.loc.append([x, y, z, 0, 0, 0, 0])
                    if x > rect_x:
                        break
                else:
                    x = self.loc[len(self.loc) - 1][0]
                    y = self.loc[len(self.loc) - 1][1] - rect_y/3
                    z = self.loc[len(self.loc) - 1][2]
                    self.loc.append([x, y, z, 0, 0, 0, 0])
                    x = self.loc[len(self.loc) - 1][0]
                    y = self.loc[len(self.loc) - 1][1] - rect_y/3
                    z = self.loc[len(self.loc) - 1][2]
                    self.loc.append([x, y, z, 0, 0, 0, 0])
                    x = self.loc[len(self.loc) - 1][0]
                    y = self.loc[len(self.loc) - 1][1] - rect_y/3
                    z = self.loc[len(self.loc) - 1][2]
                    self.loc.append([x, y, z, 0, 0, 0, 0])
                    left = True
                    x = self.loc[len(self.loc) - 1][0] + offset_x
                    y = self.loc[len(self.loc) - 1][1]
                    z = self.loc[len(self.loc) - 1][2]
                    self.loc.append([x, y, z, 0, 0, 0, 0])
                    if x > rect_x:
                        break

        rate = rospy.Rate(10)
        self.des_pose = self.copy_pose(self.curr_pose)
        shape = len(self.loc)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        print self.mode

        while self.mode == "SURVEY" and not rospy.is_shutdown():
            if self.waypointIndex == shape:
                self.waypointIndex = 1

            if self.isReadyToFly:

                des_x = self.loc[self.waypointIndex][0]
                des_y = self.loc[self.waypointIndex][1]
                des_z = self.loc[self.waypointIndex][2]
                self.des_pose.pose.position.x = des_x
                self.des_pose.pose.position.y = des_y
                self.des_pose.pose.position.z = des_z
                self.des_pose.pose.orientation.x = self.loc[self.waypointIndex][3]
                self.des_pose.pose.orientation.y = self.loc[self.waypointIndex][4]
                self.des_pose.pose.orientation.z = self.loc[self.waypointIndex][5]
                self.des_pose.pose.orientation.w = self.loc[self.waypointIndex][6]

                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z

                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) +
                                 (curr_z - des_z)*(curr_z - des_z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1

            pose_pub.publish(self.des_pose)

            rate.sleep()

    def hover(self):
        """ hover at height mentioned in location
            set mode as HOVER to make it work
        """
        location = self.hover_loc
        loc = [location,
               location,
               location,
               location,
               location,
               location,
               location,
               location,
               location]

        rate = rospy.Rate(10)
        rate.sleep()
        shape = len(loc)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        des_pose = self.copy_pose(self.curr_pose)
        waypoint_index = 0
        sim_ctr = 1
        print self.mode

        while self.mode == "HOVER" and not rospy.is_shutdown():
            if waypoint_index == shape:
                waypoint_index = 0
                sim_ctr += 1
                print "HOVER STOP COUNTER: " + str(sim_ctr)
            if self.isReadyToFly:

                des_x = loc[waypoint_index][0]
                des_y = loc[waypoint_index][1]
                des_z = loc[waypoint_index][2]
                des_pose.pose.position.x = des_x
                des_pose.pose.position.y = des_y
                des_pose.pose.position.z = des_z
                des_pose.pose.orientation.x = loc[waypoint_index][3]
                des_pose.pose.orientation.y = loc[waypoint_index][4]
                des_pose.pose.orientation.z = loc[waypoint_index][5]
                des_pose.pose.orientation.w = loc[waypoint_index][6]

                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z

                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) +
                                 (curr_z - des_z)*(curr_z - des_z))
                if dist < self.distThreshold:
                    waypoint_index += 1

            if sim_ctr == 50:
                pass
                # self.pub.publish("HOVER COMPLETE")  # FIX  UNCOMMENT AND REMOVE pass

            pose_pub.publish(des_pose)
            rate.sleep()

    def scan(self, rect_y, offset_x):
        move = ""
        rate = rospy.Rate(10)
        if self.waypointIndex % 4 == 1:
            move = "back"
        else:
            if ((self.waypointIndex + (self.waypointIndex % 4)) / 4) % 2 == 0:  # HACK
                move = "right"
            else:
                move = "left"
        print self.mode
        loc = self.curr_pose.pose.position
        print loc
        print rect_y
        print offset_x
        while self.mode == "SCAN" and not rospy.is_shutdown():
            if move == "left":
                self.vel_pub.publish(self.get_descent(0, 0.5, 0.1))
                if abs(self.curr_pose.pose.position.y - loc.y) > rect_y/3:
                    self.pub.publish("SCAN COMPLETE")
            elif move == "right":
                self.vel_pub.publish(self.get_descent(0, -0.5, 0.1))
                if abs(self.curr_pose.pose.position.y  - loc.y) > rect_y/3:
                    self.pub.publish("SCAN COMPLETE")
            elif move == "back":
                self.vel_pub.publish(self.get_descent(-0.3, 0, 0.1))
                if abs(self.curr_pose.pose.position.x - loc.x) > offset_x:
                    self.pub.publish("SCAN COMPLETE")
            else:
                print "move error"
            print
            print abs(self.curr_pose.pose.position.y - loc.y)
            print abs(self.curr_pose.pose.position.x - loc.x)
            rate.sleep()

    def descent(self):
        rate = rospy.Rate(10)
        print self.mode
        self.x_change = 1
        self.y_change = 1
        self.x_prev_error = 0
        self.y_prev_error = 0
        self.x_sum_error = 0
        self.y_sum_error = 0
        timeout = 30

        while self.mode == "DESCENT" and not rospy.is_shutdown():
            err_x = 0 - self.tag_pt.x
            err_y = 0 - self.tag_pt.y

            self.x_change += err_x * self.KP  + (self.x_prev_error * self.KD) + (self.x_sum_error * self.KI)
            self.y_change += err_y * self.KP  + (self.y_prev_error * self.KD) + (self.y_sum_error * self.KI)

            self.x_change = max(min(0.4, self.x_change), -0.4)
            self.y_change = max(min(0.4, self.y_change), -0.4)

            if err_x > 0 and err_y < 0:
                des = self.get_descent(-1 * self.x_change, -1 * self.y_change, -0.08)
            elif err_x < 0 and err_y > 0:
                des = self.get_descent(-1 * self.x_change, -1 * self.y_change, -0.08)
            else:
                des = self.get_descent(self.x_change, self.y_change, -0.08)

            self.vel_pub.publish(des)
            timeout -= 1
            rate.sleep()
            self.x_prev_error = err_x
            self.y_prev_error = err_y
            self.x_sum_error += err_x
            self.y_sum_error += err_y


            if timeout == 0 and self.curr_pose.pose.position.z > 0.7:
                timeout = 30
                print timeout
                self.x_change = 0
                self.y_change = 0
                self.x_sum_error = 0
                self.y_sum_error = 0
                self.x_prev_error = 0
                self.y_prev_error = 0

            if self.curr_pose.pose.position.z < 0.2:  # TODO
                # self.mode = "HOVER" # PICK UP
                # self.hover_loc = [self.curr_pose.pose.position.x]  # TODO
                self.pub.publish("PICKUP COMPLETE")


    def rt_survey(self):
        location = [self.saved_location.pose.position.x,
                    self.saved_location.pose.position.y,
                    self.saved_location.pose.position.z,
                    0, 0, 0, 0]
        loc = [location,
               location,
               location,
               location,
               location]

        rate = rospy.Rate(10)
        rate.sleep()
        shape = len(loc)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        des_pose = self.copy_pose(self.curr_pose)
        waypoint_index = 0
        sim_ctr = 1
        print self.mode

        while self.mode == "RT_SURVEY" and not rospy.is_shutdown():
            if waypoint_index == shape:
                waypoint_index = 0
                sim_ctr += 1
            if self.isReadyToFly:

                des_x = loc[waypoint_index][0]
                des_y = loc[waypoint_index][1]
                des_z = loc[waypoint_index][2]
                des_pose.pose.position.x = des_x
                des_pose.pose.position.y = des_y
                des_pose.pose.position.z = des_z
                des_pose.pose.orientation.x = loc[waypoint_index][3]
                des_pose.pose.orientation.y = loc[waypoint_index][4]
                des_pose.pose.orientation.z = loc[waypoint_index][5]
                des_pose.pose.orientation.w = loc[waypoint_index][6]

                curr_x = self.curr_pose.pose.position.x
                curr_y = self.curr_pose.pose.position.y
                curr_z = self.curr_pose.pose.position.z

                dist = math.sqrt((curr_x - des_x)*(curr_x - des_x) + (curr_y - des_y)*(curr_y - des_y) +
                                 (curr_z - des_z)*(curr_z - des_z))
                if dist < self.distThreshold:
                    waypoint_index += 1

            if sim_ctr == 5:
                self.pub.publish("RTS COMPLETE")
                self.saved_location = None

            pose_pub.publish(des_pose)
            rate.sleep()

    def controller(self):
        while not rospy.is_shutdown():

            if self.mode == "SURVEY":
                self.lawnmover(200, 20, 7, 10, 3)

            if self.mode == "HOVER":
                self.hover()

            if self.mode == "SCAN":
                self.scan(20, 3) # pass x_offset, length of rectangle, to bound during small search

            if self.mode == "TEST":
                print self.mode
                self.vel_pub.publish(self.get_descent(0, 0.1, 0))

            if self.mode == "DESCENT":
                self.descent()

            if self.mode == "RT_SURVEY":
                self.rt_survey()

    def planner(self, msg):
        if msg.data == "FOUND UAV" and self.mode == "SURVEY":
            self.saved_location = self.curr_pose
            self.mode = "SCAN"

        if msg.data == "FOUND UAV" and self.mode == "SCAN":
            self.detection_count += 1
            print self.detection_count
            if self.detection_count > 25:
                self.hover_loc = [self.curr_pose.pose.position.x,
                                  self.curr_pose.pose.position.y,
                                  self.curr_pose.pose.position.z,
                                  0, 0, 0, 0]
                self.mode = "HOVER"
                self.detection_count = 0

        if msg.data == "FOUND UAV" and self.mode == "HOVER":
            print self.detection_count
            self.detection_count += 1
            if self.detection_count > 40:
                self.mode = "DESCENT"
                self.detection_count = 0

        if msg.data == "MISSING UAV" and self.mode == "DESCENT":
            self.missing_count += 1
            if self.missing_count > 80:
                self.mode = "HOVER"
                self.missing_count = 0

        if msg.data == "FOUND UAV" and self.mode == "DESCENT":
            self.missing_count = 0

        if msg.data == "SCAN COMPLETE":
            self.mode = "RT_SURVEY"
            self.detection_count = 0

        if msg.data == "HOVER COMPLETE":
            if self.waypointIndex == 0:  # TODO remove this, this keeps the drone in a loop of search
                self.mode = "SURVEY"
            else:
                self.mode = "RT_SURVEY"
            self.detection_count = 0

        if msg.data == "RTS COMPLETE":
            self.mode = "SURVEY"

        if msg.data == "PICKUP COMPLETE":
            # self.mode = "CONFIRM_PICKUP"

            # go back and hover at takeoff location
            self.hover_loc = [self.loc[0][0], self.loc[0][1], self.loc[0][2], 0, 0, 0, 0]
            self.mode = "HOVER"
            self.loc = []
            self.waypointIndex = 0


if __name__ == "__main__":
    OffbPosCtl()#无人机根据AprilTag自动控制