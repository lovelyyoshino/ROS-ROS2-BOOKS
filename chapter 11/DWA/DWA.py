# -*- coding: utf-8 -*-
"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


def dwa_control(x, config, goal, ob):
    """
    动态窗方法控制
    """
    dw = calc_dynamic_window(x, config)  # 根据当前状态x计算动态窗口

    u, trajectory = calc_control_and_trajectory(
        x, dw, config, goal, ob)  # 计算最优控制量和轨迹

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    仿真参数类
    """

    def __init__(self):
        # 机器人的参数
        self.max_speed = 1.0  # 最大速度 [m/s]
        self.min_speed = -0.5  # 最小速度[m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # 最大旋转速度[rad/s]
        self.max_accel = 0.2  # 最大加速度 [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # 最大旋转加速度 [rad/ss]
        self.v_resolution = 0.01  # 速度分辨率[m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # 旋转角速度分辨率 [rad/s]
        self.dt = 0.1  # [s] 时间差
        self.predict_time = 3.0  # 预测的时间长度 [s]
        self.to_goal_cost_gain = 0.15  # 预测时间内到达目标点的成本倍率
        self.speed_cost_gain = 1.0  # 预测时间内速度的成本倍率
        self.obstacle_cost_gain = 1.0   # 预测时间内障碍物的成本倍率
        self.robot_stuck_flag_cons = 0.001  # 机器人阻塞的阈值
        self.robot_type = RobotType.circle  # 机器人的形状

        # if robot_type == RobotType.circle
        # 也用于检查目标是否为原型或者方形这两种
        self.robot_radius = 1.0  # 机器人碰撞检测 [m]

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # 机器人碰撞检测 [m]
        self.robot_length = 1.2  # 机器人碰撞检测 [m]
        # 障碍物设置
        self.ob = np.array([[-1, -1],
                            [0, 2],
                            [4.0, 2.0],
                            [5.0, 4.0],
                            [5.0, 5.0],
                            [5.0, 6.0],
                            [5.0, 9.0],
                            [8.0, 9.0],
                            [7.0, 9.0],
                            [8.0, 10.0],
                            [9.0, 11.0],
                            [12.0, 13.0],
                            [12.0, 12.0],
                            [15.0, 15.0],
                            [13.0, 13.0]
                            ])

    # 机器人类型
    @property
    def robot_type(self):
        return self._robot_type

    # 机器人类型，并判断是否在可选择类型中
    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


# 全局变量，用于存储配置参数
config = Config()


def motion(x, u, dt):
    """
    运动模型
    """

    x[2] += u[1] * dt  # 旋转角度
    x[0] += u[0] * math.cos(x[2]) * dt  # x位置
    x[1] += u[0] * math.sin(x[2]) * dt  # y位置
    x[3] = u[0]  # 速度
    x[4] = u[1]  # 角速度

    return x


def calc_dynamic_window(x, config):
    """
    根据当前状态x计算动态窗口
    """
    # 配置的动态窗口
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # 求当前速度状态的动态窗口
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    # 从而可以求出在当前时刻下的动态窗口
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    用输入预测轨迹
    """
    x = np.array(x_init)  # 当前车辆的状态
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:  # 小于预测时间，则需要不断预测
        x = motion(x, [v, y], config.dt)  # 推算dt下的时间
        trajectory = np.vstack((trajectory, x))  # 拿到轨迹，不同的x的组成的轨迹
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    计算最优控制量和轨迹
    """

    x_init = x[:]
    min_cost = float("inf")  # 初始化最小成本为无穷大
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # 在动态窗口中评估所有采样输入的轨迹,从最大和最小的速度和角速度进行切分
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
            # 对每一个小段预测轨迹，并推算出在接下来predict_time下的轨迹
            trajectory = predict_trajectory(x_init, v, y, config)
            # 计算与目标，速度，障碍物的权重
            to_goal_cost = config.to_goal_cost_gain * \
                calc_to_goal_cost(trajectory, goal)  # 用角度差计算到目标的cost
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * \
                calc_obstacle_cost(trajectory, ob, config)  # 与障碍物的cost计算

            final_cost = to_goal_cost + speed_cost + ob_cost  # 最终的cost

            # 判断代价最小的路径作为下一个选择路径
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]  # 拿到划窗中最优的位置
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:  # 当最优速度或者当前速度小于阻塞值，则旋转以防阻塞
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    计算障碍物的cost
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]  # 将轨迹和障碍物的坐标进行差值
    r = np.hypot(dx, dy)  # 计算轨迹和障碍物的欧几里得距离

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]  # 计算轨迹的角度
        rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                       [np.sin(yaw), np.cos(yaw)]])  # 将角度转换为旋转矩阵
        rot = np.transpose(rot, [2, 0, 1])  # 将旋转矩阵转置
        local_ob = ob[:, None] - trajectory[:, 0:2]  # 将障碍物和轨迹的坐标进行差值
        # 将差值的结果转换为n行shape[-1]列的数组
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])  # 将差值的结果与旋转矩阵相乘
        # 将差值的结果转换为n行shape[-1]列的数组
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        # 从与障碍物的距离计算出障碍物的cost
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():  # 如果障碍物与安全距离重合则直接放弃该路径
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
    用角度差计算到目标的cost
    """

    dx = goal[0] - trajectory[-1, 0]  # 目标点与最后一个点的x差
    dy = goal[1] - trajectory[-1, 1]  # 目标点与最后一个点的y差
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle),
               math.cos(cost_angle)))  # 与目标方向的角度差

    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -
                             config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")


def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")
    # 初始状态 [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # 最终位置 [x(m), y(m)]
    goal = np.array([gx, gy])

    # input [forward speed, yaw_rate]

    # 配置文件，内部包含了机器人的状态
    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = config.ob
    while True:
        u, predicted_trajectory = dwa_control(x, config, goal, ob)  # 动态窗方法控制
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0],
                     predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()


if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)  # 主程序入口，通过RobotType.rectangle完成调用车辆形状
    # main(robot_type=RobotType.circle)
