"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRT:
    """
    RRT规划类
    """

    class Node:
        """
        RRT节点存放了父类，路径这些问题
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500):
        """
        设置参数
        启动:启动位置(x, y)
        目标:目标位置(x, y)
        obstacleList:障碍位置[[x, y,大小]…]
        randArea:随机采样区域[min,max] 
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis# 扩展距离
        self.path_resolution = path_resolution# 路径的分辨率
        self.goal_sample_rate = goal_sample_rate# 目标的采样概率
        self.max_iter = max_iter# 搜索的最大迭代次数
        self.obstacle_list = obstacle_list# 障碍物列表，用于建立障碍物
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt 路径规划

        animation: 开启动画标志
        """

        self.node_list = [self.start]# 起点存放到node_list当中
        for i in range(self.max_iter):# 最大的迭代次数
            rnd_node = self.get_random_node()# 随机获得新的node信息
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)# 获取两个节点最近的节点
            nearest_node = self.node_list[nearest_ind]# 拿到最近的结果

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)# 通过该函数完成了下一个节点的选取

            if self.check_collision(new_node, self.obstacle_list):#检查在路径上是否存在碰撞
                self.node_list.append(new_node)#将该节点加入到节点列表中

            if animation and i % 5 == 0:# 每五次绘制一次
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:# 计算与目标的距离是否小于一个步长
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)# 以目标点进行搜索
                if self.check_collision(final_node, self.obstacle_list):# 检查是否发生碰撞
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    # 通过该函数完成了下一个节点的选取
    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)# 拿到当前的节点信息
        d, theta = self.calc_distance_and_angle(new_node, to_node)#计算出当前节点和目标节点的距离

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:# 计算距离是否小于阈值，小于则就运动到目标点
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)#按照分辨率计算

        for _ in range(n_expand):# 按照分辨率添加到列表中
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)#计算出new_node节点和to_node目标节点的距离
        if d <= self.path_resolution:#如果小于分辨率,则认为找到
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y
        new_node.parent = from_node# 设置父节点

        return new_node
    # 生成从终点到起点的路径
    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path
    # 计算与目标的距离
    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)
    # 获得随机的node节点
    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:#随机采样
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))#计算出前进的情况
        else:  # 目标点取样
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    #绘制图表
    def draw_graph(self, rnd=None):
        plt.clf()
        # 用esc键停止模拟
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        # 如果仍然可以找到rnd_node
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        # 从已开放的节点中查询父节点并绘制
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        # 绘制障碍物
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)
    # 绘制障碍物圆
    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)
    # 计算出当前存在的节点列与求出最新node的距离，并返回最近的index信息
    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind
    #检查在路径上是否存在碰撞
    @staticmethod
    def check_collision(node, obstacleList):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:# 障碍物与path_x，path_y计算出距离，并检测是否碰撞
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size**2:
                return False  # 发生碰撞

        return True  # 安全

    # 计算当前节点和目标节点的距离和角度
    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    # 障碍物的位姿,以圆的形式表示
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    # 设置初始参数
    rrt = RRT(
        start=[0, 0],
        goal=[gx, gy],#目标点
        rand_area=[-2, 15],# 从-2到15取运动位置
        obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)#使用RRT搜索路径

    if path is None:#如果找不到最终返回的路径
        print("Cannot find path")
    else:
        print("found path!!")

        # 绘制最终的轨迹
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()