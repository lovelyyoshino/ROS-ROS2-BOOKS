# -*- coding: utf-8 -*-
"""

Path planning Sample Code with RRT*

author: Atsushi Sakai(@Atsushi_twi)

"""

import math
import os
import sys

import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../RRT/")

try:
    from RRT import RRT
except ImportError:
    raise

show_animation = True


class RRTStar(RRT):  # 这里作为子类继承了RRT类
    """
    RRT Star规划类
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0  # 加入了代价信息

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=30.0,
                 path_resolution=1.0,
                 goal_sample_rate=20,
                 max_iter=300,
                 connect_circle_dist=50.0,
                 search_until_max_iter=False):
        """
        设置参数
        启动:启动位置(x, y)
        目标:目标位置(x, y)
        obstacleList:障碍位置[[x, y,大小]…]
        randArea:随机采样区域[min,max] 
        """
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter)
        self.connect_circle_dist = connect_circle_dist  # 链接圆距离
        self.goal_node = self.Node(goal[0], goal[1])  # 目标节点
        self.search_until_max_iter = search_until_max_iter  # 一直搜索直到最大

    def planning(self, animation=True):
        """
        rrt star 路径规划

        animation: 开启动画标志
        """
        self.node_list = [self.start]  # 起点存放到node_list当中
        for i in range(self.max_iter):  # 最大的迭代次数
            print("Iter:", i, ", number of nodes:", len(
                self.node_list))  # 迭代次数以及对应的节点列表打印
            rnd = self.get_random_node()  # 随机获得新的node信息
            nearest_ind = self.get_nearest_node_index(
                self.node_list, rnd)  # 获取两个节点最近的节点
            new_node = self.steer(self.node_list[nearest_ind], rnd,
                                  self.expand_dis)  # 通过该函数完成了下一个节点的选取
            near_node = self.node_list[nearest_ind]  # 拿到最近的结果
            new_node.cost = near_node.cost + \
                math.hypot(new_node.x-near_node.x,
                           new_node.y-near_node.y)  # 根据之前的代价与移动距离计算出当前代价

            if self.check_collision(new_node, self.obstacle_list):  # 检查是否发生碰撞
                near_inds = self.find_near_nodes(new_node)  # 根据新拿到的点来搜索近处的节点
                node_with_updated_parent = self.choose_parent(
                    new_node, near_inds)  # 选择最优的节点
                if node_with_updated_parent:  # 如果找到了最近的节点
                    # 更新对应的node节点以及对应的父节点的cost
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(
                        node_with_updated_parent)  # 将该节点加入到节点列表中
                else:
                    self.node_list.append(new_node)  # 将该节点加入到节点列表中

            if animation:
                self.draw_graph(rnd)

            if ((not self.search_until_max_iter)
                    and new_node):  # 如果达到目标
                last_index = self.search_best_goal_node()  # 根据这些节点重新寻找最优的节点
                if last_index is not None:
                    # 生成从终点到起点的路径
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)
        return None

    def choose_parent(self, new_node, near_inds):
        """
        计算near_inds列表中指向new_node的代价最低的点，并将该节点设置为new_node的父节点。 
        """
        if not near_inds:  # 如果从find_near_nodes函数里面拿不到索引
            return None

        # 在near_inds中搜索最近的成本
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]  # 获得的附近的node节点
            t_node = self.steer(near_node, new_node)  # 通过该函数完成了下一个节点的选取
            # 检查是否发生碰撞
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(
                    near_node, new_node))  # 计算出新的代价信息
            else:
                costs.append(float("inf"))  # 存在障碍物代价就是无穷大
        min_cost = min(costs)  # 获得最小的cost值

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]  # 根据cost计算得到near_node的索引
        new_node = self.steer(
            self.node_list[min_ind], new_node)  # 通过该函数完成了下一个节点的选取
        new_node.cost = min_cost

        return new_node

    # 根据这些节点重新寻找最优的节点
    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]  # 从节点列表中计算出距离到目标点的距离
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]  # 根据距离到目标点的距离来获取到目标点的索引的node节点

        safe_goal_inds = []
        for goal_ind in goal_inds:  # 拿出所有符合条件的node节点
            # 计算出与目标的下一个节点的选取
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list):  # 检查是否发生碰撞
                # 将可以安全到达目标的node节点存放到safe_goal_inds中
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min(
            [self.node_list[i].cost for i in safe_goal_inds])  # 计算出最小的代价
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:  # 返回最小的代价的索引
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) 定义一个以new_node为中心的圆
        2) 返回在这个球内中的所有节点
        """
        nnode = len(self.node_list) + 1  # 将当前的node_list加一
        r = self.connect_circle_dist * \
            math.sqrt((math.log(nnode) / nnode))  # 计算出该节点的有效半径
        # 如果expand_dist存在，搜索的顶点范围不超过expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)  # 确保不会大于expand_dist
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]  # 计算出原来的节点与传入的new_node之间的距离
        # 并计算出在r范围内的节点索引
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
        对于near_inds中的每个节点，这将检查从new_node到达它们是否代价更低。
        在这种情况下，这将把near_inds中节点的父节点重新分配给new_node。
        """
        for i in near_inds:
            near_node = self.node_list[i]  # 对每一个near_node取出节点，并判断出节点是否需要更新
            edge_node = self.steer(new_node, near_node)  # 通过该函数完成了下一个节点的选取
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(
                new_node, near_node)  # 计算新的代价函数

            no_collision = self.check_collision(
                edge_node, self.obstacle_list)  # 计算是否发生碰撞
            improved_cost = near_node.cost > edge_node.cost  # 检查当前的cost是否小于之前的cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)  # 将父节点的参数进行更新
    # 计算出新的代价信息

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    # 将父节点的参数进行更新
    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)  # 进行迭代


def main():
    print("Start " + __file__)

    # ====Search Path with RRT====
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
    ]  # [x,y,size(radius)]

    # 设置初始参数
    rrt_star = RRTStar(
        start=[0, 0],
        goal=[6, 10],  # 目标点
        rand_area=[-2, 15],  # 从-2到15取运动位置
        obstacle_list=obstacle_list,
        expand_dis=1)
    path = rrt_star.planning(animation=show_animation)  # 使用RRT Star搜索路径

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
