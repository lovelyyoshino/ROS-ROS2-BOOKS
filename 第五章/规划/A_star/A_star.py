import math

import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize map for a star planning
        ox: 障碍物x方向上的位置 [m]
        oy: 障碍物y方向上的位置 [m]
        resolution: 栅格分辨率 [m]
        rr: 机器人半径[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:# 封装的Node类
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # 栅格地图的索引
            self.y = y  # 栅格地图的索引
            self.cost = cost # 需要的代价
            self.parent_index = parent_index  # 上一个节点的索引

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star 路径搜索算法
        input:
            s_x: x的起始位置 [m]
            s_y: y的起始位置 [m]
            gx: x的目标位置 [m]
            gx: y的目标位置 [m]
        output:
            rx: 最终路径的x位置列表
            ry: 最终路径的y位置列表
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()# 设置的set集合
        open_set[self.calc_grid_index(start_node)] = start_node # 将索引存入到open_set中

        while 1:
            if len(open_set) == 0: # 判断集合是否为空，如果是空则代表找不到路径
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))#选择出在open_set集合里面最小的cost节点作为下一次的更新的起始点，在A*中需要计算当前点与目标的启发式权重
            current = open_set[c_id]#找到对应的Node节点

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")#绘制所在位置的点
                # 用于使用esc键停止模拟
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)
			# 如果当前的位置为终点位置
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index#拿到父index，并更新到goal_node中
                goal_node.cost = current.cost
                break# 跳出while

            # 从open_set中移除当前的索引，并将该索引存入到closed_set中
            del open_set[c_id]
            closed_set[c_id] = current

            # 基于motion中的运动候选策略扩展搜索网格
            for i, _ in enumerate(self.motion):
            	# 加入新的Node
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                # 计算出索引
                n_id = self.calc_grid_index(node)

                # 是否超出边界
                if not self.verify_node(node):
                    continue
				# 是否在closed_set中
                if n_id in closed_set:
                    continue
				# 如果不在open_set中直接更新Node，如果在，则需要判断cost的大小
                if n_id not in open_set:
                    open_set[n_id] = node  # 发现新节点
                else:
                    if open_set[n_id].cost > node.cost:
                        # 这条路是迄今为止最好的，记录它
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # 生成最后的路径
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]# 计算出当前真实的位置
        parent_index = goal_node.parent_index# 判断是否存在父节点
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry
	# 启发式搜索函数，通过与目标距离计算欧几里得范数
    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # 启发式的重量
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d
    # 根据index和分辨率转化为真实的距离
    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos
    # 计算出在地图中的距离
    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)
    # 计算出该点在栅格地图Node中的信息
    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)
    # 判断是否为障碍物或者超出边界
    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

	# 对整个数组获取最小值，作为边界
    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)
        #获得在地图中障碍物的长宽
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # 障碍物地图信息，一开始设置为False
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                # 计算出地图中对应真实场景中的x,y距离
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)# 返回欧几里得范数
                    if d <= self.rr:# 通过机器人半径设置障碍物检测
                        self.obstacle_map[ix][iy] = True#将存在有障碍物的设置为True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost，对应了机器人可以移动的方向
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # 定义起始点，终止点，栅格大小以及机器人大小
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # 设置障碍物的位置信息
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)#初始化信息
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()