import math
import heapq

class AStar3D:
    nodes_explored = 0
    def __init__(self, height_map, min_altitude=5, max_altitude=100):
        self.height_map = height_map
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.rows, self.cols = height_map.shape
        


    def is_valid_position(self, x, y, z):
        if not (0 <= x < self.cols and 0 <= y < self.rows):
            return False
        ground_height = self.height_map[y, x]
        return ground_height + self.min_altitude <= z <= self.max_altitude

    def get_neighbors(self, x, y, z):
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    nx, ny, nz = x + dx, y + dy, z + dz
                    if self.is_valid_position(nx, ny, nz):
                        neighbors.append((nx, ny, nz))
        return neighbors

    def calculate_cost(self, current, neighbor):
        # current和neighbor都是(x,y,z)元组
        dx = abs(current[0] - neighbor[0])
        dy = abs(current[1] - neighbor[1])
        dz = abs(current[2] - neighbor[2])
        # 预计算的代价表
        cost_map = {
            (1, 0, 0): 1.0,
            (0, 1, 0): 1.0,
            (0, 0, 1): 1.0,
            (1, 1, 0): 1.414,
            (1, 0, 1): 1.414,
            (0, 1, 1): 1.414,
            (1, 1, 1): 1.732
        }
        return cost_map.get((dx, dy, dz), math.sqrt(dx*dx + dy*dy + dz*dz))

    def heuristic(self, node, goal):
        """推荐的启发函数优化版本"""
        dx = abs(node[0] - goal[0])
        dy = abs(node[1] - goal[1])
        dz = abs(node[2] - goal[2])
    
        # 对三个轴差值排序
        a, b, c = sorted([dx, dy, dz], reverse=True)
        
        # 对角线距离近似 - 精度好且计算快
        return a + 0.414 * b + 0.318 * c

    def find_path(self, start, end):
        
        start_node = (start[0], start[1], start[2])
        goal_node = (end[0], end[1], end[2])

        open_set = []
        heapq.heappush(open_set, (0, start_node))

        g_score = {start_node: 0}
        parent = {start_node: None}
        
        while open_set:
            current_f, current = heapq.heappop(open_set)

            if current == goal_node:
                return self.reconstruct_path(parent, current)

            for neighbor in self.get_neighbors(*current):
                tentative_g = g_score[current] + self.calculate_cost(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    parent[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_node)
                    heapq.heappush(open_set, (f_score, neighbor))
                    self.nodes_explored += 1
                    if self.nodes_explored % 10000 == 0:
                        print(f"已探索节点: {self.nodes_explored}, 开放队列大小: {len(open_set)}")
        

        
        return []  # 无路径

    def find_path_bidirectional(self, start, end):
        """双向 A* 搜索。保持其他方法不变。
        返回与 find_path 相同格式的路径：[(x,y,z), ...] 或 []。
        """
        start_node = (start[0], start[1], start[2])
        goal_node = (end[0], end[1], end[2])

        if start_node == goal_node:
            return [start_node]

        # 双向开放集（heap），g 值，父指针，已关闭集合
        open_f = []
        open_b = []
        heapq.heappush(open_f, (0, start_node))
        heapq.heappush(open_b, (0, goal_node))

        g_f = {start_node: 0}
        g_b = {goal_node: 0}
        parent_f = {start_node: None}
        parent_b = {goal_node: None}

        closed_f = set()
        closed_b = set()

        # Helper to reconstruct joined path when meeting at `meet`
        def _reconstruct(meet):
            path_f = self.reconstruct_path(parent_f, meet)  # start -> meet
            path_b_goal_to_meet = self.reconstruct_path(parent_b, meet)  # goal -> meet
            # reconstruct_path(parent_b, meet) 返回的是从 goal 到 meet 的路径，需反转为 meet -> goal
            path_meet_to_goal = list(reversed(path_b_goal_to_meet))
            # 合并（去重 meet）
            return path_f + path_meet_to_goal[1:]

        # 主循环：每次扩展两个方向中 f 值较小的一侧
        while open_f and open_b:
            # 看看两边当前的最小 f 来决定扩展哪一边
            f_f = open_f[0][0] if open_f else float('inf')
            f_b = open_b[0][0] if open_b else float('inf')

            # 选择要扩展的方向
            expand_forward = f_f <= f_b

            if expand_forward:
                _, current = heapq.heappop(open_f)
                if current in closed_f:
                    continue
                closed_f.add(current)

                # 如果碰到了反向已访问的节点，构建路径
                if current in closed_b:
                    return _reconstruct(current)

                for neighbor in self.get_neighbors(*current):
                    tentative_g = g_f[current] + self.calculate_cost(current, neighbor)
                    if neighbor not in g_f or tentative_g < g_f[neighbor]:
                        parent_f[neighbor] = current
                        g_f[neighbor] = tentative_g
                        f_score = tentative_g + self.heuristic(neighbor, goal_node)
                        heapq.heappush(open_f, (f_score, neighbor))
                        self.nodes_explored += 1
                        if self.nodes_explored % 10000 == 0:
                            print(f"已探索节点: {self.nodes_explored}, 开放队列大小: {len(open_f)}")

            else:
                _, current = heapq.heappop(open_b)
                if current in closed_b:
                    continue
                closed_b.add(current)

                if current in closed_f:
                    return _reconstruct(current)

                for neighbor in self.get_neighbors(*current):
                    tentative_g = g_b[current] + self.calculate_cost(current, neighbor)
                    if neighbor not in g_b or tentative_g < g_b[neighbor]:
                        parent_b[neighbor] = current
                        g_b[neighbor] = tentative_g
                        f_score = tentative_g + self.heuristic(neighbor, start_node)
                        heapq.heappush(open_b, (f_score, neighbor))
                        self.nodes_explored += 1
                        if self.nodes_explored % 10000 == 0:
                            print(f"已探索节点: {self.nodes_explored}, 开放队列大小: {len(open_b)}")

        # 无路径
        return []

    def reconstruct_path(self, parent, current):
        path = []
        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()
        return path
    
    def can_reach(astar, point):
        x, y, z = point
        return astar.is_valid_position(x, y, z)


    def visualize(self, path=None, start=None, end=None):
        """可视化地图和路径"""
        try:
            import pyvista as pv
            import numpy as np
        except ImportError:
            print("请先安装pyvista: pip install pyvista")
            return
        
        # 创建地形
        x = np.arange(self.height_map.shape[1])
        y = np.arange(self.height_map.shape[0])
        X, Y = np.meshgrid(x, y)
        terrain = pv.StructuredGrid(X, Y, self.height_map)
        
        plotter = pv.Plotter()
        
        # 地形（半透明）
        plotter.add_mesh(terrain, cmap='terrain', show_scalar_bar=True)
        
        # 如果有路径，显示路径
        if path and len(path) > 0:
            path_points = np.array([(p[0], p[1], p[2]) for p in path])
            path_line = pv.lines_from_points(path_points)
            
            plotter.add_mesh(path_line, color='red', line_width=6, label='Flight Path')
            plotter.add_points(path_points, color='yellow', point_size=4, opacity=0.7)
            plotter.add_points(path_points[0], color='green', point_size=20, label='Start')
            plotter.add_points(path_points[-1], color='blue', point_size=20, label='End')
        else:
            # 如果没有路径，只显示起点终点
            if start:
                plotter.add_points([start], color='green', point_size=20, label='Start')
            if end:
                plotter.add_points([end], color='blue', point_size=20, label='End')
        
        plotter.add_legend()
        plotter.add_title("3D Path Planning Visualization")
        plotter.show()
