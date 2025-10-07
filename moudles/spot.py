import numpy as np
from collections import deque

class Spot_correction:
    def __init__(self, matrix, min_altitude=5, max_altitude=100):
        self.matrix = matrix
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        
        # 判断是2D还是3D地图
        if len(matrix.shape) == 3:
            # 3D地图
            self.dx, self.dy, self.dz = matrix.shape
            self.is_3d = True
        else:
            # 2.5D地图 (高度图)
            self.dx, self.dy = matrix.shape
            self.is_3d = False
    
    def is_obstacle(self, point):
        """检查点是否为障碍物"""
        if self.is_3d:
            # 3D地图：直接检查体素值
            x, y, z = point
            return self.matrix[x, y, z] == 0
        else:
            # 2.5D地图：比较z值与高度图
            x, y, z = point
            if not (0 <= x < self.dx and 0 <= y < self.dy):
                return True  # 边界外视为障碍物
            ground_height = self.matrix[y, x]  # 注意坐标顺序
            return z <= ground_height + self.min_altitude or z > self.max_altitude
    
    def find_nearest_free_point_bfs(self, point):
        """使用BFS查找最近的自由点"""
        if not self.is_obstacle(point):
            return point  # 点已经是自由的
            
        # 定义邻居方向
        if self.is_3d:
            # 3D地图：26个方向
            directions = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    for dz in [-1, 0, 1]:
                        if dx == 0 and dy == 0 and dz == 0:
                            continue  # 跳过自身
                        directions.append((dx, dy, dz))
        else:
            # 2.5D地图：8个水平方向 + 垂直方向
            directions = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue  # 跳过自身
                    directions.append((dx, dy, 0))  # 水平移动
            
            # 添加垂直移动方向（限制范围）
            directions.append((0, 0, 1))   # 上升
            directions.append((0, 0, -1))  # 下降
        
        # BFS队列
        queue = deque([point])
        visited = set([point])
        
        while queue:
            current_point = queue.popleft()
            x, y, z = current_point
            
            # 检查所有方向
            for dx, dy, dz in directions:
                nx, ny, nz = x + dx, y + dy, z + dz
                new_point = (nx, ny, nz)
                
                # 检查边界和是否已访问
                if new_point in visited:
                    continue
                    
                if self.is_3d:
                    # 3D边界检查
                    if (0 <= nx < self.dx and 0 <= ny < self.dy and 0 <= nz < self.dz):
                        if not self.is_obstacle(new_point):  # 找到自由点
                            return new_point
                        queue.append(new_point)
                        visited.add(new_point)
                else:
                    # 2.5D边界检查
                    if (0 <= nx < self.dx and 0 <= ny < self.dy and 
                        self.min_altitude <= nz <= self.max_altitude):
                        if not self.is_obstacle(new_point):  # 找到自由点
                            return new_point
                        queue.append(new_point)
                        visited.add(new_point)
        
        # 如果找不到自由点（理论上不应该发生）
        return point