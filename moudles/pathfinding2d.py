import numpy as np
import matplotlib.pyplot as plt
import heapq
from collections import deque

class Findpath2d:
    """
    从三维矩阵中提取指定高度范围并构建二维地图的类
    """
    
    def __init__(self, alt, matrix_path='fullmatrix.npy'):
        """
        初始化 Findpath2d 类
        
        参数:
            alt: 中心高度值，将提取 (alt-1, alt, alt+1) 三层
            matrix_path: 三维矩阵文件路径
        """
        self.alt = alt
        self.matrix_path = matrix_path
        self.full_matrix = None
        self.dx, self.dy, self.dz = 0, 0, 0
        self.layer_matrix = None  # 提取的三层合并后的二维地图
        
        # 加载矩阵并处理
        self.load_matrix()
        self.extract_layers()
    
    def load_matrix(self):
        """加载三维矩阵文件"""
        try:
            self.full_matrix = np.load(self.matrix_path)
            self.dx, self.dy, self.dz = self.full_matrix.shape
            print(f"成功加载矩阵，尺寸: {self.dx}x{self.dy}x{self.dz}")
        except FileNotFoundError:
            print(f"错误: 找不到文件 {self.matrix_path}")
            # 创建一个示例矩阵用于测试
            self.create_sample_matrix()
        except Exception as e:
            print(f"加载矩阵时出错: {e}")
            self.create_sample_matrix()
    
    def create_sample_matrix(self):
        """创建示例矩阵（当无法加载文件时使用）"""
        print("创建示例矩阵...")
        self.dx, self.dy, self.dz = 50, 50, 50
        self.full_matrix = np.ones((self.dx, self.dy, self.dz), dtype=np.int8)
        
        # 添加一些障碍物
        # 在z=10到z=20之间添加一个立方体障碍物
        self.full_matrix[10:20, 10:20, 10:20] = 0
        
        # 添加一些随机障碍物
        np.random.seed(42)  # 固定随机种子以便结果可重现
        obstacle_count = 100
        for _ in range(obstacle_count):
            x = np.random.randint(0, self.dx)
            y = np.random.randint(0, self.dy)
            z = np.random.randint(0, self.dz)
            size = np.random.randint(1, 5)
            self.full_matrix[x:x+size, y:y+size, z:z+size] = 0
    
    def extract_layers(self):
        """提取指定高度范围的三层并合并为二维地图"""
        # 计算要提取的高度范围
        z_min = max(0, self.alt - 1)
        z_max = min(self.dz - 1, self.alt + 1)
        
        print(f"提取高度范围: {z_min} 到 {z_max}")
        
        # 检查高度范围是否有效
        if z_min > z_max:
            print(f"错误: 高度范围无效 ({z_min} > {z_max})")
            self.layer_matrix = np.ones((self.dx, self.dy), dtype=np.int8)
            return
        
        # 提取指定高度范围的三层
        layers = self.full_matrix[:, :, z_min:z_max+1]
        
        # 合并三层：如果任何一层有障碍物(0)，则二维地图中该位置为障碍物
        # 使用最小值操作：如果任何一层是0（障碍），结果就是0
        self.layer_matrix = np.min(layers, axis=2)
        
        print(f"二维地图创建完成，障碍物比例: {np.sum(self.layer_matrix == 0) / (self.dx * self.dy):.2%}")
    
    def get_2d_map(self):
        """返回二维地图"""
        return self.layer_matrix
    
    def find_free_point_near(self, x, y, max_radius=10):
        """
        在二维地图上找到指定点附近的自由点
        
        参数:
            x, y: 原始点坐标
            max_radius: 最大搜索半径
            
        返回:
            最近的自由点坐标，如果找不到则返回None
        """
        # 确保点在网格范围内
        x = max(0, min(self.dx-1, x))
        y = max(0, min(self.dy-1, y))
        
        # 如果原始点就是自由的，直接返回
        if self.layer_matrix[x, y] == 1:
            return (x, y)
        
        # 使用广度优先搜索查找最近的自由点
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),  # 上下左右
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 对角线
        
        queue = deque([(x, y)])
        visited = set([(x, y)])
        
        radius = 0
        while queue and radius <= max_radius:
            next_level = []
            
            for cx, cy in queue:
                for dx, dy in directions:
                    nx, ny = cx + dx, cy + dy
                    
                    # 检查边界
                    if 0 <= nx < self.dx and 0 <= ny < self.dy and (nx, ny) not in visited:
                        if self.layer_matrix[nx, ny] == 1:  # 找到自由点
                            return (nx, ny)
                        
                        next_level.append((nx, ny))
                        visited.add((nx, ny))
            
            queue = next_level
            radius += 1
        
        # 如果找不到自由点
        print(f"警告: 在半径 {max_radius} 内找不到自由点")
        return None
    
    def heuristic(self, a, b):
        """A*算法的启发式函数（使用欧几里得距离）"""
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
    
    def aster2d(self, stp, edp):
        """
        二维A*寻路算法
        
        参数:
            stp: 起点坐标 (x, y)
            edp: 终点坐标 (x, y)
            
        返回:
            path: 路径列表 [[x,y], [x,y], ...]
        """
        # 确保起点和终点在自由空间
        valid_start = self.find_free_point_near(stp[0], stp[1])
        valid_end = self.find_free_point_near(edp[0], edp[1])
        
        if valid_start is None or valid_end is None:
            print("错误: 无法找到有效的起点或终点")
            return []
        
        print(f"寻路从 {valid_start} 到 {valid_end}")
        
        # 如果起点和终点相同
        if valid_start == valid_end:
            return [valid_start]
        
        # 定义8个移动方向（包括对角线）
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 上下左右
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # 对角线
        ]
        
        # 对角线移动成本稍高
        diagonal_cost = 1.414  # √2
        
        # 初始化开放列表和关闭列表
        open_set = []
        heapq.heappush(open_set, (0, valid_start))
        
        # 记录每个节点的来源（用于回溯路径）
        came_from = {}
        
        # 记录从起点到每个节点的实际代价
        g_score = {valid_start: 0}
        
        # 记录每个节点的预估总代价（g_score + 启发式值）
        f_score = {valid_start: self.heuristic(valid_start, valid_end)}
        
        while open_set:
            # 获取当前最小f值的节点
            current_f, current = heapq.heappop(open_set)
            
            # 如果到达终点
            if current == valid_end:
                # 回溯路径
                path = []
                while current in came_from:
                    path.append([current[0], current[1]])
                    current = came_from[current]
                path.append([valid_start[0], valid_start[1]])
                path.reverse()
                return path
            
            # 检查所有可能的移动方向
            for i, (dx, dy) in enumerate(directions):
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 检查边界
                if not (0 <= neighbor[0] < self.dx and 0 <= neighbor[1] < self.dy):
                    continue
                
                # 检查是否为障碍物
                if self.layer_matrix[neighbor[0], neighbor[1]] == 0:
                    continue
                
                # 计算移动成本（对角线成本更高）
                cost = 1.0 if i < 4 else diagonal_cost
                tentative_g_score = g_score[current] + cost
                
                # 如果找到更短的路径到该邻居节点
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # 更新路径信息
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, valid_end)
                    
                    # 如果邻居节点不在开放列表中，则添加
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # 如果开放列表为空但未找到路径
        print("警告: 未找到路径")
        return []
    
    def visualize_path(self, path, stp, edp, title=None):
        """可视化路径"""
        if title is None:
            title = f"高度 {self.alt} 的路径规划"
        
        plt.figure(figsize=(10, 8))
        
        # 显示地图
        plt.imshow(self.layer_matrix.T, cmap='gray_r', origin='lower', alpha=0.7)
        
        # 绘制路径
        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            plt.plot(path_x, path_y, 'b-', linewidth=2, label='路径')
            plt.plot(path_x, path_y, 'bo', markersize=3)
        
        # 标记起点和终点
        plt.plot(stp[0], stp[1], 'go', markersize=8, label='起点')
        plt.plot(edp[0], edp[1], 'ro', markersize=8, label='终点')
        
        plt.colorbar(label='0=障碍物, 1=自由空间')
        plt.xlabel('X轴')
        plt.ylabel('Y轴')
        plt.title(title)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    
    def visualize(self, title=None):
        """可视化二维地图"""
        if title is None:
            title = f"高度 {self.alt} 附近的二维地图"
        
        plt.figure(figsize=(10, 8))
        plt.imshow(self.layer_matrix.T, cmap='gray_r', origin='lower')
        plt.colorbar(label='0=障碍物, 1=自由空间')
        plt.xlabel('X轴')
        plt.ylabel('Y轴')
        plt.title(title)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    
    def save_2d_map(self, filename=None):
        """保存二维地图到文件"""
        if filename is None:
            filename = f"2d_map_alt_{self.alt}.npy"
        
        np.save(filename, self.layer_matrix)
        print(f"二维地图已保存到 {filename}")


# 使用示例和测试代码
if __name__ == "__main__":
    # 创建 Findpath2d 对象，指定高度为 25
    finder = Findpath2d(alt=25)
    
    # 获取二维地图
    map_2d = finder.get_2d_map()
    print(f"二维地图尺寸: {map_2d.shape}")
    
    # 测试A*寻路
    start_point = (5, 5)
    end_point = (1145, 1145)
    
    path = finder.aster2d(start_point, end_point)
    
    if path:
        print(f"找到路径，包含 {len(path)} 个点")
        print(f"路径前5个点: {path[:5]}")
        print(f"路径后5个点: {path[-5:]}")
        
        # 可视化路径
        finder.visualize_path(path, start_point, end_point)
    else:
        print("未找到路径")
    
    # 保存二维地图
    finder.save_2d_map()