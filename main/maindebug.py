import sys
sys.path.append('.')
import numpy as np
from moudles.linear import transformer
from moudles.matrix import Mat
from moudles.spot import Spot_correction
from pathfinding3d.core.diagonal_movement import DiagonalMovement
from pathfinding3d.core.grid import Grid
from pathfinding3d.finder.a_star import AStarFinder
from pathfinding25d import AStar3D

#这是全地图版本的寻路算法，起点上方不能有遮挡，用于高空规划
class LowAltitude:
    def __init__(self, startpoint, endpoint,level=15):
        """
        初始化低空飞行规划对象
        
        参数:
        startpoint: 起始点坐标 [经度, 纬度, 高度]
        endpoint: 终点坐标 [经度, 纬度, 高度]
        """
        self.startpoint_gps = startpoint
        self.endpoint_gps = endpoint
        self.level=level
        
        # 将GPS坐标转换为平面坐标
        self.startpoint = transformer.gps2txt(startpoint[0], startpoint[1], startpoint[2])
        self.endpoint = transformer.gps2txt(endpoint[0], endpoint[1], endpoint[2])
        
        # 比较两点之间的距离
        self.distance = np.max([self.endpoint[0] - self.startpoint[0], self.endpoint[1] - self.startpoint[1]])
    
    def _plan_long_distance(self):
        """
        处理长距离路径规划（距离 > 70)
        这里可以添加复杂路径规划算法
        """
        print("使用长距离路径规划算法")
        # 长距离规划可以调用短距离规划作为子部分
        # 这里长距离调用2.5d栅格寻路，舍弃3维寻路
       
        height_data = np.load('grid.npy')
        astar = AStar3D(height_data, min_altitude=10, max_altitude=200)
        startpoint = self.startpoint
        endpoint = self.endpoint
        print("origin:", startpoint)
        print("origin:",endpoint)

        # 使用Spot_correction类修正起点和终点
        spot_corrector = Spot_correction(np.load('grid.npy'))
        startpoint=spot_corrector.find_nearest_free_point_bfs(startpoint)
        endpoint=spot_corrector.find_nearest_free_point_bfs(endpoint)

        print("corrected:",startpoint)
        print("corrected:",endpoint)


        print(f"起点可达: {AStar3D.can_reach(astar, startpoint)}")
        print(f"终点可达: {AStar3D.can_reach(astar, endpoint)}")
        path1 = astar.find_path(startpoint, endpoint)

        return path1

    
    def _plan_short_distance(self,sp,ep):
        """
        处理短距离路径规划（距离 <= 70)
        返回直接路径
        """
        print("使用短距离路径规划算法")
        shift=self.load_map(sp,ep)
        path3d=self.findpath3d(shift)
        ##print(path3d)
        


        




        return (path3d)
    
    def fullplan(self):
        """
        生成完整的飞行路径
        根据距离选择不同的规划算法
        
        返回:
        path: 从起点到终点的路径列表
        """
        if self.distance > 74:
            path = self._plan_long_distance()
            ##计算坐标转回gps坐标输出
            path = transformer.txt2gps(np.array(path))
            return path
        else:
            path = self._plan_short_distance(self.startpoint,self.endpoint)
            ##计算坐标转回gps坐标输出
            path = transformer.txt2gps(np.array(path))
            return path
        
    def load_map(self,sp,ep):
        """
        根据startpoint和endpoint的中点坐标，加载最近的四个地图区块
        采用象限判断法确定需要加载的四个区块
        
        返回:
        list: 包含四个最近区块文件名的列表
        """
        # 计算中点坐标
        mid_x = (sp[0] + ep[0]) / 2
        mid_y = (sp[1] + ep[1]) / 2
        
        # 计算区块索引（x从5开始，y从6开始）
        tile_x = int(mid_x / 70) + 5
        tile_y = int(mid_y / 70) + 6
        
        # 确保索引在有效范围内
        tile_x = max(5, min(31, tile_x))
        tile_y = max(6, min(24, tile_y))
        
        # 计算中点相对于当前区块的相对位置
        block_start_x = (tile_x - 5) * 70
        block_start_y = (tile_y - 6) * 70
        rel_x = mid_x - block_start_x
        rel_y = mid_y - block_start_y
        
        # 确定中点所在象限（将当前区块分为四个小区域）
        # 区块中心点：x=35, y=35
        if rel_x >= 35 and rel_y >= 35:  # 第一象限（右上）
            offsets = [(0, 0), (1, 0), (0, 1), (1, 1)]  # 当前区块、右侧、上侧、右上
        elif rel_x < 35 and rel_y >= 35:  # 第二象限（左上）
            offsets = [(0, 0), (-1, 0), (0, 1), (-1, 1)]  # 当前区块、左侧、上侧、左上
        elif rel_x < 35 and rel_y < 35:  # 第三象限（左下）
            offsets = [(0, 0), (-1, 0), (0, -1), (-1, -1)]  # 当前区块、左侧、下侧、左下
        else:  # 第四象限（右下）
            offsets = [(0, 0), (1, 0), (0, -1), (1, -1)]  # 当前区块、右侧、下侧、右下
        
        # 生成四个区块的文件名
        filenames = []
        for dx, dy in offsets:
            x_idx = tile_x + dx
            y_idx = tile_y + dy
             
            # 确保索引不超出范围
            if 5 <= x_idx <= 31 and 6 <= y_idx <= 24:
                # 格式化索引为三位数带符号
                x_str = f"+{x_idx:03d}"
                y_str = f"+{y_idx:03d}"
                filename = [x_str,y_str]
                filenames.append(filename)
        
        mat = Mat(filenames,self.level)
        mat.load_and_process()
        mat.save_matrix('my_matrix.npy')

        return(mat.x_min,mat.y_min,mat.z_min)

    def findpath3d(self,shift):
        """
        使用A*算法在3D网格中查找路径，处理坐标偏移
        使用类属性中的起点和终点坐标
        
        返回:
        list: 路径坐标列表 (还原偏移后的坐标)
        """
        # 在函数内部使用 self.startpoint 和 self.endpoint
        startpoint = self.startpoint
        endpoint = self.endpoint

        
        # 加载矩阵和偏移量
        matrix = np.load('my_matrix.npy')
        
        # 获取偏移量（假设这些值已存储在类属性中）
        x_min = int(shift[0])
        y_min = int(shift[1])
        z_min = int(shift[2])
        
        # 应用偏移到起点和终点
        start_shifted = (
            startpoint[0] - x_min,
            startpoint[1] - y_min,
            startpoint[2] - z_min
        )
        
        end_shifted = (
            endpoint[0] - x_min,
            endpoint[1] - y_min,
            endpoint[2] - z_min
        )
        
        # 创建网格对象
        grid = Grid(matrix=matrix)
        print("origin:", start_shifted)
        print("origin:",end_shifted)
        # 使用Spot_correction类修正起点和终点
        spot_corrector = Spot_correction(matrix)
        start_shifted=spot_corrector.find_nearest_free_point_bfs(start_shifted)
        end_shifted=spot_corrector.find_nearest_free_point_bfs(end_shifted)

        print("corrected:",start_shifted)
        print("corrected:",end_shifted)
        
        # 设置起点和终点（使用偏移后的坐标）
        start = grid.node(start_shifted[0], start_shifted[1], start_shifted[2])
        end = grid.node(end_shifted[0], end_shifted[1], end_shifted[2])
        

        # 创建A*查找器实例（允许对角线移动）
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        
        # 查找路径
        path, runs = finder.find_path(start, end, grid)
        
        # 将路径转换为坐标元组列表并还原偏移
        path_coords = []
        for p in path:
            coords = p.identifier
            # 还原偏移
            restored_coords = (
                coords[0] + x_min,
                coords[1] + y_min,
                coords[2] + z_min
            )
            path_coords.append(restored_coords)
        
        print("operations:", runs, "path length:", len(path_coords))
        print("path:", path_coords)
        
    
        # 可视化路径（可选）
        return(path_coords)
        




# 使用示例
if __name__ == "__main__":
    # 调用无人机输入开始坐标,结束坐标
    startpoint = [117.120049, 31.835725, 35]
    endpoint = [117.121164, 31.835526, 41]
    
    # 创建低空飞行规划对象
    flight_plan = LowAltitude(startpoint, endpoint)
    print(flight_plan.startpoint)
    print(flight_plan.endpoint)
    
    # 生成完整路径
    path = flight_plan.fullplan()
    
    print("距离:", flight_plan.distance)
    print("startpoint:", flight_plan.startpoint)
    print("endpoint:", flight_plan.endpoint)

    print("path:", path)
 
