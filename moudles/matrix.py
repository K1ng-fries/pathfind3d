import numpy as np
import glob
import os

class Mat:
    def __init__(self, coords,zz, base_path="F:/my pathfinding/Data_txt"):
        """
        初始化Mat类
        
        参数:
        coords: 坐标列表，格式如 [['+020', '+020'], ['+019', '+020'], ...]
        base_path: 数据文件的基础路径
        """
        self.coords = coords
        self.base_path = base_path
        self.zz = zz
        self.x_min = None
        self.y_min = None
        self.z_min = None
        self.matrix = None
        self.points = None
        
    def load_and_process(self):
        """加载数据并处理为矩阵"""
        # 存储所有点的列表
        all_points = []

        for x, y in self.coords:
            # 构建文件模式，使用通配符匹配随机后缀
            pattern = f"Tile_{x}_{y}_L19_*.txt"
            search_path = os.path.join(self.base_path, pattern)
            
            # 查找所有匹配的文件
            matching_files = glob.glob(search_path)
            
            if not matching_files:
                print(f"警告: 未找到匹配 {pattern} 的文件")
                continue
            
            # 加载所有匹配的文件
            for file_path in matching_files:
                try:
                    points_data = np.loadtxt(file_path)
                    all_points.append(points_data)
                    print(f"已加载: {os.path.basename(file_path)}")
                except Exception as e:
                    print(f"错误加载文件 {file_path}: {e}")

        # 将所有点数据合并为一个数组
        if all_points:
            self.points = np.vstack(all_points)
            print(f"总共加载了 {len(all_points)} 个文件，共 {self.points.shape[0]} 个点")
        else:
            self.points = np.array([])
            print("未找到任何有效文件")
            return False

        # 步骤2: 每行x,y,z取整 
        self.points = np.round(self.points).astype(int)

        # 步骤3: 转换为矩阵
        ##we take z_min as ground, which means we usually don't go underneath
        self.x_min = np.min(self.points[:, 0])
        self.y_min = np.min(self.points[:, 1])
        self.z_min = np.min(self.points[:, 2])
        x_max = np.max(self.points[:, 0])
        y_max = np.max(self.points[:, 1])
        z_max = np.max(self.points[:, 2])
        
        self.points[:, 0] -= self.x_min  # Shift x coordinates
        self.points[:, 1] -= self.y_min  # Shift y coordinates
        self.points[:, 2] -= self.z_min  # Shift z coordinates

        #shift level
        if z_max<self.zz:
            z_max = self.zz

        # 计算矩阵尺寸
        dx = x_max - self.x_min + 1
        dy = y_max - self.y_min + 1
        dz = z_max - self.z_min + 1
        ## make sure z is within range


        print(f"X offset (x_min): {self.x_min}")
        print(f"Y offset (y_min): {self.y_min}")
        print(f"Z offset (z_min): {self.z_min}")
        print(f"Matrix dimensions: ({dx}, {dy}, {dz})")

        # 初始化一个全1的3D矩阵
        self.matrix = np.ones((dx, dy, dz), dtype=np.int8)

        # 将取整后的点设置为0（障碍）
        self.matrix[self.points[:, 0], self.points[:, 1], self.points[:, 2]] = 0


        
        return True
    
    def save_matrix(self, filename='matrix.npy'):
        """保存矩阵到文件"""
        if self.matrix is not None:
            np.save(filename, self.matrix)
            print(f"矩阵已保存到 {filename}")
        else:
            print("没有矩阵数据可保存")
    
    def get_matrix_info(self):
        """获取矩阵信息"""
        if self.matrix is not None:
            return {
                'shape': self.matrix.shape,
                'x_min': self.x_min,
                'y_min': self.y_min,
                'z_min': self.z_min,
                'obstacle_count': np.sum(self.matrix == 0)
            }
        else:
            return None

# 使用示例
if __name__ == "__main__":
    # 坐标列表
    coords = [['+020', '+020'],['+019','+020']]
    
    # 创建Mat实例
    mat = Mat(coords,70)
    
    # 加载和处理数据
    if mat.load_and_process():
        # 访问属性
        print(f"x_min: {mat.x_min}")
        print(f"y_min: {mat.y_min}")
        print(f"z_min: {mat.z_min}")
        print(f"Matrix shape: {mat.matrix.shape}")
        
        # 保存矩阵
        mat.save_matrix()
        
        # 获取矩阵信息
        info = mat.get_matrix_info()
        print(f"矩阵信息: {info}")