import numpy as np
from pyproj import Transformer

class CoordinateTransformer:
    def __init__(self):
        # 定义坐标系转换器：EPSG:4548 -> EPSG:4326
        self.transformer_4548_to_4326 = Transformer.from_crs("EPSG:4548", "EPSG:4326", always_xy=True)
        # 定义坐标系转换器：EPSG:4326 -> EPSG:4548
        self.transformer_4326_to_4548 = Transformer.from_crs("EPSG:4326", "EPSG:4548", always_xy=True)
        
        # 定义偏移量
        self.x_offset = -511150 + 786
        self.y_offset = -3522686 + 43
    
    def txt2gps(self, points):
        """
        将txt坐标转换为GPS坐标（EPSG:4326）
        
        参数:
        points: 单个点 [x, y, z] 或点列表 [[x1,y1,z1], [x2,y2,z2], ...]
        
        返回:
        保持输入格式：单个点返回 [lon, lat, alt]，点列表返回 [[lon1,lat1,alt1], [lon2,lat2,alt2], ...]
        """
        # 转换为numpy数组以便处理
        points = np.array(points)
        
        # 检查输入维度
        if points.ndim == 1 and points.shape[0] == 3:
            # 单个点 [x, y, z]
            x, y, z = points
            is_single = True
        elif points.ndim == 2 and points.shape[1] == 3:
            # 点列表 [[x1,y1,z1], [x2,y2,z2], ...]
            x, y, z = points[:, 0], points[:, 1], points[:, 2]
            is_single = False
        else:
            raise ValueError("输入必须是 [x,y,z] 或 [[x1,y1,z1], [x2,y2,z2], ...]")
        
        # 应用偏移
        x_shifted = x - self.x_offset
        y_shifted = y - self.y_offset
        
        # 转换为WGS84
        lon, lat = self.transformer_4548_to_4326.transform(x_shifted, y_shifted)
        
        # 保持输入格式返回
        if is_single:
            return [lon, lat, z]
        else:
            return np.column_stack([lon, lat, z]).tolist()
        
    def gps2txt(self, lon, lat, alt):
        """
        将GPS坐标（EPSG:4326）转换为txt坐标
        
        参数:
        lon, lat, alt: 经度、纬度和高度，可以是单个值或数组
        
        返回:
        x, y, z: 转换后的txt坐标
        """
        # 转换为EPSG:4548坐标
        if np.isscalar(lon):
            # 处理单个值
            x_global, y_global = self.transformer_4326_to_4548.transform(lon, lat)
            # 应用反向偏移
            x = int(round(x_global + self.x_offset))
            y = int(round(y_global + self.y_offset))
            return x, y, alt
        else:
            # 处理数组
            x_global, y_global = self.transformer_4326_to_4548.transform(lon, lat)
            # 应用反向偏移
            x = np.round( x_global + self.x_offset ).astype(int)
            y = np.round( y_global + self.y_offset ).astype(int)
            return x, y, alt

# 创建全局转换器实例
transformer = CoordinateTransformer()

# 提供简便的函数接口
def txt2gps(point):
    """简便函数：将txt坐标转换为GPS坐标"""
    return transformer.txt2gps(point)

def gps2txt(lon, lat, alt):
    """简便函数：将GPS坐标转换为txt坐标"""
    return transformer.gps2txt(lon, lat, alt)

# 示例用法
if __name__ == "__main__":
    # 示例：单个点转换
    x, y, z = 1070, 984, 36
    lon, lat, alt = txt2gps([x, y, z])
    print(f"txt坐标 {[x,y,z]} -> GPS坐标 {[lon,lat,alt]}")
    x, y, z = 1106, 978, 41
    lon, lat, alt = txt2gps([x, y, z])
    print(f"txt坐标 {[x,y,z]} -> GPS坐标 {[lon,lat,alt]}")

    
    # 示例：批量转换
    points = np.array([
        [1000, 1000, 100],
        [1100, 1100, 110],
        [1200, 1200, 120]
    ])
    
    # 正向转换

    
  
""""
这个 linear.py 文件提供了：

CoordinateTransformer 类：包含完整的坐标转换功能

全局转换器实例 transformer：可以直接使用

简便函数 txt2gps() 和 gps2txt()：可以直接导入使用
"""