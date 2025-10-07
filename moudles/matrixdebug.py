import numpy as np
import glob
import os

# 你的坐标列表
coords = [['+020', '+020'], ['+019', '+020'], ['+020', '+021'], ['+019', '+021']]

# 存储所有点的列表
all_points = []

# 基础路径
base_path = "F:/my pathfinding/Data_txt"

for x, y in coords:
    # 构建文件模式，使用通配符匹配随机后缀
    pattern = f"Tile_{x}_{y}_L19_*.txt"
    search_path = os.path.join(base_path, pattern)
    
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
    points = np.vstack(all_points)
    print(f"总共加载了 {len(all_points)} 个文件，共 {points.shape[0]} 个点")
else:
    points = np.array([])
    print("未找到任何有效文件")

# 步骤2: 每行x,y,z取整 
points = np.round(points).astype(int)

# 步骤3: 转换为矩阵
x_min = np.min(points[:, 0])
y_min = np.min(points[:, 1])
z_min = np.min(points[:, 2])
x_max = np.max(points[:, 0])
y_max = np.max(points[:, 1])
z_max = np.max(points[:, 2])
points[:, 0] -= x_min  # Shift x coordinates
points[:, 1] -= y_min  # Shift y coordinates
points[:, 2] -= z_min  # Shift y coordinates

# Print offset values


dx=x_max - x_min + 1
dy=y_max - y_min + 1
dz=z_max - z_min + 1

print(f"X offset (x_min): {x_min}")
print(f"Y offset (y_min): {y_min}")
print(f"Z offset (z_min): {z_min}")

# 初始化一个全1的3D矩阵，大小(71,71,34)

matrix = np.ones((dx, dy, dz), dtype=np.int8)

# 将取整后的点设置为0（障碍）
# 注意：假设所有取整后的坐标在0-70 for x,y 和0-33 for z范围内，如果超出会报错
matrix[points[:, 0], points[:, 1], points[:, 2]] = 0

# 可选：保存矩阵到文件，例如npy格式
np.save('matrix.npy', matrix)