import numpy as np

# 加载原始网格数据
grid_data = np.load('grid.npy')

# 交换行和列坐标（转置矩阵）
grid_transposed = grid_data.T

# 保存修改后的网格数据（覆盖原文件或保存为新文件）
np.save('grid.npy', grid_transposed)

print(f"原始网格形状: {grid_data.shape}")
print(f"转置后网格形状: {grid_transposed.shape}")
print("网格坐标已修正：行和列已交换")