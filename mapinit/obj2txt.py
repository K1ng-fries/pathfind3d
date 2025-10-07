import pymeshlab as ml
import numpy as np
import os

# 指定输入OBJ文件夹路径
input_obj_dir = "H:/3dpointcloud/Data_obj19"

# 指定输出文件夹
output_dir = "H:/3dpointcloud/Data_txt"

# 确保输出文件夹存在
os.makedirs(output_dir, exist_ok=True)

# 获取输入文件夹中的所有OBJ文件
obj_files = [f for f in os.listdir(input_obj_dir) if f.endswith('.obj')]

if not obj_files:
    print(f"在文件夹 {input_obj_dir} 中未找到OBJ文件")
    exit(1)

print(f"找到 {len(obj_files)} 个OBJ文件，开始转换...")

# 处理每个OBJ文件
for obj_file in obj_files:
    # 构建完整的输入文件路径
    input_obj_path = os.path.join(input_obj_dir, obj_file)
    
    # 生成输出TXT文件名（与输入文件同名，仅扩展名不同）
    output_filename = os.path.splitext(obj_file)[0] + '.txt'
    output_txt_path = os.path.join(output_dir, output_filename)
    
    print(f"正在处理: {obj_file}")
    
    # 创建MeshSet并加载OBJ文件
    ms = ml.MeshSet()
    try:
        ms.load_new_mesh(input_obj_path)
        
        # 应用Poisson Disk Sampling过滤器，设置半径为边界框对角线的1%
        p = ml.PercentageValue(1)
        ms.generate_sampling_poisson_disk(radius=p)
        
        # 获取当前网格（采样后的点云）
        m = ms.current_mesh()
        
        # 提取顶点坐标作为numpy数组
        points = m.vertex_matrix()
        
        # 对点坐标进行四舍五入并偏移
        points = np.round(points).astype(int)
        points[:, 0] -= -786  # Shift x coordinates
        points[:, 1] -= -743  # Shift y coordinates
        
        # 保存为TXT文件，每行x y z以空格分隔
        np.savetxt(output_txt_path, points, delimiter=' ', fmt='%d')
        print(f"已保存: {output_filename}")
        
    except Exception as e:
        print(f"处理文件 {obj_file} 时出错: {e}")
        continue  # 继续处理下一个文件

print("批量转换完成！")