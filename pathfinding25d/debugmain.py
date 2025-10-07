import numpy as np
import time
from finder import AStar3D   # 替换为您的实际模块名

def main():
    # 加载大地图
    print("加载地图数据...")
    try:
        height_data = np.load('grid.npy')
        print(f"地图加载成功，尺寸: {height_data.shape}")
    except FileNotFoundError:
        print("错误: 找不到 grid.npy 文件")
        return
    except Exception as e:
        print(f"加载地图时出错: {e}")
        return
    
    # 创建A*查找器
    print("初始化A*算法...")
    astar = AStar3D(height_data, min_altitude=10, max_altitude=200)
    
    # 设置起点和终点 - 使用实际地图范围内的坐标
    # 假设地图尺寸为1891×1330，选择合理的起点和终点
    start = (899, 999, 35)   # (x, y, z)
    end = (500, 999, 35)    # (x, y, z)

    print(f"起点可达: {AStar3D.can_reach(astar, start)}")
    print(f"终点可达: {AStar3D.can_reach(astar, end)}")
    
    print(f"开始寻路: {start} -> {end}")
    print("这可能需要一些时间，请耐心等待...")
    
    # 记录开始时间
    start_time = time.time()
    
    try:
        # 执行路径查找
        path = astar.find_path_bidirectional(start, end)
        
        # 计算耗时
        elapsed_time = time.time() - start_time
        
        if path:
            print(f"✓ 路径查找成功!")
            print(f"路径长度: {len(path)} 个点")
            print(f"耗时: {elapsed_time:.2f} 秒")
            print(f"起点: {path[0]}")
            print(f"终点: {path[-1]}")
            
            # 显示部分路径点（前5个和后5个）
            if len(path) > 10:
                print("路径预览 (前5个点):")
                for i, point in enumerate(path[:5]):
                    print(f"  {i+1}: {point}")
                print("  ...")
                print("路径预览 (后5个点):")
                for i, point in enumerate(path[-5:], len(path)-4):
                    print(f"  {i}: {point}")
            else:
                print("完整路径:")
                for i, point in enumerate(path):
                    print(f"  {i+1}: {point}")
        else:
            print("✗ 未找到路径!")
            print(f"搜索耗时: {elapsed_time:.2f} 秒")

        print("正在启动可视化...")
        astar.visualize(path=path)    
    except MemoryError:
        print("✗ 内存不足! 地图太大导致内存溢出")
    except KeyboardInterrupt:
        print("✗ 用户中断了搜索")
    except Exception as e:
        print(f"✗ 寻路过程中出错: {e}")

if __name__ == "__main__":
    main()
