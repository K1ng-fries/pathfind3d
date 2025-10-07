# For different visualization styles with your data:
import pyvista as pv
import numpy as np

height_data = np.load('grid.npy')

def elevation_zones_visualization(data, zone_height=None):
    """将地形按高程带分层显示，这是最清晰的方法"""
    import pyvista as pv
    import numpy as np
    
    # 创建网格
    x = np.arange(data.shape[1])
    y = np.arange(data.shape[0])
    X, Y = np.meshgrid(x, y)
    grid = pv.StructuredGrid(X, Y, data)
    
    # 自动确定高程带高度（如果不指定）
    if zone_height is None:
        height_range = np.max(data) - np.min(data)
        zone_height = height_range / 15  # 分为15个带
    
    # 为每个高程带分配颜色
    min_elev = np.min(data)
    elevation_zones = ((data - min_elev) / zone_height).astype(int)
    
    plotter = pv.Plotter()
    mesh = plotter.add_mesh(
        grid,
        scalars=elevation_zones.ravel(),
        cmap='Set3',  # 离散的、对比度强的颜色
        show_scalar_bar=True,
        scalar_bar_args={'title': f"Elevation Zones ({zone_height:.1f} units)"}
    )
    
    plotter.add_text("Elevation Zone Visualization", position='upper_edge')
    plotter.show()
    return plotter

# 使用高程带显示
elevation_zones_visualization(height_data)


