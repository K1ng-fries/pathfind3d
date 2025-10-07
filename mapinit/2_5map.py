import numpy as np
import os

def create_25d_grid(input_txt_path, output_dir, output_filename="grid.npy", x_max=1891, y_max=1330, z_min=-22):
    """
    Create a 2.5D grid from point cloud data, taking max z for each (x, y) cell, and save as NPY.
    
    Args:
        input_txt_path (str): Path to input TXT file with point cloud data (x y z per line)
        output_dir (str): Directory to save output NPY file
        output_filename (str): Name of the output NPY file
        x_max (int): Maximum x value (inclusive)
        y_max (int): Maximum y value (inclusive)
        z_min (int): Minimum z value for initialization
    """
    # Calculate grid dimensions
    grid_width = x_max + 1  # 0 to x_max inclusive
    grid_height = y_max + 1  # 0 to y_max inclusive

    # Ensure input file exists
    if not os.path.exists(input_txt_path):
        print(f"Input TXT file {input_txt_path} does not exist")
        return

    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    output_npy_path = os.path.join(output_dir, output_filename)

    # Read point cloud data (assuming integers)
    try:
        points = np.loadtxt(input_txt_path, delimiter=' ', dtype=int)
    except Exception as e:
        print(f"Failed to read TXT file {input_txt_path}: {e}")
        return

    # Initialize grid with z_min (for cells with no points)
    grid = np.full((grid_width, grid_height), z_min, dtype=int)

    # Update grid with max z for each (x, y)
    for x, y, z in points:
        if 0 <= x <= x_max and 0 <= y <= y_max:
            grid[x, y] = max(grid[x, y], z)
        else:
            print(f"Point ({x}, {y}, {z}) out of bounds, skipped")

    # Save grid to NPY file
    try:
        np.save(output_npy_path, grid)
        print(f"2.5D grid saved to: {output_npy_path}")
        print(f"Grid shape: {grid.shape}")
    except Exception as e:
        print(f"Failed to save NPY file {output_npy_path}: {e}")

if __name__ == "__main__":
    input_txt_path = "F:\my pathfinding\Data_txt\outputlinear.txt"
    output_dir = "F:\my pathfinding"
    create_25d_grid(input_txt_path, output_dir)