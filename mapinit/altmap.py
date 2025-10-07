import numpy as np
import matplotlib.pyplot as plt
import os

def visualize_25d_grid(input_npy_path, output_dir, output_filename="grid_visual3d.png", cmap='terrain'):
    """
    Save a pure pixel-for-pixel image of the 2.5D grid without any borders, labels, or extras.
    
    Args:
        input_npy_path (str): Path to input NPY file with 2D grid (x, y) -> z
        output_dir (str): Directory to save output PNG file
        output_filename (str): Name of the output PNG file
        cmap (str): Colormap for visualization (e.g., 'terrain', 'viridis')
    """
    # Ensure input file exists
    if not os.path.exists(input_npy_path):
        print(f"Input NPY file {input_npy_path} does not exist")
        return

    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    output_png_path = os.path.join(output_dir, output_filename)

    # Memory-mapped loading to handle large files efficiently
    try:
        grid = np.load(input_npy_path, mmap_mode='r')
    except Exception as e:
        print(f"Failed to load NPY file {input_npy_path}: {e}")
        return

    # Normalize grid for colormap (optional: set vmin/vmax if needed)
    # Here, it will auto-scale to min/max of grid
    try:
        # Use imsave for direct pixel output without figure elements
        plt.imsave(output_png_path, grid, cmap=cmap)
        print(f"Pure grid image saved to: {output_png_path}")
        print(f"Image dimensions: {grid.shape[1]} width x {grid.shape[0]} height pixels")
    except Exception as e:
        print(f"Failed to save PNG file {output_png_path}: {e}")

if __name__ == "__main__":
    input_npy_path = "F:\my pathfinding/fullmatrix.npy"
    output_dir = "F:\my pathfinding"
    visualize_25d_grid(input_npy_path, output_dir)