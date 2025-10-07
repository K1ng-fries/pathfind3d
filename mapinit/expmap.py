import numpy as np
import matplotlib.pyplot as plt
import os

def visualize_25d_grid(input_npy_path, output_dir, output_filename="grid_visual.png", cmap='terrain'):
    """
    Visualize a 2.5D grid as a heatmap where color represents z-height, using efficient memory handling.
    
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

    # Get z min and max for normalization (using min/max to avoid full array copy if possible)
    z_min = np.min(grid)
    z_max = np.max(grid)

    # Create figure with appropriate size for large grid (downsample if too large for display)
    fig, ax = plt.subplots(figsize=(10, 7))  # Adjust figsize for better aspect ratio

    # Use imshow for efficient rendering
    im = ax.imshow(grid, cmap=cmap, vmin=z_min, vmax=z_max, interpolation='nearest')

    # Add colorbar
    cbar = plt.colorbar(im, ax=ax, shrink=0.5)
    cbar.set_label('Height (z)')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('2.5D Grid Visualization')

    # Save to PNG (efficient, no display needed)
    try:
        plt.savefig(output_png_path, dpi=1000, bbox_inches='tight')  # Lower DPI for faster save
        print(f"Visualization saved to: {output_png_path}")
    except Exception as e:
        print(f"Failed to save PNG file {output_png_path}: {e}")
    finally:
        plt.close(fig)  # Close to free memory

if __name__ == "__main__":
    input_npy_path = "F:\my pathfinding\grid.npy"
    output_dir = "F:\my pathfinding"
    visualize_25d_grid(input_npy_path, output_dir)