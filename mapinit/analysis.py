import numpy as np

class GridAnalyzer:
    def __init__(self, file_path):
        """
        Initialize the GridAnalyzer with the path to the grid NPY file.
        
        Args:
            file_path (str): Path to the grid.npy file
        """
        self.grid = np.load(file_path)
        print(f"Grid loaded with shape: {self.grid.shape}")

    def count_zeros(self):
        """
        Count the number of zeros in the grid.
        
        Returns:
            int: Number of zero values in the grid
        """
        return np.sum(self.grid == 0)

if __name__ == "__main__":
    file_path = "F:\my pathfinding\grid.npy"
    analyzer = GridAnalyzer(file_path)
    zero_count = analyzer.count_zeros()
    print(f"Number of zeros in the grid: {zero_count}")