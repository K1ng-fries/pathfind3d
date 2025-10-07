import sys
sys.path.append('.')
import numpy as np

from pathfinding3d.core.diagonal_movement import DiagonalMovement
from pathfinding3d.core.grid import Grid
from pathfinding3d.finder.a_star import AStarFinder
from moudles.spot import Spot_correction

'''
# Create a 3D numpy array with 0s as obstacles and 1s as walkable paths
matrix = np.ones((10, 10, 10), dtype=np.int8)
# mark the center of the grid as an obstacle
matrix[5, 5, 5] = 0
'''
# Load the matrix from the npy file
matrix = np.load('F:\my pathfinding\matrix.npy')
# Create a grid object from the numpy array
grid = Grid(matrix=matrix)

# Mark the start and end points
startpoint = (0, 0, 0)
endpoint = (10, 10, 10)

# find nearest free point
spot_corrector = Spot_correction(matrix)
startpoint=spot_corrector.find_nearest_free_point_bfs(startpoint)
endpoint=spot_corrector.find_nearest_free_point_bfs(endpoint)

print("Corrected startpoint:", startpoint)
print("Corrected endpoint:", endpoint)

start = grid.node(startpoint[0], startpoint[1], startpoint[2])
end = grid.node(endpoint[0], endpoint[1], endpoint[2])


# Create an instance of the A* finder with diagonal movement allowed
finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
path, runs = finder.find_path(start, end, grid)

# Path will be a list with all the waypoints as nodes
# Convert it to a list of coordinate tuples
path = [p.identifier for p in path]

print("operations:", runs, "path length:", len(path))
print("path:", path)

grid.visualize(
  path=path,  # optionally visualize the path
  start=start,
  end=end,
  visualize_weight=True,  # weights above 1 (default) will be visualized
  save_html=True,  # save visualization to html file
  save_to="path_visualization.html",  # specify the path to save the html file
  always_show=False,  # always show the visualization in the browser
)
