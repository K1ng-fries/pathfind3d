import numpy as np
from pathfinding25d.finder import AStar3D

# simple flat height map 20x20 with ground height 0
h = np.zeros((20,20), dtype=int)
astar = AStar3D(h, min_altitude=1, max_altitude=10)

start = (1, 1, 2)
end = (18, 18, 2)

p1 = astar.find_path(start, end)
print('find_path length:', len(p1))
print('sample:', p1[:5])

p2 = astar.find_path_bidirectional(start, end)
print('find_path_bidirectional length:', len(p2))
print('sample:', p2[:5])

# compare
print('paths equal length?', len(p1) == len(p2))
