import coordinateGeneration
from coordinateGeneration import x_coords, z_coords
import time

print('Number of points: ', len(x_coords))
for n in range(len(x_coords) - 2):
    print(x_coords[n+1], z_coords[n+1], '\n')
    time.sleep(1)