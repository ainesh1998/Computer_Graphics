import random
import numpy as np
from math import floor
from PIL import Image
import matplotlib.pyplot as plt
from collections import deque

heightmapWidth = 65

# initialize the heightmap to 0's
heightmap = [[0]*heightmapWidth for i in range(heightmapWidth)]

# set the corner points to the same random value
rand = random.randint(0, 256)
heightmap[0][0] = rand
heightmap[heightmapWidth - 1][0] = rand
heightmap[0][heightmapWidth - 1] = rand
heightmap[heightmapWidth - 1][heightmapWidth - 1] = rand

# set the randomness bounds, higher values mean rougher landscapes
randomness = 128
tileWidth = heightmapWidth - 1

# we make a pass over the heightmap
# each time we decrease the side length by 2
while tileWidth > 1:
    halfSide = tileWidth // 2

    # set the diamond values (the centers of each tile)
    for x in range(0, heightmapWidth - 1, tileWidth):
        for y in range(0, heightmapWidth - 1, tileWidth):
            # print(tileWidth, x, y)
            cornerSum = heightmap[x][y] + \
                        heightmap[x + tileWidth][y] + \
                        heightmap[x][y + tileWidth] + \
                        heightmap[x + tileWidth][y + tileWidth]

            avg = cornerSum / 4
            avg += random.randint(-randomness, randomness)

            heightmap[x + halfSide][y + halfSide] = avg

    # set the square values (the midpoints of the sides)
    for x in range(0, heightmapWidth - 1, halfSide):
        for y in range((x + halfSide) % tileWidth, heightmapWidth - 1, tileWidth):
            avg = heightmap[(x - halfSide + heightmapWidth - 1) % (heightmapWidth - 1)][y] + \
                  heightmap[(x + halfSide) % (heightmapWidth - 1)][y] + \
                  heightmap[x][(y + halfSide) % (heightmapWidth - 1)] + \
                  heightmap[x][(y - halfSide + heightmapWidth - 1) % (heightmapWidth - 1)]

            avg /= 4.0
            avg += random.randint(-randomness, randomness)

            heightmap[x][y] = avg

            # because the values wrap round, the left and right edges are equal, same with top and bottom
            if x == 0:
                heightmap[heightmapWidth - 1][y] = avg
            if y == 0:
                heightmap[x][heightmapWidth - 1] = avg

    print(heightmap)
    # reduce the randomness in each pass, making sure it never gets to 0
    randomness = max(randomness // 2, 1)
    tileWidth //= 2

# trying to plot it

ax = plt.gca()
plt.imshow(heightmap)
plt.show()
