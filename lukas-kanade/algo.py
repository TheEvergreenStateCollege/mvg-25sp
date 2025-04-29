# Load first two images and compute

import numpy as np
import matplotlib.pyplot as plt

# for each image pixels, compute these quantities, which are scalars in black-white image intensity.

# I_X(q) I_Y(q)   I_T(q)

import os
from os import walk

# For 5x5 image size
WINDOW_SIZE = 5


def read_pgm(filename):
    with open(filename, 'rb') as pgmf:
        header = pgmf.readline()
        if header != b'P5\n':
            raise ValueError("Not a PGM file")

        size = pgmf.readline().split()
        width = int(size[0])
        height = int(size[1])

        # Make sure gray value is never more than pure white
        assert maxval <= 255
        maxval = int(pgmf.readline())
        raster = np.fromfile(pgmf, dtype=np.ubyte,
                             count=width*height).reshape((height, width))

        return raster


# Example usage
image_array = read_pgm('image.pgm')
plt.imshow(image_array, cmap='gray')
plt.show()

# NetPBM black and white images


def open_pgm(filename):
    with open(filename, "rb") as f:
        data = f.read()
        print(type(data))
        for (index, byte) in enumerate(data):
            print(f"{index}: {data}\n")


f = []
for (dirpath, dirnames, filenames) in walk("flowergarden"):
    for image_fn in filenames:
        print(f"Filename {image_fn}")
        f.append(image_fn)

        open_pgm(os.path.join("flowergarden", image_fn))
        # Run Lukas-Kanade on two consecutive images in path
