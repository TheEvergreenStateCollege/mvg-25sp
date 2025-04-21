# Load first two images and compute

import numpy
import matplotlib.pyplot as plt

# for each image pixels, compute these quantities, which are scalars in black-white image intensity.

# I_X(q) I_Y(q)   I_T(q)

import os
from os import walk

# For 5x5 image size
WINDOW_SIZE = 5


# NetPBM black and white images
def open_pgm(filename):
    with open(filename, "rb") as f:
        data = f.read()
        print(type(data))
        # for (index, byte) in enumerate(data):
        #    print(f"{index}: {data}\n")


f = []
for (dirpath, dirnames, filenames) in walk("flowergarden"):
    for image_fn in filenames:
        print(f"Filename {image_fn}")
        open_pgm(os.path.join("flowergarden", image_fn))
        # Run Lukas-Kanade on two consecutive images in path
