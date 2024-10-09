import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader

"""
This file creates a scatter plot for the lidar data at a point in time.
Mostly copied from existing filePlotter.py
"""

def plot_ranges(filename):
    headers, values=FileReader(filename).read_file()
    ranges = np.array([v[0] for v in values])
    angle_increment = [v[1] for v in values]
    # Integrate angle_increment
    theta = np.cumsum(angle_increment)

    # Filter out infinite depths
    nonInf = ~np.isinf(ranges)
    ranges = ranges[nonInf]
    theta = theta[nonInf]

    # Convert polar to euclidean
    x = ranges * np.cos(theta)
    y = ranges * np.sin(theta)

    # Scatter plot with proportional x and y axes
    plt.scatter(x,y, s=2)
    plt.axis('equal')
    plt.grid()
    plt.show()


import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_ranges(filename)
