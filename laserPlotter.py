import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader

def plot_ranges(filename):
    headers, values=FileReader(filename).read_file()
    ranges = np.array([v[0] for v in values])
    angle_increment = [v[1] for v in values]
    theta = np.cumsum(angle_increment)

    nonInf = ~np.isinf(ranges)
    ranges = ranges[nonInf]
    theta = theta[nonInf]

    x = ranges * np.cos(theta)
    y = ranges * np.sin(theta)

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
