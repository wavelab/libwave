#!/usr/bin/env python2
import random
import numpy as np
import matplotlib.pylab as plt

# GLOBAL VARIABLES
OUTPUT_FILE = "ransac_sample.dat"


def frange(x, y, jump):
    while x < y:
        yield x
        x += jump


def line_sample_data(n, x_start, x_end, m=20, c=10):
    data = {"x": [], "y": [], "m": [], "c": []}
    step_size = (x_end - x_start) / float(n)
    x_data = list(frange(x_start, x_end, step_size))

    for i in range(n):
        y = m * x_data[i] + c
        data["x"].append(np.random.normal(x_data[i], 1.0, 1)[0])
        data["y"].append(np.random.normal(y, 1.0, 1)[0])
        data["m"].append(m)
        data["c"].append(c)

    return data


def add_outliers(data, n):
    x_min = min(data["x"])
    x_max = max(data["x"])
    y_min = min(data["y"])
    y_max = max(data["y"])

    for i in range(n):
        data["x"].append(random.uniform(x_min, x_max))
        data["y"].append(random.uniform(y_min, y_max))

    return data


def save_data(data):
    output_file = open(OUTPUT_FILE, "w")

    output_file.write("x, y\n")
    for i in range(len(data["x"])):
        line = "{0}, {1}\n".format(data["x"][i], data["y"][i])
        output_file.write(line)

    output_file.close()


def plot_data(data):
    plt.scatter(data["x"], data["y"])
    plt.show()


if __name__ == "__main__":
    data = line_sample_data(200, 0, 100)
    data = add_outliers(data, 80)
    save_data(data)
    plot_data(data)
