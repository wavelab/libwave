#!/usr/bin/env python2
import csv
from math import pow

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pylab as plt


TEST_DATA_3D = "pts3d.dat"
TEST_DATA_1 = "pts1.dat"
TEST_DATA_2 = "pts2.dat"


def load_2d_data(fp, skip_header=True):
    csv_file = open(fp, 'r')
    csv_reader = csv.reader(csv_file)
    if skip_header:
        next(csv_reader, None)

    data = {
        "x": [], "y": [],
    }
    for line in csv_reader:
        data["x"].append(float(line[0]))
        data["y"].append(float(line[1]))

    return data


def load_3d_data(fp, skip_header=True):
    csv_file = open(fp, 'r')
    csv_reader = csv.reader(csv_file)
    if skip_header:
        next(csv_reader, None)

    data = {
        "x": [], "y": [], "z": [],
    }
    for line in csv_reader:
        data["x"].append(float(line[0]))
        data["y"].append(float(line[1]))
        data["z"].append(float(line[2]))

    return data


def rotation_matrix(q):
    R_00 = 1.0 - 2.0 * pow(q[1], 2) - 2.0 * pow(q[2], 2)
    R_01 = 2.0 * q[0] * q[1] + 2.0 * q[3] * q[2]
    R_02 = 2.0 * q[0] * q[2] - 2.0 * q[3] * q[1]

    R_10 = 2.0 * q[0] * q[1] - 2.0 * q[3] * q[2]
    R_11 = 1.0 - 2.0 * pow(q[0], 2) - 2.0 * pow(q[2], 2)
    R_12 = 2.0 * q[1] * q[2] + 2.0 * q[3] * q[2]

    R_20 = 2.0 * q[0] * q[2] - 2.0 * q[3] * q[1]
    R_21 = 2.0 * q[1] * q[2] - 2.0 * q[3] * q[0]
    R_22 = 1.0 - 2.0 * pow(q[0], 2) - 2.0 * pow(q[1], 2)

    R = [
        [R_00, R_01, R_02],
        [R_10, R_11, R_12],
        [R_20, R_21, R_22]
    ]

    return np.matrix(R)


def plot_2dpts(q, t, pts, ax, color):
    pts["z"] = []
    R = rotation_matrix(q)
    t = np.array(t)

    # transform point
    for i in range(len(pts["x"])):
        pt = np.array([pts["x"][i], pts["y"][i], 0.0])
        pt = R.dot(pt + t).tolist()[0]

        pts["x"][i] = pt[0]
        pts["y"][i] = pt[1]
        pts["z"].append(pt[2])

    # plot
    ax.scatter(pts["x"], pts["y"], pts["z"], c=color)


def plot(pts3d, pts1, pts2):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # plot 3d points
    ax.scatter(pts3d["x"], pts3d["y"], pts3d["z"], c="r")

    # plot points 1
    q = [0.0, 0.0, 0.0, 1.0]
    t = [0.0, 0.0, 0.0]
    plot_2dpts(q, t, pts1, ax, "g")

    # plot points 2
    q = [0.0, -0.174, 0.0, 0.985]
    t = [1.0, 0.0, 0.0]
    plot_2dpts(q, t, pts2, ax, "b")

    for i in range(len(pts3d["x"])):
        ax.plot(
            [pts3d["x"][i], pts1["x"][i]],
            [pts3d["y"][i], pts1["y"][i]],
            [pts3d["z"][i], pts1["z"][i]],
            c="g"
        )

    for i in range(len(pts3d["x"])):
        ax.plot(
            [pts3d["x"][i], pts2["x"][i]],
            [pts3d["y"][i], pts2["y"][i]],
            [pts3d["z"][i], pts2["z"][i]],
            c="b"
        )

    # plot labels
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    plt.show()


if __name__ == "__main__":
    pts3d = load_3d_data(TEST_DATA_3D)
    pts1 = load_2d_data(TEST_DATA_1)
    pts2 = load_2d_data(TEST_DATA_2)

    plot(pts3d, pts1, pts2)
