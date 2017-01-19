#!/usr/bin/env python2
import csv
import matplotlib.pylab as plt

# GLOBAL VARIABLES
KF_OUTPUT_FILE = "/tmp/estimation_kf_test.output"
EKF_OUTPUT_FILE = "/tmp/estimation_ekf_test.output"


def load_file(fp, skip_header=True):
    csv_file = open(fp, 'r')
    csv_reader = csv.reader(csv_file)
    if skip_header:
        next(csv_reader, None)

    data = {
        "i": [],
        "x": [], "y": [], "z": [],
        "bx": [], "by": [], "bz": []
    }
    for line in csv_reader:
        data["i"].append(float(line[0]))
        data["x"].append(float(line[1]))
        data["y"].append(float(line[2]))
        data["z"].append(float(line[3]))
        data["bx"].append(float(line[4]))
        data["by"].append(float(line[5]))
        data["bz"].append(float(line[6]))

    return data


def plot_kf_results():
    data = load_file(KF_OUTPUT_FILE)
    plt.plot(data["x"], data["y"], label="Ground State")
    plt.plot(data["bx"], data["by"], marker="o", label="Belief State")
    plt.title("KF - Test")
    plt.legend()
    plt.show()


def plot_ekf_results():
    data = load_file(EKF_OUTPUT_FILE)
    plt.plot(data["x"], data["y"], label="Ground State")
    plt.plot(data["bx"], data["by"], marker="o", label="Belief State")
    plt.title("EKF - Test")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    plot_kf_results()
    plot_ekf_results()
