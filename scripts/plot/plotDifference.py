import csv
import matplotlib.pylab as plt
import numpy as np

GROUND_TRUTH_FILE = "/home/ben/git/docs/thesis/MASc/2018-Benjamin_Skikos-Lidar_Odometry/data/moose/base/ground_truth.txt"
CONFIG_A_PATH = "/home/ben/git/docs/thesis/MASc/2018-Benjamin_Skikos-Lidar_Odometry/data/moose/base/11.txt"
CONFIG_B_PATH = "/home/ben/git/docs/thesis/MASc/2018-Benjamin_Skikos-Lidar_Odometry/data/moose/Wintensity/11.txt"
CONFIG_C_PATH = "/home/ben/git/docs/thesis/MASc/2018-Benjamin_Skikos-Lidar_Odometry/data/moose/largewindow/11.txt"

def importData(path):
    data = {
        "R11": [],
        "R12": [],
        "R13": [],
        "R21": [],
        "R22": [],
        "R23": [],
        "R31": [],
        "R32": [],
        "R33": [],
        "Tx": [],
        "Ty": [],
        "Tz": []
    }

    file_data = open(path, "r")
    reader = csv.reader(file_data, delimiter=" ")

    for row in reader:
        data["R11"].append(float(row[0]))
        data["R12"].append(float(row[1]))
        data["R13"].append(float(row[2]))
        data["Tx"].append(float(row[3]))
        data["R21"].append(float(row[4]))
        data["R22"].append(float(row[5]))
        data["R23"].append(float(row[6]))
        data["Ty"].append(float(row[7]))
        data["R31"].append(float(row[8]))
        data["R32"].append(float(row[9]))
        data["R33"].append(float(row[10]))
        data["Tz"].append(float(row[11]))

    return data

def plotData(data, gt, series_name, dim):
    plt.plot(np.asarray(data[dim]) - np.asarray(gt[dim]), label=series_name, linewidth=0.5)

def savePlot(ylabel):
    plt.xlabel("Frame Index")
    plt.ylabel(ylabel)
    plt.legend(loc=0)
    plt.savefig(ylabel + 'plot.eps', format="eps", bbox_inches="tight")

if __name__ == "__main__":
    ground_truth = importData(GROUND_TRUTH_FILE)
    cA_data = importData(CONFIG_A_PATH)
    cB_data = importData(CONFIG_B_PATH)
    cC_data = importData(CONFIG_C_PATH)

    dims = ["Tx", "Tz", "Ty"]
    for dim in dims:
        plotData(cA_data, ground_truth, "Config A", dim)
        plotData(cB_data, ground_truth, "Config B", dim)
        plotData(cC_data, ground_truth, "Config C", dim)
        savePlot(dim)
        plt.clf()