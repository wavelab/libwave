import csv
import matplotlib.pylab as plt

GROUND_TRUTH_FILE = "/home/ben/rosbags/dataset/poses/09.txt"
CONFIG_A_PATH = "/home/ben/git/docs/thesis/MASc/2018-Benjamin_Skikos-Lidar_Odometry/data/kitti/plane_edges_3/09.txt"
CONFIG_B_PATH = "/home/ben/git/docs/thesis/MASc/2018-Benjamin_Skikos-Lidar_Odometry/data/kitti/plane_edges_int_3/09.txt"
CONFIG_C_PATH = "/home/ben/git/docs/thesis/MASc/2018-Benjamin_Skikos-Lidar_Odometry/data/kitti/long_window/09.txt"
CONFIG_D_PATH = "/home/ben/git/docs/thesis/MASc/2018-Benjamin_Skikos-Lidar_Odometry/data/kitti/plane_edges_int_long_2/09.txt"

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

def plotData(data, series_name):
    plt.plot(data["Tx"], data["Tz"], label=series_name, linewidth=0.3)

def savePlot():
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend(loc=0)
    plt.savefig('plot.eps', format="eps", bbox_inches="tight")

if __name__ == "__main__":
    plotData(importData(GROUND_TRUTH_FILE), "Ground Truth")
    plotData(importData(CONFIG_A_PATH), "Geo, SWF = 5")
    plotData(importData(CONFIG_B_PATH), "Geo + Int, SWF = 5")
    plotData(importData(CONFIG_C_PATH), "Geo, SWF = 10")
    plotData(importData(CONFIG_D_PATH), "Geo + Int, SWF = 10")
    savePlot()