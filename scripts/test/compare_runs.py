import sys
import difflib
from pathlib import Path
import matplotlib.pyplot as plt

# This script takes as arguments two paths containing results from two runs and compares the error between them
# The config files are also parsed and any differences are output.

sequences = ['00', '01', '02', '03', '04', '05', '06', '07', '08', '09', '10']
suffixes = ['_rl.txt', '_rs.txt', '_tl.txt', '_ts.txt']
config_files = ['bin_config.yaml', 'features.yaml', 'odom.yaml']

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Need to provide folders to compare")
        sys.exit()

    result_set_1 = Path(sys.argv[1])
    result_set_2 = Path(sys.argv[2])

    config_1 = result_set_1 / "config"
    config_2 = result_set_2 / "config"

    # Look for the differences between configurations and output
    for c_file in config_files:
        text_1 = (config_1 / c_file).read_text()
        lines_1 = text_1.split("\n")
        text_2 = (config_2 / c_file).read_text()
        lines_2 = text_2.split("\n")
        print("Comparing " + c_file)
        for line in difflib.unified_diff(lines_1, lines_2, "file_1", "file_2", lineterm="\n"):
            print(line)

    # plot the error from each sequence
    error_path_1 = result_set_1 / 'plot_error'
    error_path_2 = result_set_2 / 'plot_error'
    for seq in sequences:
        for suffix in suffixes:
            error_1 = (error_path_1 / (seq + suffix)).read_text().splitlines()
            error_2 = (error_path_2 / (seq + suffix)).read_text().splitlines()
            x1 = []
            y1 = []
            x2 = []
            y2 = []
            for elem in error_1:
                coords = elem.split(" ")
                x1.append(float(coords[0]))
                y1.append(float(coords[1]))
            for elem in error_2:
                coords = elem.split(" ")
                x2.append(float(coords[0]))
                y2.append(float(coords[1]))
            plt.plot(x1, y1, label=result_set_1)
            plt.plot(x2, y2, label=result_set_2)
            plt.legend()
            plt.title("Sequence " + seq + " Suffix " + suffix)
            plt.show(True)