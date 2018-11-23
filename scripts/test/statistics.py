import pandas as pd
from pathlib import Path
import sys

if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit()

    error_files = Path(sys.argv[1]).glob('**/*.txt')

    for err in error_files:
        data = pd.read_csv(err, delim_whitespace=True)
        mean_data = data.mean(axis=0)

        filename = err.stem
        savename = filename + "_avg.txt"
        savepath = Path(sys.argv[1]) / savename
        degrees = mean_data[1] * 57.29577951
        with open(str(savepath), 'w') as file:
            file.write("%.17f" % degrees)
            file.write(" ")
            file.write("%.17f" % mean_data[2])
