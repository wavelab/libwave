import subprocess
import sys
import shutil

#This script takes as arguments the path to the odometry executable, the path to the kitti dataset, and the path to the error evaluation executable
# It runs each training dataset in kitti, then calculates error metrics

sequences = ['00', '01', '02', '03', '04', '05', '06', '07', '08', '09', '10']
#sequences = ['00']#, '01', '02', '03', '04', '05', '06', '07', '08', '09', '10']

if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit()

    odom_exec = sys.argv[1]
    kitti_data = sys.argv[2]
    eval_exec = sys.argv[3]

    # Copy config folder into results directory to record settings
    config_path = '/home/bapskiko/git/libwave/wave_odometry/tests/config'
    shutil.copytree(config_path, 'config')

    for seq in sequences:
        print("Processing sequence ", seq)
        subprocess.call([odom_exec, kitti_data, config_path, '0', seq, '0', '-1'])
        print("Evaluating sequence ", seq)
        subprocess.call([eval_exec, kitti_data + '/poses', '/home/bapskiko/git/libwave/scripts/test', seq])