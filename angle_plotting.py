import numpy as np
import os
import pathlib
import pickle as pkl
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from skspatial.objects import Line, Point
import math
from copy import deepcopy


def plot_joint_angles_delta(paths, names):
    ind = 0
    figure, axis = plt.subplots(2, 4)
    for i in range(len(names)):
        with open(paths[names[i]], 'rb') as f:
            trial = pkl.load(f)
            j1 = []
            j2 = []
            j3 = []
            j4 = []
            j1_act = []
            j2_act = []
            j3_act = []
            j4_act = []
            for j in range(1, len(trial)):
                j1.append(trial[j]["ik_angles"][0] - trial[j-1]["ik_angles"][0])
                j2.append(trial[j]["ik_angles"][1] - trial[j-1]["ik_angles"][1])
                j3.append(trial[j]["ik_angles"][2] - trial[j-1]["ik_angles"][2])
                j4.append(trial[j]["ik_angles"][3] - trial[j-1]["ik_angles"][3])
                j1_act.append(trial[j]["joint_1"] - trial[j-1]["joint_1"])
                j2_act.append(trial[j]["joint_2"] - trial[j-1]["joint_2"])
                j3_act.append(trial[j]["joint_3"] - trial[j-1]["joint_3"])
                j4_act.append(trial[j]["joint_4"] - trial[j-1]["joint_4"])
            if i == 4:
                ind = 0
            axis[i//4, ind].plot(np.arange(min(200, len(j1))), j1[:min(200, len(j1))], color="red", label="J1_IK")
            axis[i//4, ind].plot(np.arange(min(200, len(j1_act))), j1_act[:min(200, len(j1_act))], "--", color="red")
            axis[i//4, ind].plot(np.arange(min(200, len(j2))), j2[:min(200, len(j2))], color="blue", label="J2_IK")
            axis[i//4, ind].plot(np.arange(min(200, len(j2_act))), j2_act[:min(200, len(j2_act))], "--", color="blue")
            axis[i//4, ind].plot(np.arange(min(200, len(j3))), j3[:min(200, len(j3))], color="green", label="J3_IK")
            axis[i//4, ind].plot(np.arange(min(200, len(j3_act))), j3_act[:min(200, len(j3_act))], "--", color="green")
            axis[i//4, ind].plot(np.arange(min(200, len(j4))), j4[:min(200, len(j4))], color="purple", label="J4_IK")
            axis[i//4, ind].plot(np.arange(min(200, len(j4_act))), j4_act[:min(200, len(j4_act))], "--", color="purple")
            axis[i//4, ind].set_title(names[i] + " (joint angle changes)")
            axis[i//4, ind].set_xlabel("Timesteps")
            axis[i//4, ind].set_ylabel("Delta Theta (Radians)")
            axis[i//4, ind].axhline(y=0, color='black', linestyle='-')
            axis[i//4, ind].legend(loc="upper right", ncol=2)
            # axis[i//4, ind].plot(np.arange(len(j1)), j1, color="red")
            # axis[i//4, ind].plot(np.arange(len(j1_act)), j1_act, "--", color="red")
            # axis[i//4, ind].plot(np.arange(len(j2)), j2, color="blue")
            # axis[i//4, ind].plot(np.arange(len(j2_act)), j2_act, "--", color="blue")
            # axis[i//4, ind].plot(np.arange(len(j3)), j3, color="green")
            # axis[i//4, ind].plot(np.arange(len(j3_act)), j3_act, "--", color="green")
            # axis[i//4, ind].plot(np.arange(len(j4)), j4, color="purple")
            # axis[i//4, ind].plot(np.arange(len(j4_act)), j4_act, "--", color="purple")
            # axis[i//4, ind].set_title(names[i])
            # axis[i//4, ind].axhline(y=0, color='black', linestyle='-')
            ind += 1
    plt.show()


def plot_joint_angles(paths, names):
    ind = 0
    figure, axis = plt.subplots(2, 4)
    for i in range(len(names)):
        with open(paths[names[i]], 'rb') as f:
            trial = pkl.load(f)
            j1 = []
            j2 = []
            j3 = []
            j4 = []
            j1_act = []
            j2_act = []
            j3_act = []
            j4_act = []
            for j in range(len(trial)):
                j1.append(trial[j]["ik_angles"][0])
                j2.append(trial[j]["ik_angles"][1])
                j3.append(trial[j]["ik_angles"][2])
                j4.append(trial[j]["ik_angles"][3])
                j1_act.append(trial[j]["joint_1"])
                j2_act.append(trial[j]["joint_2"])
                j3_act.append(trial[j]["joint_3"])
                j4_act.append(trial[j]["joint_4"])
            if i == 4:
                ind = 0
            axis[i//4, ind].plot(np.arange(min(200, len(j1))), j1[:min(200, len(j1))], color="red", label="J1_IK")
            axis[i//4, ind].plot(np.arange(min(200, len(j1_act))), j1_act[:min(200, len(j1_act))], "--", color="red")
            axis[i//4, ind].plot(np.arange(min(200, len(j2))), j2[:min(200, len(j2))], color="blue", label="J2_IK")
            axis[i//4, ind].plot(np.arange(min(200, len(j2_act))), j2_act[:min(200, len(j2_act))], "--", color="blue")
            axis[i//4, ind].plot(np.arange(min(200, len(j3))), j3[:min(200, len(j3))], color="green", label="J3_IK")
            axis[i//4, ind].plot(np.arange(min(200, len(j3_act))), j3_act[:min(200, len(j3_act))], "--", color="green")
            axis[i//4, ind].plot(np.arange(min(200, len(j4))), j4[:min(200, len(j4))], color="purple", label="J4_IK")
            axis[i//4, ind].plot(np.arange(min(200, len(j4_act))), j4_act[:min(200, len(j4_act))], "--", color="purple")
            axis[i//4, ind].set_title(names[i] + "(joint angles over time)")
            axis[i//4, ind].set_xlabel("Timesteps")
            axis[i//4, ind].set_ylabel("Theta (Radians)")
            axis[i//4, ind].axhline(y=0, color='black', linestyle='-')
            axis[i//4, ind].legend(loc="upper right", ncol=2)
            # axis[i//4, ind].plot(np.arange(len(j1)), j1, color="red")
            # axis[i//4, ind].plot(np.arange(len(j1_act)), j1_act, "--", color="red")
            # axis[i//4, ind].plot(np.arange(len(j2)), j2, color="blue")
            # axis[i//4, ind].plot(np.arange(len(j2_act)), j2_act, "--", color="blue")
            # axis[i//4, ind].plot(np.arange(len(j3)), j3, color="green")
            # axis[i//4, ind].plot(np.arange(len(j3_act)), j3_act, "--", color="green")
            # axis[i//4, ind].plot(np.arange(len(j4)), j4, color="purple")
            # axis[i//4, ind].plot(np.arange(len(j4_act)), j4_act, "--", color="purple")
            # axis[i//4, ind].set_title(names[i])
            # axis[i//4, ind].axhline(y=0, color='black', linestyle='-')
            ind += 1
    plt.show()


def load_all_paths():
    current_path = str(pathlib.Path().resolve())
    paths = {}
    names = []
    for file in os.listdir(current_path + "/data_angles"):
        temp_path = current_path + "/data_angles/" + str(file)
        prefix = file.split("_")[0]
        names.append(prefix)
        paths[prefix] = temp_path
    return paths, names


if __name__ == "__main__":
    paths, names = load_all_paths()
    plot_joint_angles(paths, names)
