import numpy as np
import os
import pathlib
import pickle as pkl
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from skspatial.objects import Line, Point
import pandas as pd
import math

from copy import deepcopy


def load_all_paths(path_name):
    indexing = {"N": 0, "NE": 1, "E": 2, "SE": 3, "S": 4, "SW": 5, "W": 6, "NW": 7}
    current_path = str(pathlib.Path().resolve())
    paths = {}
    names = []
    for file in os.listdir(current_path + "/" + path_name):
        temp_path = current_path + "/" + path_name + "/" + str(file)
        names.append(str(file))
        paths[file+"_TT"] = {}
        paths[file+"_BB"] = {}
        paths[file+"_MM"] = {}
        paths[file+"_TB"] = {}
        paths[file+"_BT"] = {}
        paths[file+"_TM"] = {}
        paths[file+"_MT"] = {}
        paths[file+"_BM"] = {}
        paths[file+"_MB"] = {}
        # print("NAME", file)
        for file_sub in os.listdir(temp_path):
            temp_path2 = temp_path + "/" + str(file_sub)
            prefix = temp_path2.split("/")
            start = prefix[-1].split("_", 2)[1]
            prefix, t = prefix[-1].split("_", 1)
            paths[file+"_"+start][str(prefix)] = temp_path2

    # print("KEYS", paths.keys())
    # print(paths[list(paths.keys())[0]]["N"])
    return paths, names


def get_xy(trial_data):
    x = []
    y = []
    for i in trial_data:
        x.append(i["obj_pos"][0])
        y.append(i["obj_pos"][1])
    return np.array(x), np.array(y)


def get_force_info_correct(trial_data):
    normal_force_f1 = []
    normal_force_f2 = []
    lateral_friction1 = []
    lateral_friction2 = []
    for i in trial_data:
        total_norm1x = 0
        total_norm1y = 0
        total_norm1z = 0
        total_norm2x = 0
        total_norm2y = 0
        total_norm2z = 0
        for j in i["cp1"]:
            total_norm1x += j[7][0] * j[9]
            total_norm1y += j[7][1] * j[9]
            total_norm1z += j[7][2] * j[9]
        for j in i["cp2"]:
            total_norm2x += j[7][0] * j[9]
            total_norm2y += j[7][1] * j[9]
            total_norm2z += j[7][2] * j[9]

        normal_force_f1.append(math.sqrt(total_norm1x**2 + total_norm1y**2 + total_norm1z**2))
        normal_force_f2.append(math.sqrt(total_norm2x**2 + total_norm2y**2 + total_norm2z**2))
    return normal_force_f1, normal_force_f2, lateral_friction1, lateral_friction2


def get_force_info(trial_data):
    normal_force_f1 = []
    normal_force_f2 = []
    lateral_friction1 = []
    lateral_friction2 = []
    for i in trial_data:
        total_norm1 = 0
        total_norm2 = 0
        total_fric1 = 0
        total_fric2 = 0
        for j in i["cp1"]:
            total_norm1 += j[9]
            total_fric1 += j[10]
        for j in i["cp2"]:
            total_norm2 += j[9]
            total_fric2 += j[10]
        normal_force_f1.append(total_norm1)
        normal_force_f2.append(total_norm2)
        lateral_friction1.append(total_fric1)
        lateral_friction2.append(total_fric2)
    return normal_force_f1, normal_force_f2, lateral_friction1, lateral_friction2


def get_forces(paths, name, best=None, worst=None):
    colors = ["blue", "red", "green", "olive", "purple", "cyan", "orange", "deeppink"]
    indexing = {"N": 0, "NE": 1, "E": 2, "SE": 3, "S": 4, "SW": 5, "W": 6, "NW": 7}
    fig, axes = plt.subplots(2, 4)
    cnt = 0
    for trial in list(paths[name].keys()):
        with open(paths[name][trial], 'rb') as f:
            if cnt < 4:
                ind = 0
            else:
                ind = 1
            trial_data = pkl.load(f)
            # print(trial_data)
            nf1, nf2, lf1, lf2 = get_force_info(trial_data)
            #print(len(nf1))
            #print(nf1)
            axes[ind][cnt % 4].plot(np.arange(len(nf1)), nf1, color="blue", label="Normal Force F1")
            #axes[ind][cnt % 4].plot(np.arange(len(nf2)), nf2, color="red", label="Normal Force F2", alpha=.7)
            # axes[ind][cnt % 4].plot(np.arange(len(lf1)), lf1, "--", color="red", label="Lateral Friction F1")
            # axes[ind][cnt % 4].plot(np.arange(len(lf2)), lf2, "--", color="red", label="Lateral Friction F2")
            axes[ind][cnt % 4].set_title(trial)
            axes[ind][cnt % 4].set_xlabel("Timesteps")
            axes[ind][cnt % 4].set_ylabel("Force (N)")
            axes[ind][cnt % 4].legend(loc="upper left")
            # axes[ind][cnt % 4].set_ylim(-60, 60)
            cnt += 1
    plt.show()


if __name__ == "__main__":
    # create_quickstats()
    df = pd.read_pickle("quickstats_starting_positions.pkl")
    df_TT = df[(df["start"] == "TT")]
    df_BB = df[(df["start"] == "BB")]
    df_MM = df[(df["start"] == "MM")]
    # '3v3_25.35.40_33.3.33.3.33.3_0.95.1_53'
    paths, names = load_all_paths(path_name="data_starting_positions")
    # get_forces(paths, '3v3_25.35.40_33.3.33.3.33.3_0.95.1_53_MM')
    get_forces(paths, '2v2_50.50_50.50_1.1_53_MM')
    get_forces(paths, '3v3_25.35.40_33.3.33.3.33.3_0.95.1_53_MM')
