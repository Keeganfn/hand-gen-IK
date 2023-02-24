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


def filter_late_contact(trial_data, x=None, y=None, tstep_cutoff=10, threshold=.001):
    if x is None or y is None:
        x, y = get_xy(trial_data)
    last_n = trial_data[-tstep_cutoff:]
    cutoff = None
    for i in range(len(last_n)):
        if last_n[i]["d1"] > threshold or last_n[i]["d2"] > threshold:
            cutoff = i
            break
    if cutoff is not None:
        trial_data = trial_data[:(len(trial_data) - (tstep_cutoff - cutoff))]
        x = x[:(len(x) - (tstep_cutoff - cutoff))]
        y = y[:(len(y) - (tstep_cutoff - cutoff))]
    return trial_data, x, y


def filter_path_deviation(trial_data, direction, x=None, y=None, threshold=.01):
    indexing = {"N": 0, "NE": 1, "E": 2, "SE": 3, "S": 4, "SW": 5, "W": 6, "NW": 7}
    if x is None or y is None:
        x, y = get_xy(trial_data)
    vectors = [[0, .2], [.2, .2], [.2, 0], [.2, -.2], [0, -.2], [-.2, -.2], [-.2, 0], [-.2, .2]]
    max_distance = 0
    point_projected_max = (0, 0)
    for i in range(len(x)):
        point = Point([x[i], y[i]])
        line = Line(point=[0, 0], direction=vectors[indexing[direction]])
        point_projected = line.project_point(point)
        distance_to_line = math.sqrt((point_projected[0] - x[i])**2 + (point_projected[1] - y[i])**2)
        if distance_to_line > threshold:
            trial_data = trial_data[:i]
            x = x[:i]
            y = y[:i]
            break
        else:
            max_distance = math.sqrt(point_projected[0]**2 + point_projected[1]**2)
            point_projected_max = (point_projected[0], point_projected[1])

    return trial_data, x, y, max_distance, point_projected_max


def get_xy(trial_data):
    x = []
    y = []
    for i in trial_data:
        x.append(i["obj_pos"][0])
        y.append(i["obj_pos"][1])
    return np.array(x), np.array(y)


def plot_asterisk(paths, name):
    colors = ["blue", "red", "green", "olive", "purple", "cyan", "orange", "deeppink"]
    indexing = {"N": 0, "NE": 1, "E": 2, "SE": 3, "S": 4, "SW": 5, "W": 6, "NW": 7}

    projected_points = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]
    for trial in list(paths[name].keys()):
        with open(paths[name][trial], 'rb') as f:
            trial_data = pkl.load(f)
            trial_data, x, y = filter_late_contact(trial_data)
            trial_data, x, y, _, point_projected_max = filter_path_deviation(trial_data, trial, x, y)
            projected_points[indexing[trial]] = point_projected_max
            plt.plot(x, y, linewidth=3, color=colors[indexing[trial]])
    pgon = Polygon(projected_points)
    plt.fill(np.array(projected_points)[:, 0], np.array(projected_points)[:, 1], alpha=.2)
    plt.plot([0, 0], [0, .2], "--", color="blue", alpha=.6)
    plt.plot([0, .1414], [0, .1414], "--", color="red", alpha=.6)
    plt.plot([0, .2], [0, 0], "--", color="green", alpha=.6)
    plt.plot([0, .1414], [0, -.1414], "--", color="olive", alpha=.6)
    plt.plot([0, 0], [0, -.2], "--", color="purple", alpha=.6)
    plt.plot([0, -.1414], [0, -.1414], "--", color="cyan", alpha=.6)
    plt.plot([0, -.2], [0, 0], "--", color="orange", alpha=.6)
    plt.plot([0, -.1414], [0, .1414], "--", color="deeppink", alpha=.6)
    plt.xlim(-.2, .2)
    plt.ylim(-.2, .2)
    plt.axis('square')
    plt.title(name)
    plt.show()


def get_bottom_full_asterisk(df, nbest=10):
    sorted_df = df.sort_values("total_distance")
    print(sorted_df["total_distance"].nsmallest(10))
    info = list(sorted_df.head(nbest).values)
    top_n_df = list(sorted_df.head(nbest)["name"].values)
    # top_n_df.flip()
    #np.flip(top_n_df, 0)
    print(top_n_df)
    return top_n_df, info


def get_top_full_asterisk(df, nbest=10):
    sorted_df = df.sort_values("total_distance")
    print(sorted_df["total_distance"].nlargest(10))
    info = list(sorted_df.tail(nbest).values)
    top_n_df = list(sorted_df.tail(nbest)["name"].values)
    top_n_df.reverse()
    # top_n_df.flip()
    #np.flip(top_n_df, 0)
    print(top_n_df)
    return top_n_df, info


def get_top_NS(df, nbest=10):
    df["NS_total_distance"] = df["N_max"] + df["S_max"]
    sorted_df = df.sort_values("NS_total_distance")
    print(sorted_df["NS_total_distance"].nlargest(10))
    info = list(sorted_df.tail(nbest).values)
    top_n_df = list(sorted_df.tail(nbest)["name"].values)
    top_n_df.reverse()
    # top_n_df.flip()
    #np.flip(top_n_df, 0)
    print(top_n_df)
    return top_n_df, info


def get_top_EW(df, nbest=10):
    df["EW_total_distance"] = df["E_max"] + df["W_max"]
    sorted_df = df.sort_values("EW_total_distance")
    print(sorted_df["EW_total_distance"].nlargest(10))
    info = list(sorted_df.tail(nbest).values)
    top_n_df = list(sorted_df.tail(nbest)["name"].values)
    top_n_df.reverse()
    # top_n_df.flip()
    #np.flip(top_n_df, 0)
    print(top_n_df)
    return top_n_df, info


def plot_total_areas(df):
    sorted_df = df.sort_values("total_area")
    df_2v2 = sorted_df[(sorted_df == "2v2").any(axis=1)]
    df_2v3 = sorted_df[(sorted_df == "2v3").any(axis=1)]
    df_3v3 = sorted_df[(sorted_df == "3v3").any(axis=1)]
    figure, axis = plt.subplots(1, 3)
    axis[0].scatter(np.arange(len(df_2v2.index)), df_2v2["total_area"])
    axis[0].set_title("Hand Approx. Workspace 2v2 (Projected)")
    axis[0].set_xlabel("Hands")
    axis[0].set_ylabel("Total Area (Meters)")
    axis[0].set_ylim(.0015, .018)
    axis[1].scatter(np.arange(len(df_2v3.index)), df_2v3["total_area"], color="maroon")
    axis[1].set_title("Hand Approx. Workspace 2v3 (Projected)")
    axis[1].set_xlabel("Hands")
    axis[1].set_ylabel("Total Area (Meters)")
    axis[1].set_ylim(.0015, .018)
    axis[2].scatter(np.arange(len(df_3v3.index)), df_3v3["total_area"], color="purple")
    axis[2].set_title("Hand Approx. Workspace 3v3 (Projected)")
    axis[2].set_xlabel("Hands")
    axis[2].set_ylabel("Total Area (Meters)")
    axis[2].set_ylim(.0015, .018)
    plt.show()


def plot_total_distances_EW(df):
    df["EW_total_distance"] = df["E_max"] + df["W_max"]
    sorted_df = df.sort_values("EW_total_distance")
    df_2v2 = sorted_df[(sorted_df == "2v2").any(axis=1)]
    df_2v3 = sorted_df[(sorted_df == "2v3").any(axis=1)]
    df_3v3 = sorted_df[(sorted_df == "3v3").any(axis=1)]
    figure, axis = plt.subplots(1, 3)
    axis[0].scatter(np.arange(len(df_2v2.index)), df_2v2["EW_total_distance"])
    axis[0].set_title("Hand Distances 2v2 EW (Projected)")
    axis[0].set_xlabel("Hands")
    axis[0].set_ylabel("Total Distance (Meters)")
    axis[0].set_ylim(.07, .21)
    axis[1].scatter(np.arange(len(df_2v3.index)), df_2v3["EW_total_distance"], color="maroon")
    axis[1].set_title("Hand Distances 2v3 EW (Projected)")
    axis[1].set_xlabel("Hands")
    axis[1].set_ylabel("Total Distance (Meters)")
    axis[1].set_ylim(.07, .21)
    axis[2].scatter(np.arange(len(df_3v3.index)), df_3v3["EW_total_distance"], color="purple")
    axis[2].set_title("Hand Distances 3v3 EW (Projected)")
    axis[2].set_xlabel("Hands")
    axis[2].set_ylabel("Total Distance (Meters)")
    axis[2].set_ylim(.07, .21)
    plt.show()


def plot_total_distances_NS(df):
    df["NS_total_distance"] = df["N_max"] + df["S_max"]
    sorted_df = df.sort_values("NS_total_distance")
    df_2v2 = sorted_df[(sorted_df == "2v2").any(axis=1)]
    df_2v3 = sorted_df[(sorted_df == "2v3").any(axis=1)]
    df_3v3 = sorted_df[(sorted_df == "3v3").any(axis=1)]
    figure, axis = plt.subplots(1, 3)
    axis[0].scatter(np.arange(len(df_2v2.index)), df_2v2["NS_total_distance"])
    axis[0].set_title("Hand Distances 2v2 NS (Projected)")
    axis[0].set_xlabel("Hands")
    axis[0].set_ylabel("Total Distance (Meters)")
    axis[0].set_ylim(.03, .15)
    axis[1].scatter(np.arange(len(df_2v3.index)), df_2v3["NS_total_distance"], color="maroon")
    axis[1].set_title("Hand Distances 2v3 NS (Projected)")
    axis[1].set_xlabel("Hands")
    axis[1].set_ylabel("Total Distance (Meters)")
    axis[1].set_ylim(.03, .15)
    axis[2].scatter(np.arange(len(df_3v3.index)), df_3v3["NS_total_distance"], color="purple")
    axis[2].set_title("Hand Distances 3v3 NS (Projected)")
    axis[2].set_xlabel("Hands")
    axis[2].set_ylabel("Total Distance (Meters)")
    axis[2].set_ylim(.03, .15)
    plt.show()


def plot_total_distances(df):
    sorted_df = df.sort_values("total_distance")
    df_2v2 = sorted_df[(sorted_df == "2v2").any(axis=1)]
    df_2v3 = sorted_df[(sorted_df == "2v3").any(axis=1)]
    df_3v3 = sorted_df[(sorted_df == "3v3").any(axis=1)]
    figure, axis = plt.subplots(1, 3)
    axis[0].scatter(np.arange(len(df_2v2.index)), df_2v2["total_distance"])
    axis[0].set_title("Hand Distances 2v2 (Projected)")
    axis[0].set_xlabel("Hands")
    axis[0].set_ylabel("Total Distance (Meters)")
    axis[0].set_ylim(.2, .65)
    axis[1].scatter(np.arange(len(df_2v3.index)), df_2v3["total_distance"], color="maroon")
    axis[1].set_title("Hand Distances 2v3  (Projected)")
    axis[1].set_xlabel("Hands")
    axis[1].set_ylabel("Total Distance (Meters)")
    axis[1].set_ylim(.2, .65)
    axis[2].scatter(np.arange(len(df_3v3.index)), df_3v3["total_distance"], color="purple")
    axis[2].set_title("Hand Distances 3v3  (Projected)")
    axis[2].set_xlabel("Hands")
    axis[2].set_ylabel("Total Distance (Meters)")
    axis[2].set_ylim(.2, .65)
    plt.show()


def load_all_paths(path_name):
    indexing = {"N": 0, "NE": 1, "E": 2, "SE": 3, "S": 4, "SW": 5, "W": 6, "NW": 7}
    current_path = str(pathlib.Path().resolve())
    paths = {}
    names = []
    for file in os.listdir(current_path + "/" + path_name):
        temp_path = current_path + "/" + path_name + "/" + str(file)
        names.append(str(file))
        paths[file] = {}
        for file_sub in os.listdir(temp_path):
            temp_path2 = temp_path + "/" + str(file_sub)
            prefix = temp_path2.split("/")
            prefix, t = prefix[-1].split("_", 1)
            paths[file][str(prefix)] = temp_path2

    # print("KEYS", paths.keys())
    # print(paths[list(paths.keys())[0]]["N"])
    return paths, names


def create_quickstats():
    paths, names = load_all_paths(path_name="data")
    # filter_late_contact(paths)
    indexing = {"N": 0, "NE": 1, "E": 2, "SE": 3, "S": 4, "SW": 5, "W": 6, "NW": 7}
    hand_quickstats = []
    for hand in paths:
        prefix = hand.split("_")
        hand_info = [hand, prefix[0], prefix[1], prefix[2], prefix[3], prefix[4]+"5"]
        total_distance = 0
        projected_points = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]
        for trial in list(paths[hand].keys()):
            with open(paths[hand][trial], 'rb') as f:
                trial_data = pkl.load(f)
                trial_data, x, y = filter_late_contact(trial_data)
                trial_data, x, y, max_distance, max_point_projected = filter_path_deviation(trial_data, trial, x, y)
                hand_info.append(max_distance)
                hand_info.append(max_point_projected)
                total_distance += max_distance
                projected_points[indexing[trial]] = max_point_projected
        pgon = Polygon(projected_points)
        hand_info.append(total_distance)
        hand_info.append(pgon.area)
        hand_quickstats.append(deepcopy(hand_info))
    df = pd.DataFrame(hand_quickstats,
                      columns=["name", "type", "f0", "f1", "f1:f0", "palm_width", "E_max", "E_point", "NE_max",
                               "NE_point", "NW_max", "NW_point", "N_max", "N_point", "SE_max", "SE_point", "SW_max",
                               "SW_point", "S_max", "S_point", "W_max", "W_point", "total_distance", "total_area"])
    df.to_pickle("quickstats.pkl")
    print(df)


if __name__ == "__main__":
    paths, names = load_all_paths(path_name="data")
    # create_quickstats()
    df = pd.read_pickle("quickstats.pkl")
    # df_2v2 = df[(df == "2v2").any(axis=1)]
    # df_2v3 = df[(df == "2v3").any(axis=1)]
    # df_3v3 = df[(df == "3v3").any(axis=1)]
    # print(len(df_2v2.index), len(df_2v3.index), len(df_3v3.index))

    plot_total_areas(df)
    # plot_total_distances(df)
    # plot_total_distances_NS(df)
    # plot_total_distances_EW(df)
    #best, info = get_top_full_asterisk(df)
    #best, info = get_bottom_full_asterisk(df)
    #best, info = get_top_EW(df)
    best, info = get_top_NS(df)

    # for i in range(len(info)):
    #     print(i, list(info[i]))

    for i in best:
        plot_asterisk(paths, i)
