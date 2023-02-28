import numpy as np
import os
import pathlib
import pickle as pkl
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from skspatial.objects import Line, Point
import pandas as pd
import math
import json
from sklearn.linear_model import LinearRegression
import random

from copy import deepcopy


def NormalizeData(data):
    return (data - np.min(data)) / (np.max(data) - np.min(data))


if __name__ == "__main__":
    f = open("hand_descriptions_test.json")
    data = json.load(f)
    f = open("hand_descriptions_batch_2v2_1.1.json")
    data2 = json.load(f)
    df = pd.read_pickle("quickstats.pkl")

    l0 = []
    l1 = []
    l2 = []
    l3 = []
    palm = []
    ratios_list = []
    distances = []
    name = []
    for i in data:
        if i["sim"]["finger1"]["num_links"] == 2 and i["sim"]["finger2"]["num_links"] == 2:
            # if float(df.loc[df["name"] == i["name"]]["palm_width"].values[0]) == 535:
            l0.append(float(i["sim"]["finger1"]["link_lengths"][0][1]*100))
            l1.append(float(i["sim"]["finger1"]["link_lengths"][1][1]*100))
            l2.append(float(i["sim"]["finger2"]["link_lengths"][0][1]*100))
            l3.append(float(i["sim"]["finger2"]["link_lengths"][1][1]*100))
            distances.append(float(df.loc[df["name"] == i["name"]]["total_distance"].values[0]*100))
            palm.append(float(df.loc[df["name"] == i["name"]]["palm_width"].values[0]))
            name.append(i["name"])
            if distances[-1] < 35:
                print(i["name"])

    for i in data2:
        if i["sim"]["finger1"]["num_links"] == 2 and i["sim"]["finger2"]["num_links"] == 2:
            # if float(df.loc[df["name"] == i["name"]]["palm_width"].values[0]) == 535:
            l0.append(float(i["sim"]["finger1"]["link_lengths"][0][1]*100))
            l1.append(float(i["sim"]["finger1"]["link_lengths"][1][1]*100))
            l2.append(float(i["sim"]["finger2"]["link_lengths"][0][1]*100))
            l3.append(float(i["sim"]["finger2"]["link_lengths"][1][1]*100))
            distances.append(float(df.loc[df["name"] == i["name"]]["total_distance"].values[0]*100))
            name.append(i["name"])
            palm.append(float(df.loc[df["name"] == i["name"]]["palm_width"].values[0]))

    # plt.scatter(l0, distances)
    # plt.show()
    # plt.scatter(l1, distances)
    # plt.show()
    # plt.scatter(l2, distances)
    # plt.show()
    # plt.scatter(l3, distances)
    # plt.show()
    # plt.scatter(palm, distances)
    # plt.show()

    # plt.scatter(l3, distances)

    print(len(l1))
    print(len(distances))

    combined = []
    colors = []
    for i in range(len(l0)):
        combined.append((l1[i] + l3[i]) / (l2[i] + l0[i]))
        #combined.append((l1[i]/l2[i]) / (l0[i]/l3[i]))
        # combined.append((l1[i]/l0[i]) * (l2[i]/l3[i]))
        # combined.append(((l1[i] + l3[i])/2) / ((l2[i] + l0[i])/2))
        # combined.append((l1[i]/l0[i]) + (l3[i]/l2[i]))
        # colors.append(((l1[i]/l2[i]) / (l3[i]/l0[i])))
        # colors.append(min((l1[i]/l0[i]) * (l2[i]/l3[i]), (l3[i]/l2[i]) / (l1[i]/l0[i])))
        if combined[-1] < .5 and distances[i] < 40:
            print("HERe", name[i], l0[i], l1[i], l2[i], l3[i])
        # colors.append((l1[i] + l3[i]) / 2)
        # colors.append((min(l0[i]/l1[i], l1[i]/l0[i]) + min(l2[i]/l3[i], l3[i]/l2[i])) / 2)
        # colors.append((max(1, max(l0[i]-l1[i], l1[i]-l0[i])) * max(1, (max(l2[i]-l3[i], l3[i]-l2[i])))))
        # colors.append(max(l0[i]/l1[i], l1[i]/l0[i], l3[i]/l2[i], l2[i]/l3[i]))
        #colors.append(max(l0[i]/l1[i], l1[i]/l0[i], l3[i]/l2[i], l2[i]/l3[i]))
        colors.append((max(l0[i]/l1[i], l1[i]/l0[i]) + max(l3[i]/l2[i], l2[i]/l3[i]))/2)
        #colors.append((max(l0[i]/l1[i], l1[i]/l0[i]) + max(l3[i]/l2[i], l2[i]/l3[i])))
        # colors.append((max(l1[i]/l0[i], l3[i]/l2[i]) / min(l0[i]/l1[i], l2[i]/l3[i])))
        # colors.append((max(l1[i], l3[i])))
        # colors.append((l1[i] / l2[i]) + (l3[i] / l0[i]))
        # colors.append(((l1[i]/l2[i]) / (l0[i]/l3[i])) * ((l1[i]/l0[i]) + (l3[i]/l2[i])))
        # colors.append((l1[i]/l0[i]) * (l2[i]/l3[i]))
        # colors.append(((l1[i]/l2[i]) / (l0[i]/l3[i])) * ((l1[i]/l0[i]) + (l3[i]/l2[i])))
        # colors.append((min(l0[i]/l1[i], l1[i]/l0[i]) + min(l2[i]/l3[i], l3[i]/l2[i])) / 2)
        # colors.append((l0[i]/l1[i] + l2[i]/l3[i]) / 2)
        # colors.append((l1[i] -  + l3[i]/(l3[i]+l2[i])) / 2)
    plt.scatter(combined, distances)
    plt.show()

    colors = NormalizeData(np.array(colors))
    print(colors)

    combined_keep = deepcopy(combined)
    distances_keep = deepcopy(distances)
    z = list(zip(combined, distances))
    random.shuffle(z)
    random.shuffle(z)
    random.shuffle(z)
    combined, distances = zip(*z)

    train_combined = list(combined[:len(combined)//2])
    train_distances = list(distances[:len(distances)//2])
    test_combined = list(combined[len(combined)//2:])
    test_distances = list(distances[len(distances)//2:])

    test_distances.pop(3)
    test_combined.pop(3)
    print(len(train_combined))
    print(len(test_combined))

    # plt.scatter(l1, distances)
    # plt.scatter(l3, distances)
    # plt.scatter(l0, distances)
    # plt.scatter(np.reshape(combined, (-1, 1)), np.reshape(distances, (-1, 1)))
    # plt.scatter(combined, distances)
    model = LinearRegression()
    model.fit(np.reshape(train_combined, (-1, 1)),  np.reshape(train_distances, (-1, 1)))
    pred = model.predict(np.reshape(test_combined, (-1, 1)))
    from sklearn.metrics import mean_squared_error, r2_score
    print("Coefficients: \n", model.coef_)
    # The mean squared error
    print("Mean squared error: %.2f" % mean_squared_error(test_distances, pred))
    # The coefficient of determination: 1 is perfect prediction
    print("Coefficient of determination: %.2f" % r2_score(test_distances, pred))
    # from sklearn.feature_selection import chi2
    # print(train_combined)
    # print(test_distances)
    # scores, pvalues = chi2(
    #     np.array(train_combined).astype("float"),
    #     np.reshape(test_distances, (-1, 1)).astype("float"))
    # print(pvalues)
    # plt.scatter(palm, distances_keep)
    # plt.show()
    print(len(combined_keep))
    for i in range(len(combined_keep)):
        if palm[i] == 535:
            plt.scatter(combined_keep[i], distances_keep[i], c=colors[i], cmap="coolwarm", vmin=0, vmax=1, marker="^")
        if palm[i] == 635:
            plt.scatter(combined_keep[i], distances_keep[i], c=colors[i], cmap="coolwarm", vmin=0, vmax=1, marker="s")
        if palm[i] == 735:
            plt.scatter(combined_keep[i], distances_keep[i], c=colors[i], cmap="coolwarm", vmin=0, vmax=1, marker="o")
    plt.plot(test_combined, pred, color="red", linewidth=3)
    plt.show()

    import statsmodels.api as sma

    X2 = sma.add_constant(combined_keep)
    b = sma.OLS(distances_keep, X2)
    d = b.fit()
    print(d.summary())
    print(d.pvalues[0])
    print(d.pvalues[1])
    plt.show()
