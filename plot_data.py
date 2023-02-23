import numpy as np
import os
import pathlib
import pickle as pkl
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from skspatial.objects import Line, Point
import math

from copy import deepcopy


def get_xys(paths, names, directions=None):
    final_xys = []
    final_path_names = []

    for i in names:
        xys = []
        path_directions = []
        for j in range(len(paths[i])):
            with open(paths[i][j], 'rb') as f:
                prefix = paths[i][j].split("/")
                prefix, t = prefix[-1].split("_", 1)
                path_directions.append(prefix)
                trial = pkl.load(f)
                x = []
                y = []
                for k in trial:
                    x.append(k["obj_pos"][0])
                    y.append(k["obj_pos"][1])
                xys.append([x, y])
        final_xys.append(deepcopy(xys))
        final_path_names.append(path_directions)
    return final_xys, final_path_names


def get_total_distance(paths, names, xys, path_directions):
    colors = ["blue", "red", "green", "olive", "purple", "cyan", "orange", "deeppink"]
    indexing = {"N": 0, "NE": 1, "E": 2, "SE": 3, "S": 4, "SW": 5, "W": 6, "NW": 7}
    vectors = [[0, .2], [.2, .2], [.2, 0], [.2, -.2], [0, -.2], [-.2, -.2], [-.2, 0], [-.2, .2]]
    projected_points_all = []
    distances_all = []
    distance_totals = []
    for i in range(len(xys)):
        distances = []
        projected_points = [0, 0, 0, 0, 0, 0, 0, 0]
        for j in range(len(path_directions[i])):
            last_x = xys[i][j][0][-1]
            last_y = xys[i][j][1][-1]
            point = Point([last_x, last_y])
            line = Line(point=[0, 0], direction=vectors[indexing[path_directions[i][j]]])
            point_projected = line.project_point(point)
            projected_points[indexing[path_directions[i][j]]] = [point_projected[0], point_projected[1]]
            distance_travelled = math.sqrt(point_projected[0]**2 + point_projected[1]**2)
            distances.append(distance_travelled)
        projected_points_all.append(projected_points)
        distances_all.append(deepcopy(distances))
        distance_totals.append(sum(distances))

    return projected_points_all, distances_all, distance_totals


def get_best_distance(paths, names, xys, distance_totals, projected_points, path_directions, nbest):
    print(distance_totals)
    for i in range(len(distance_totals)):
        print(names[i], distance_totals[i])

    areas = []
    for i in range(len(projected_points)):
        pgon = Polygon(projected_points[i])  # Assuming the OP's x,y coordinates
        areas.append(pgon.area)

    d = np.array(distance_totals)
    n = np.array(names)
    inds = d.argsort()
    sorted_names = n[inds]
    sorted_distances = d[inds]
    areas = np.array(areas)
    inds_a = areas.argsort()
    sorted_areas = areas[inds_a]
    sorted_names_areas = n[inds_a]
    print(sorted_distances[-nbest:])
    print(sorted_names[-nbest:])
    print(sorted_names_areas[-nbest:])
    print(sorted_areas[-nbest:])

    plt.scatter(sorted_names, sorted_distances)
    plt.title("Total Distances (Projected)")
    plt.xlabel("Hands")
    plt.ylabel("Total Distance (Meters)")
    plt.xticks(rotation='vertical')
    plt.autoscale()
    plt.show()

    plt.scatter(sorted_names_areas, sorted_areas, color="maroon")
    plt.title("Total Approx. Workspace Area")
    plt.xlabel("Hands")
    plt.ylabel("Total Area (Meters)")
    plt.xticks(rotation='vertical')
    plt.autoscale()
    plt.show()
    nbest_xys = []
    nbest_name = []
    nbest_projected = []
    for i in range(1, nbest+1):
        nbest_xys.append(xys[inds[-i]])
        nbest_projected.append(projected_points[inds[-i]])
        nbest_name.append(sorted_names[-i])

    print(nbest_name)
    plot_asterisk(paths, nbest_name, nbest_xys, path_directions, nbest_projected)


def plot_asterisk(paths, names, xys, path_directions, projected_points):
    colors = ["blue", "red", "green", "olive", "purple", "cyan", "orange", "deeppink"]
    indexing = {"N": 0, "NE": 1, "E": 2, "SE": 3, "S": 4, "SW": 5, "W": 6, "NW": 7}

    for i in range(len(path_directions)):
        for j in range(len(path_directions[i])):
            plt.plot(xys[i][j][0], xys[i][j][1], linewidth=3, color=colors[indexing[path_directions[i][j]]])
        pgon = Polygon(projected_points[i])  # Assuming the OP's x,y coordinates
        print("POLYGON AREA = ", pgon.area)
        plt.fill(np.array(projected_points[i])[:, 0], np.array(projected_points[i])[:, 1], alpha=.2)
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
        plt.title(names[i])
        plt.show()


def load_all_paths():
    current_path = str(pathlib.Path().resolve())
    paths = {}
    names = []
    for file in os.listdir(current_path + "/data"):
        temp_path = current_path + "/data/" + str(file)
        names.append(str(file))
        paths[file] = []
        for file_sub in os.listdir(temp_path):
            temp_path2 = temp_path + "/" + str(file_sub)
            paths[file].append(temp_path2)

    return paths, names


if __name__ == "__main__":
    paths, names = load_all_paths()
    test = [names[0], names[20]]
    xys, path_directions = get_xys(paths, names)
    projected_points, distances, distance_totals = get_total_distance(paths, names, xys, path_directions)
    #plot_asterisk(paths, test, xys, path_directions, projected_points)
    get_best_distance(paths, names, xys, distance_totals, projected_points, path_directions, 3)

    #plot_asterisk(paths, test, xys, path_directions, projected_points)

    pass
