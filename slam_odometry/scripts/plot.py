#!/usr/bin/env python2
import csv
import matplotlib.pylab as plt

# GLOBAL VARIABLES
OUTPUT_FILE = "/tmp/odometry.dat"


def load_file(fp, skip_header=True):
    csv_file = open(fp, 'r')
    csv_reader = csv.reader(csv_file)
    if skip_header:
        next(csv_reader, None)

    data = {
        "i": [],
        "x": [], "y": [], "z": [],
        "yaw": []
    }
    for line in csv_reader:
        data["i"].append(float(line[0]))
        data["x"].append(float(line[1]))
        data["y"].append(float(line[2]))
        data["z"].append(float(line[3]))
        data["yaw"].append(float(line[4]))

    return data


def plot_odometry():
    data = load_file(OUTPUT_FILE)
    plt.plot(data["i"], data["x"], label="x")
    plt.plot(data["i"], data["y"], label="y")
    plt.plot(data["i"], data["z"], label="z")
    plt.plot(data["i"], data["yaw"], label="yaw")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    plot_odometry()
