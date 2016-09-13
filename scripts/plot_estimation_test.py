#!/usr/bin/env python2
import csv

import matplotlib.pylab as plt


def load_file(fp, skip_header=True):
    csv_file = open(fp, 'r')
    csv_reader = csv.reader(csv_file)
    if skip_header:
        next(csv_reader, None)

    data = {
        "i": [],
        "x": [], "y": [], "z": [],
        "bx": [], "by": [], "bz": []
    }
    for line in csv_reader:
        data["i"].append(float(line[0]))
        data["x"].append(float(line[1]))
        data["y"].append(float(line[2]))
        data["z"].append(float(line[3]))
        data["bx"].append(float(line[4]))
        data["by"].append(float(line[5]))
        data["bz"].append(float(line[6]))

    return data


if __name__ == "__main__":
    data = load_file("/tmp/estimation_test.output")

    plt.plot(data["x"], data["y"], label="Ground State")
    plt.plot(data["bx"], data["by"], marker="o", label="Belief State")
    plt.legend()
    plt.show()
