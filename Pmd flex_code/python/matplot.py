#!/usr/bin/python3

import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import queue

class MyListener:
    def __init__(self, q):
        self.queue = q
        self.point_cloud = []

    def onNewData(self, data):
        self.queue.put(data)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=60, help="Capture duration in seconds")
    options = parser.parse_args()

    q = queue.Queue()
    listener = MyListener(q)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Simulating real-time point cloud capture (replace this with your actual data source)
    for _ in range(options.duration):
        point_cloud = np.random.rand(100, 3) * 10  # Simulated point cloud data
        listener.onNewData(point_cloud)

        # Plot the point cloud
        ax.clear()
        ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], c='b', marker='o')
        plt.draw()
        plt.pause(0.1)

    # Get user input for selected point (replace this with your actual point selection logic)
    selected_index = int(input("Enter the index of the selected point: "))
    selected_point = point_cloud[selected_index]

    # Simulated waypoint calculation (replace this with your actual waypoint calculation)
    waypoints = [
        selected_point,
        selected_point + np.array([1, 1, 1]),
        selected_point - np.array([1, 1, 1]),
    ]

    # Visualize waypoints using Matplotlib
    visualize_waypoints(waypoints)

def visualize_waypoints(waypoints):
    waypoints = np.array(waypoints)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], c='r', marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

if __name__ == "__main__":
    main()
