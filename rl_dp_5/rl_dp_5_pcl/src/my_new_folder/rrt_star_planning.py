#!/usr/bin/env python3
import rospy
import numpy as np
import random
from scipy.spatial import KDTree
from shapely.geometry import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import splprep, splev

waypoints = []

def path_callback(msg):
    global waypoints
    all_points = msg.poses  
    if len(all_points) > 2:
        waypoints = [(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in all_points[1:-1]]  # Include z
        waypoints = np.array(waypoints)
        print(f"Waypoint shape: {waypoints.shape}")  # Should be (n, 3)

    rospy.loginfo(f"Received {len(waypoints)} waypoints after filtering.")
    rospy.loginfo(f"Waypoints before RRT*: {waypoints}")

    
class RRTStar:
    def __init__(self, start, goal, obstacles, step_size=0.5, max_iter=1000):
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.tree = {self.start: None}

    def distance(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def nearest_node(self, rand_point):
        tree = KDTree(self.nodes)
        _, idx = tree.query(rand_point)
        return self.nodes[idx]

    def steer(self, from_node, to_node):
        direction = np.array(to_node) - np.array(from_node)
        distance = np.linalg.norm(direction)
        direction = direction / distance
        new_node = tuple(from_node + direction * min(self.step_size, distance))
        return new_node

    def is_collision_free(self, new_point):
        point = Point(new_point)
        for obs in self.obstacles:
            if obs.contains(point):
                return False
        return True

    def plan(self):
        for _ in range(self.max_iter):
            rand_point = (random.uniform(0, 10), random.uniform(0, 10), random.uniform(0, 10))
            nearest = self.nearest_node(rand_point)
            new_node = self.steer(nearest, rand_point)

            if self.is_collision_free(new_node):
                self.nodes.append(new_node)
                self.tree[new_node] = nearest

                if self.distance(new_node, self.goal) < self.step_size:
                    self.tree[self.goal] = new_node
                    break

        path = [self.goal]
        node = self.goal
        while node in self.tree and self.tree[node]:
            node = self.tree[node]
            path.append(node)
        return path[::-1]  # Reverse the path

from scipy.interpolate import splprep, splev
import numpy as np

from scipy.interpolate import splprep, splev

def smooth_path(waypoints):
    waypoints = np.array(waypoints)  # Ensure NumPy array
    if waypoints.shape[1] != 3:  # Check dimensions
        raise ValueError("Waypoints must have (x, y, z) format.")

    tck, u = splprep([waypoints[:, 0], waypoints[:, 1], waypoints[:, 2]], k=3, s=0.5)
    u_fine = np.linspace(0, 1, len(waypoints) * 5)  # More points for smoothness
    smoothed = np.array(splev(u_fine, tck)).T  # Interpolate
    return smoothed


def plan_rrt():
    rospy.init_node("rrt_star_planner", anonymous=True)
    rospy.Subscriber("/path", Path, path_callback)
    rospy.loginfo("Waiting for waypoints...")
    
    while len(waypoints) == 0:  # Wait until waypoints are updated
        rospy.sleep(0.5)

    if len(waypoints) < 2:
        rospy.logwarn("Not enough waypoints received. Exiting...")
        return

    rospy.loginfo(f"Received {len(waypoints)} waypoints. Planning now...")

    obstacles = [Point(5, 5).buffer(1.5), Point(8, 5).buffer(1.0)]
    planned_path = []

    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        goal = waypoints[i + 1]
        rospy.loginfo(f"Planning from {start} to {goal}")
        rrt_star = RRTStar(start, goal, obstacles)
        sub_path = rrt_star.plan()
        planned_path.extend(sub_path)
        dist = np.linalg.norm(waypoints[i + 1] - waypoints[i])
        print(f"Distance between Waypoint {i+1} and {i+2}: {dist:.4f} meters")


    rospy.loginfo(f"Raw Planned Path: {planned_path}")
    smoothed_path = smooth_path(planned_path)
    rospy.loginfo(f"Smoothed Path: {smoothed_path}")

    return smoothed_path

if __name__ == "__main__":
    plan_rrt()

