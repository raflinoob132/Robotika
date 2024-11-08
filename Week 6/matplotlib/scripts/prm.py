#!/usr/bin/env python3

import rospy
import numpy as np
import yaml
import matplotlib.pyplot as plt
from std_msgs.msg import String

class PRMPlanner:
    def __init__(self):
        # Load parameters
        self.start_pos = rospy.get_param("/prm/start_pos")
        self.goal_pos = rospy.get_param("/prm/goal_pos")
        self.num_nodes = rospy.get_param("/prm/num_nodes")
        self.k_nearest = rospy.get_param("/prm/k_nearest")
        self.max_edge_dist = rospy.get_param("/prm/max_edge_dist")
        self.x_range = rospy.get_param("/prm/x_range")
        self.y_range = rospy.get_param("/prm/y_range")
        self.obstacle_list = rospy.get_param("/prm/obstacle_list")

        # Node and edge data
        self.nodes = []
        self.edges = {}

    def is_collision(self, node1, node2):
        for (ox, oy, radius) in self.obstacle_list:
            closest_x = max(ox - radius, min(node1[0], node2[0], ox + radius))
            closest_y = max(oy - radius, min(node1[1], node2[1], oy + radius))
            dist_sq = (closest_x - ox)**2 + (closest_y - oy)**2
            if dist_sq <= radius**2:
                return True
        return False

    def generate_random_nodes(self):
        self.nodes = [tuple(self.start_pos), tuple(self.goal_pos)]
        while len(self.nodes) < self.num_nodes:
            x = np.random.uniform(self.x_range[0], self.x_range[1])
            y = np.random.uniform(self.y_range[0], self.y_range[1])
            if not any(np.linalg.norm(np.array([x, y]) - np.array(obs[:2])) <= obs[2] for obs in self.obstacle_list):
                self.nodes.append((x, y))
        rospy.loginfo("Nodes generated: %d" % len(self.nodes))

    def create_edges(self):
        for node in self.nodes:
            distances = [(np.linalg.norm(np.array(node) - np.array(other)), other) for other in self.nodes if node != other]
            distances.sort(key=lambda x: x[0])
            neighbors = [n for d, n in distances[:self.k_nearest] if d <= self.max_edge_dist and not self.is_collision(node, n)]
            self.edges[node] = neighbors
        rospy.loginfo("Edges created")

    def dijkstra(self):
        start_pos = tuple(self.start_pos)
        goal_pos = tuple(self.goal_pos)
        unvisited = {node: float('inf') for node in self.nodes}
        unvisited[start_pos] = 0
        previous_nodes = {}
        path = []

        while unvisited:
            current_node = min(unvisited, key=unvisited.get)
            current_distance = unvisited[current_node]
            del unvisited[current_node]

            if current_node == goal_pos:
                while current_node in previous_nodes:
                    path.insert(0, current_node)
                    current_node = previous_nodes[current_node]
                path.insert(0, start_pos)
                break

            for neighbor in self.edges.get(current_node, []):
                distance = np.linalg.norm(np.array(current_node) - np.array(neighbor))
                new_distance = current_distance + distance
                if neighbor in unvisited and new_distance < unvisited[neighbor]:
                    unvisited[neighbor] = new_distance
                    previous_nodes[neighbor] = current_node

        return path

    def visualize(self, path):
        fig, ax = plt.subplots()
        for (ox, oy, radius) in self.obstacle_list:
            circle = plt.Circle((ox, oy), radius, color="red", alpha=0.5)
            ax.add_patch(circle)

        for node, neighbors in self.edges.items():
            for neighbor in neighbors:
                plt.plot([node[0], neighbor[0]], [node[1], neighbor[1]], "gray", linestyle="--")

        for node in self.nodes:
            plt.plot(node[0], node[1], "bo", markersize=3)

        if path:
            plt.plot([p[0] for p in path], [p[1] for p in path], "g-", linewidth=2, label="Path")
        plt.plot(self.start_pos[0], self.start_pos[1], "go", markersize=10, label="Start")
        plt.plot(self.goal_pos[0], self.goal_pos[1], "ro", markersize=10, label="Goal")

        plt.xlim(self.x_range)
        plt.ylim(self.y_range)
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.title("Probabilistic Roadmap (PRM)")
        plt.show()

    def run(self):
        rospy.loginfo("Generating nodes...")
        self.generate_random_nodes()
        rospy.loginfo("Creating edges...")
        self.create_edges()
        rospy.loginfo("Finding shortest path...")
        path = self.dijkstra()
        if path:
            rospy.loginfo("Path found!")
            self.visualize(path)
        else:
            rospy.logwarn("No path found")

def main():
    rospy.init_node('prm_node')
    prm_planner = PRMPlanner()
    prm_planner.run()

if __name__ == '__main__':
    main()
