#!/usr/bin/env python3

import math
from node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from Queue import PriorityQueue
from node import Node
from typing import Protocol, Dict, List, Iterator, Tuple, TypeVar, Optional


class Search:

    def __init__(self):
        # nodes_path = "../text/nodes.txt"
        # edges_path = "../text/edges.txt"
        nodes_path = "/home/ubuntu/mqp_ws/src/navigation/src/text/nodes.txt"
        edges_path = "/home/ubuntu/mqp_ws/src/navigation/src/text/edges.txt"
        self.nodes = self.load_nodes(nodes_path)
        self.load_edges(edges_path)

    def load_nodes(self, nodes_path):
        nodes = [[float(y) for y in x.strip().split(",")] for x in open(nodes_path, "r").readlines()]
        nodes = [Node(node) for node in nodes]
        return nodes
    
    def load_edges(self, edges_path):
        edges = [[float(y) for y in x.strip().split(",")] for x in open(edges_path, "r").readlines()]
        for edge in edges:
            c1 = [edge[0], edge[1]]
            c2 = [edge[2], edge[3]]
            n1 = self.get_node(c1)
            n2 = self.get_node(c2)
            n1.add_neighbor(n2)
            n2.add_neighbor(n1)

    def get_node(self, coord):
        for node in self.nodes:
            if node.is_same(coord):
                return node
    
    def get_distance(self, start, end):
        R = 6378.137
        lon_diff = end[0] * math.pi / 180 - start[0] * math.pi / 180
        lat_diff = end[1] * math.pi / 180 - start[1] * math.pi / 180
        a = math.sin(lat_diff/2) * math.sin(lat_diff/2) + math.cos(end[0] * math.pi / 180) * math.cos(end[1] * math.pi / 180) * math.sin(lon_diff/2) * math.sin(lon_diff/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R * c
        return d * 1000; # meters

    def get_closest_node(self, start):
        closest_node = None
        closest_distance = -1

        for node in self.nodes:
            coord = node.coord
            dist = self.get_distance(start, coord)
            if closest_distance == -1 or  dist < closest_distance:
                closest_node = node
                closest_distance = dist
        
        return closest_node

    def run(self, start, goal):
        start = self.get_closest_node(start)
        goal = self.get_node(goal)
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from: Dict[Node, Optional[Node]] = {}
        cost_so_far: Dict[Node, float] = {}
        came_from[start] = None
        cost_so_far[start] = 0
        
        while not frontier.empty():
            current: Node = frontier.get()
            if current == goal:
                break
            
            for next in current.get_neighbors():
                new_cost = cost_so_far[current] + current.dist_to_neighbor(next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + current.dist_to_neighbor(goal)
                    frontier.put(next, priority)
                    came_from[next] = current
        
        path = self.reconstruct_path(came_from, start, goal)
        return path

    def reconstruct_path(self, came_from: Dict[Node, Node],
                        start: Node, goal: Node) -> List[Node]:

        current: Node = goal
        path: List[Node] = []
        while current != start: # note: this will fail if no path found
            path.append(current)
            current = came_from[current]
        path.append(start) # optional
        path.reverse() # optional
        return path
