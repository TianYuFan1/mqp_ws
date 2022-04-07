#!/usr/bin/env python3

import math

class Node:

    def __init__(self, coord):
        self.coord = coord
        self.neighbors = []
    
    def get_coord(self):
        return self.coord

    def get_neighbors(self):
        return self.neighbors

    def add_neighbor(self, n):
        self.neighbors.append(n)    
    
    def dist_to_neighbor(self, n):
        start = self.coord 
        end = n.coord
        R = 6378.137
        lon_diff = end[0] * math.pi / 180 - start[0] * math.pi / 180
        lat_diff = end[1] * math.pi / 180 - start[1] * math.pi / 180
        a = math.sin(lat_diff/2) * math.sin(lat_diff/2) + math.cos(end[0] * math.pi / 180) * math.cos(end[1] * math.pi / 180) * math.sin(lon_diff/2) * math.sin(lon_diff/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R * c
        return d * 1000; # meters

    def is_same(self, coord):
        return self.coord[0] == coord[0] and self.coord[1] == coord[1]
    