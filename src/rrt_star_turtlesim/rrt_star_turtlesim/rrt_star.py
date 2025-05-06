import numpy as np
import math
from typing import List, Tuple, Optional

class Node:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = float('inf')
        self.children = []

class RRTStar:
    def __init__(self, 
                 start: Tuple[float, float],
                 goal: Tuple[float, float],
                 bounds: Tuple[Tuple[float, float], Tuple[float, float]],
                 obstacles: List[Tuple[float, float, float]],  # (x, y, radius)
                 max_iter: int = 1000,
                 step_size: float = 0.5,
                 goal_sample_rate: float = 0.1,
                 search_radius: float = 1.0):
        
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.bounds = bounds
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        
        self.nodes = [self.start]
        self.start.cost = 0.0

    def is_collision_free(self, node: Node) -> bool:
        for obs in self.obstacles:
            dist = math.sqrt((node.x - obs[0])**2 + (node.y - obs[1])**2)
            if dist < obs[2]:
                return False
        return True

    def get_random_point(self) -> Tuple[float, float]:
        if np.random.random() < self.goal_sample_rate:
            return self.goal.x, self.goal.y
        
        x = np.random.uniform(self.bounds[0][0], self.bounds[0][1])
        y = np.random.uniform(self.bounds[1][0], self.bounds[1][1])
        return x, y

    def find_nearest_node(self, point: Tuple[float, float]) -> Node:
        min_dist = float('inf')
        nearest_node = None
        
        for node in self.nodes:
            dist = math.sqrt((node.x - point[0])**2 + (node.y - point[1])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
                
        return nearest_node

    def steer(self, from_node: Node, to_point: Tuple[float, float]) -> Node:
        dist = math.sqrt((from_node.x - to_point[0])**2 + (from_node.y - to_point[1])**2)
        if dist < self.step_size:
            return Node(to_point[0], to_point[1])
        
        theta = math.atan2(to_point[1] - from_node.y, to_point[0] - from_node.x)
        new_x = from_node.x + self.step_size * math.cos(theta)
        new_y = from_node.y + self.step_size * math.sin(theta)
        return Node(new_x, new_y)

    def find_near_nodes(self, new_node: Node) -> List[Node]:
        near_nodes = []
        for node in self.nodes:
            dist = math.sqrt((node.x - new_node.x)**2 + (node.y - new_node.y)**2)
            if dist < self.search_radius:
                near_nodes.append(node)
        return near_nodes

    def rewire(self, new_node: Node, near_nodes: List[Node]):
        for near_node in near_nodes:
            edge_cost = math.sqrt((new_node.x - near_node.x)**2 + (new_node.y - near_node.y)**2)
            new_cost = new_node.cost + edge_cost
            
            if new_cost < near_node.cost:
                near_node.parent = new_node
                near_node.cost = new_cost
                new_node.children.append(near_node)

    def plan(self) -> Optional[List[Tuple[float, float]]]:
        for _ in range(self.max_iter):
            # Sample random point
            random_point = self.get_random_point()
            
            # Find nearest node
            nearest_node = self.find_nearest_node(random_point)
            
            # Steer towards random point
            new_node = self.steer(nearest_node, random_point)
            
            if not self.is_collision_free(new_node):
                continue
                
            # Find near nodes
            near_nodes = self.find_near_nodes(new_node)
            
            # Find minimum cost parent
            min_cost = float('inf')
            best_parent = None
            
            for near_node in near_nodes:
                edge_cost = math.sqrt((new_node.x - near_node.x)**2 + (new_node.y - near_node.y)**2)
                cost = near_node.cost + edge_cost
                
                if cost < min_cost:
                    min_cost = cost
                    best_parent = near_node
            
            if best_parent is None:
                continue
                
            new_node.parent = best_parent
            new_node.cost = min_cost
            best_parent.children.append(new_node)
            
            # Rewire
            self.rewire(new_node, near_nodes)
            
            self.nodes.append(new_node)
            
            # Check if goal is reached
            if math.sqrt((new_node.x - self.goal.x)**2 + (new_node.y - self.goal.y)**2) < self.step_size:
                self.goal.parent = new_node
                self.goal.cost = new_node.cost + math.sqrt((new_node.x - self.goal.x)**2 + (new_node.y - self.goal.y)**2)
                return self.get_path()
                
        return None

    def get_path(self) -> List[Tuple[float, float]]:
        path = []
        current = self.goal
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
            
        return path[::-1] 