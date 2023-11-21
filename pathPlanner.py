# Plans a path between waypoints using A* search algorithm
# Only finds good positions, then assumes the rake angle faces towards the next point sequentially

from astar import AStar
import numpy as np

class PathPlanner(AStar):
    def __init__(self, costmap):
        self.costmap = costmap
        self.max_x, self.max_y = self.costmap.shape[0:]

    def plan_path(self, waypoints):
        full_path = []
        for i in range(len(waypoints) - 1):
            w0 = tuple(waypoints[i])
            w1 = tuple(waypoints[i+1])
            leg = self.astar(w0, w1)
            if leg is None:
                raise Exception('You ain\'t got no legs, Lieutenant Dan!')
            for toe in leg:
                full_path.append(toe)
        return full_path

    def plan_leg(self, waypoint_A, waypoint_B):
        foundpath = self.astar(waypoint_A, waypoint_B)
        if foundpath is None:
            raise Exception('You ain\'t got no foundpath, Lieutenant Dan!')
        return list(foundpath)

    def neighbors(self, node):
        # return 8-neighbors of a cell
        neighbors = [
            (node[0] - 1, node[1] - 1),
            (node[0] - 1, node[1]),
            (node[0] - 1, node[1] + 1),
            (node[0], node[1] - 1),
            (node[0], node[1] + 1),
            (node[0] + 1, node[1] - 1),
            (node[0] + 1, node[1]),
            (node[0] + 1, node[1] + 1),
        ]
        to_remove = []
        for n in neighbors:
            if n[0] < 0 or n[0] >= self.max_x:
                to_remove.append(n)
                continue
            if n[1] < 0 or n[1] >= self.max_y:
                to_remove.append(n)
                
        for n in to_remove:
            neighbors.remove(n)

        return neighbors

    def distance_between(self, n1, n2):
        # d = abs(n1[0] - n2[0]) + abs(n1[1] - n2[1])
        d = (n1[0] - n2[0])**2 + (n1[1] - n2[1])**2
        return 1

    def heuristic_cost_estimate(self, current, goal):
        #cost = self.costmap[current[0], current[1]] + self.distance_between(current, goal)
        cost = self.costmap[current[0], current[1]]# + self.distance_between(current, goal)
        print(f"cost at {current[0]},{current[1]}: {cost}")
        return cost

    def is_goal_reached(self, current, goal):
        if current[0] == goal[0] and current[1] == goal[1]:
            return True
        return False

class MapNode():
    def __init__(self):
        pass

# class SandPlan(AStar):
#     def __init__(self, costmap):
#         self.costmap = costmap
