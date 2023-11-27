import numpy as np
from scipy.ndimage.morphology import distance_transform_edt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from obstacles import *

class TrajectoryOptimizer():

    def __init__(self, obstacles):
        self.obstacles = obstacles

    def obstacles_gradient(self, point):
        ''' Computes the gradient for a point and any obstacles it is near
        Used to push points away when inside an obstacle, and attract them towards
        the edge of obstacles.
        '''
        grad = np.array([0., 0., 0.])
        for obstacle in self.obstacles:
            if isinstance(obstacle, Circle):
                # cost based on radius and distance
                # d = pi r^2

                # TODO: Normalize this, because gradient is very high at the edge of shapes
                dx = obstacle.x - point[0]
                dy = obstacle.y - point[1]
                if np.sqrt(dx**2 + dy**2) < obstacle.radius:
                    grad[:] += [2*dx, 2*dy, 0]

            if isinstance(obstacle, Square):
                # cost based on radius and distance
                raise KeyError("Rectangle not supported in obstacle gradient yet!")
        return grad

    def smoothness_gradient(self, trajectory):
        '''
        Returns the gradient of a symmetric smoothing cost function.
        '''

        smoothness_grad2 = trajectory[0:] - np.concatenate([[trajectory[0]], trajectory[0:-1]]) + \
            trajectory[0:] - np.concatenate([trajectory[1:], [trajectory[-1]]])
        return smoothness_grad2
    
    def constant_angle_gradient(self, trajectory):
        smooth_angle_grad = trajectory[0:,-1] - np.concatenate([[trajectory[0]], trajectory[0:-1]])[:,-1] + \
            trajectory[0:, -1] - np.concatenate([trajectory[1:], [trajectory[-1]]])[:,-1]
        grad = self.smoothness_gradient(trajectory)
        grad[:,0:2] *= 0
        print("grad:", grad)

        return grad


    def optimize(self, path, cost_fun):
        '''
        path: Trajectory

        Returns:
            Trajectory

        '''
        # Keep initial point and end point fixed
        new_path = path.array.copy()
        lr = 0.1
        # lr = 1
        prev_total_cost = 999999
        for i in range(50):
            # Old code using a costmap as the cost_fun
            # gx, gy = np.gradient(cost_fun)
            # path_indices = np.floor(new_path).astype(np.int32)
            # path_gradient_x = gx[path_indices[:,0], path_indices[:,1]]
            # path_gradient_y = gy[path_indices[:,0], path_indices[:,1]]
            # grad = np.stack([path_gradient_x, path_gradient_y],axis=1)

            obstacle_grads = []
            for point in new_path:
                obstacle_grad = self.obstacles_gradient(point)
                obstacle_grads.append(obstacle_grad)
            obstacle_grads = np.array(obstacle_grads)

            smoothness_grad = self.smoothness_gradient(new_path)
            angle_grad = self.constant_angle_gradient(new_path)

            total_grad = 1.0 * obstacle_grads + 4 * smoothness_grad + 4 * angle_grad

            # total_grad = 0.8 * grad + 4 * smoothness_grad2

            new_path[1:-2] = new_path[1:-2] -  lr * total_grad[1:-2]

            # Check convergence
            # new_path_indices = np.floor(new_path).astype(np.int32)

            # new_path_cost = cost_fun[new_path_indices[:,0], new_path_indices[:,1]]
            # total_cost = np.sum(new_path_cost)
            # print("total_cost:", total_cost)
            # dcost = prev_total_cost - total_cost
            # prev_total_cost = total_cost
            # print("dcost:", dcost)

            # if dcost < 0.2:
            #     break

        return new_path
