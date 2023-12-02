import numpy as np
from scipy.ndimage.morphology import distance_transform_edt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from obstacles import *
from trajectory import *
from tqdm import tqdm

class TrajectoryOptimizer():

    def __init__(self, obstacles, rake_width):
        self.obstacles = obstacles
        self.rake_width = rake_width

    def obstacles_gradient(self, point):
        ''' Computes the gradient for a point and any obstacles it is near
        Used to push points away when inside an obstacle, and attract them towards
        the edge of obstacles.
        '''
        grad = np.array([0., 0., 0.])
        for obstacle in self.obstacles:
            # if isinstance(obstacle, Circle):
            if True:
                # cost based on radius and distance
                # d = pi r^2

                # TODO: Normalize this, because gradient is very high at the edge of shapes
                dx = obstacle.x - point[0]
                dy = obstacle.y - point[1]
                if obstacle.is_inside([point[0], point[1]], self.rake_width/2):
                    grad_vec = [2*dx, 2*dy, 0]
                    grad[:] +=  grad_vec / np.linalg.norm(grad_vec)

            # if isinstance(obstacle, Square):
            #     # cost based on radius and distance
            #     raise KeyError("Rectangle not supported in obstacle gradient yet!")
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
        # print("constant angle grad:", grad)

        return grad


    def optimize_path(self, path: Path):
        optimized_path = []
        # TODO Paths around waypoints are probably too sharp/discontinuous
        # We need to do some global
        for leg in tqdm(path.legs, desc="Optimizing Legs", position=0):
            optimized_leg = self.optimize_leg(leg)

            optimized_path.append(Trajectory(coord_path=optimized_leg))
        
        return Path(optimized_path)

    def optimize_leg(self, leg: Trajectory):
        '''
        path: Trajectory

        Returns:
            Trajectory

        '''
        # Keep initial point and end point fixed, since they should be waypoints
        new_path = leg.array.copy().astype(np.float32)
        initial_lr = 0.1
        prev_total_cost = 999999
        iterations = 500

        for i in tqdm(range(iterations), desc="Optimizing trajectory", position=1, leave=False):
            # Use Learning rate decay
            lr = initial_lr - initial_lr*(i/iterations)
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
            # angle_grad = self.constant_angle_gradient(new_path)

            #total_grad = 0.5 * obstacle_grads + 4 * smoothness_grad + 4 * angle_grad
            debug = False
            if debug:
                print("obstacle grads:", obstacle_grads)
                print("smoothness grad:", smoothness_grad)
            # total_grad = 0.5 * obstacle_grads + 4 * smoothness_grad
            total_grad = 1 * obstacle_grads + 4 * smoothness_grad

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
