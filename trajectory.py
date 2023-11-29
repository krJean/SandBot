from typing import List, Optional, Tuple
import numpy as np

class Pose:
    '''
    Position and orientation represented in cartesian coordinates

    Parameters
    ----------
    x : float
        x coordinate of pose
    y : float
        y coordinate of pose
    theta : float
        rotation (orientation) of pose in radians wrt positive x-axis
    '''

    _x: float
    _y: float
    _theta: float

    def __init__(self,
                    x:float,
                    y:float,
                    theta:float):
        self._x = x
        self._y = y
        self._theta = theta

    def __repr__(self):
        return f'[{self.x},{self.y},{self.theta}]'

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def theta(self):
        return self._theta

class Trajectory:
    '''Container for path represented by a list of Pose objects

    Parameters
    ----------
    path : List[Pose]
        Path representation
    '''

    path: List[Pose]

    def __init__(self,
                 pose_path:Optional[List[Pose]]=None,
                 coord_path:Optional[List[Tuple[float,float,float]]]=None):
        if pose_path is not None:
            self.path = pose_path
        elif coord_path is not None:
            self.path = [Pose(coord[0], coord[1], coord[2]) for coord in coord_path]
        else:
            self.path = []

    def __repr__(self):
        return '\n'.join([str(p) for p in self.path])

    def add_to_path(self, pose: List[Pose]):
        self.path += pose

    @property
    def x(self):
        return [p.x for p in self.path]

    @property
    def y(self):
        return [p.y for p in self.path]

    @property
    def theta(self):
        return [p.theta for p in self.path]

    @property
    def array(self):
        return np.column_stack([self.x, self.y, self.theta], )

    
class Path:
    def __init__(self, legs: Optional[List[Trajectory]]=None):
        self.legs = legs

    @property
    def x(self):
        return np.concatenate([leg.x for leg in self.legs])

    @property
    def y(self):
        return np.concatenate([leg.y for leg in self.legs])

if __name__ == '__main__':
    path = Trajectory(pose_path = [Pose(e,-e, e/3.14159) for e in range(50)])
    print(path)