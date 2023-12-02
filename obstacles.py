import math
from typing import List, Tuple
import matplotlib.patches as mpl_patches
import matplotlib.transforms as mpl_transforms
import numpy as np

def parse_obstacles(json_obs_list:List[dict]) -> List:
    '''JSON parser for sandbox obstacles

    Parameters
    ----------
    json_obs_list : List[dict]
        List of obstacles configuration dictionaries

    Returns
    -------
    List
        List of obstacles as shape objects (circle or square)
    '''
    obs_list = []
    for obs in json_obs_list:
        shape = list(obs.keys())[0]
        vals = list(obs.values())[0]
        if shape == 'circle':
            obs_list.append(Circle(center=vals['center'],
                                   radius=vals['radius']))
        elif shape == 'square':
            obs_list.append(Square(center=vals['center'],
                                   side_length=vals['side_length'],
                                   rotation=vals.get('rotation',0.)))
        else:
            raise Exception(f'No implementation for obstacle of shape {list(obs.keys())}')
    return obs_list

class Circle:
    '''
    Representation of a circle on a coordinate plane

    Parameters
    ----------
    center : Tuple[float,float]
        x y coordinates of the center of the circle
    radius : float
        radius of the circle
    '''

    x : float
    y : float
    radius : float

    def __init__(self, center:Tuple[float,float], radius:float):
        self.x, self.y = center
        self.radius = radius

        self._min_x = self.x - self.radius
        self._max_x = self.x + self.radius
        self._min_y = self.y - self.radius
        self._max_y = self.y + self.radius

    def is_inside(self, point:Tuple[float,float], bloat:float=0.):
        '''Determine if point lies inside the circle with some bloat bounds.

        Parameters
        ----------
        point : Tuple[float,float]
            x y coordinates of point to check
        bloat : float
            Distance from obstacle edge that's still "out-of-bounds",
            by default 0.0

        Returns
        -------
        bool
            True if point lies inside the circle
        '''
        p_x, p_y = point
        dist_from_center = math.sqrt( (p_x-self.x)**2 + (p_y-self.y)**2 )

        # TODO: add tolerance
        return dist_from_center <= self.radius+bloat

    def get_patch(self, ax, color='r') -> mpl_patches.Circle:
        '''Matplotlib circle patch

        Returns
        -------
        matplotlib.patches.Circle
            Circle patch of this Circle's dimensions and position
        '''
        return mpl_patches.Circle((self.x, self.y), self.radius, color=color)


class Square:
    '''
    Representation of an axis-aligned square on a coordinate plane

    Parameters
    ----------
    center : Tuple[float,float]
        x y coordinates of the center of the square
    side_length : float
        length of one side of the square
    '''

    x : float
    y : float
    side_length : float
    rotation : float

    def __init__(self, center:Tuple[float,float], side_length:float, rotation:float=0.):
        self.x, self.y = center
        self.side_length = side_length
        self.rotation = rotation
        self._half_length = self.side_length / 2
        self._min_x = self.x-self._half_length
        self._max_x = self.x+self._half_length
        self._min_y = self.y-self._half_length
        self._max_y = self.y+self._half_length

        self.norm_corners = np.array([
            [self._min_x, self._max_y],
            [self._max_x, self._max_y],
            [self._max_x, self._min_y],
            [self._min_x, self._min_y],
        ])
        self.rot_matrix = np.array([
            [np.cos(self.rotation), -np.sin(self.rotation)],
            [np.sin(self.rotation), np.cos(self.rotation)],
        ])
        self.rot_corners = (self.rot_matrix @ (self.norm_corners - [self.x, self.y]).T).T

    def is_inside(self, point:Tuple[float,float], bloat:float=0.):
        '''Determine if point lies inside the square with some bloat bounds.

        Parameters
        ----------
        point : Tuple[float,float]
            x y coordinates of point to check
        bloat : float
            Distance from obstacle edge that's still "out-of-bounds",
            by default 0.0

        Returns
        -------
        bool
            True if point lies inside the square
        '''
        p_x, p_y = (self.rot_matrix @ (np.array(point) - [self.x, self.y]).T).T + [self.x, self.y]
        bloat_min_x = self._min_x - bloat
        bloat_min_y = self._min_y - bloat
        bloat_max_x = self._max_x + bloat
        bloat_max_y = self._max_y + bloat

        # TODO: add tolerance
        # Covers square and rectangular bloat sections on the sides
        if (self._min_x <= p_x <= self._max_x and
            bloat_min_y <= p_y <= bloat_max_y) or \
           (bloat_min_x <= p_x <= bloat_max_x and
            self._min_y <= p_y <= self._max_y):
            return True

        # Covers rounded corners from bloat
        for c_x, c_y in [(self._min_x,self._min_y),
                         (self._min_x,self._max_y),
                         (self._max_x,self._min_y),
                         (self._max_x,self._max_y)]:
            if math.sqrt( (p_x-c_x)**2 + (p_y-c_y)**2 ) <= bloat:
                return True

        return False

    def get_patch(self, ax, color='r') -> mpl_patches.Rectangle:
        '''Matplotlib rectangle patch

        Returns
        -------
        matplotlib.patches.Rectangle
            Rectangle patch of this Square's dimensions and position
        '''
        transform = mpl_transforms.Affine2D().rotate_around(self.x, self.y, self.rotation) + ax.transData
        patch = mpl_patches.Rectangle((self._min_x, self._min_y), self.side_length, self.side_length, color=color)
        patch.set_transform(transform)
        return patch
