import math
from typing import List, Tuple
import matplotlib.patches as mpl_patches

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
            obs_list.append(Circle(center=vals['center'],radius=vals['radius']))
        elif shape == 'square':
            obs_list.append(Square(center=vals['center'],side_length=vals['side_length']))
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

    def is_inside(self, point:Tuple[float,float]):
        '''Determine if point lies inside the circle.

        Parameters
        ----------
        point : Tuple[float,float]
            x y coordinates of point to check

        Returns
        -------
        bool
            True if point lies inside the circle
        '''
        p_x, p_y = point
        dist_from_center = math.sqrt( (p_x-self.x)**2 + (p_y-self.y)**2 )

        # TODO: add tolerance
        return dist_from_center <= self.radius

    def get_patch(self) -> mpl_patches.Circle:
        '''Matplotlib circle patch

        Returns
        -------
        matplotlib.patches.Circle
            Circle patch of this Circle's dimensions and position
        '''
        return mpl_patches.Circle((self.x, self.y), self.radius, color='r')


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

    def __init__(self, center:Tuple[float,float], side_length:float):
        self.x, self.y = center
        self.side_length = side_length

        self._half_length = self.side_length / 2
        self._min_x = self.x-self._half_length
        self._max_x = self.x+self._half_length
        self._min_y = self.y-self._half_length
        self._max_y = self.y+self._half_length

    def is_inside(self, point:Tuple[float,float]):
        '''Determine if point lies inside the square.

        Parameters
        ----------
        point : Tuple[float,float]
            x y coordinates of point to check

        Returns
        -------
        bool
            True if point lies inside the square
        '''
        p_x, p_y = point
        # TODO: add tolerance
        return p_x >= self._min_x and p_x <= self._max_x and \
                p_y >= self._min_y and p_y <= self._max_y

    def get_patch(self) -> mpl_patches.Rectangle:
        '''Matplotlib rectangle patch

        Returns
        -------
        matplotlib.patches.Rectangle
            Rectangle patch of this Square's dimensions and position
        '''
        return mpl_patches.Rectangle((self.x, self.y), self.side_length, self.side_length, color='r')
