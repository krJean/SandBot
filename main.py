import json
import matplotlib.axes
import matplotlib.pyplot as plt
import numpy as np
import sys
from typing import List, Optional, Tuple, Union

from obstacles import Circle, Square, parse_obstacles
from pathPlanner import PathPlanner
from trajectory import Trajectory

def get_sandbox_plot(sandbox: dict) -> matplotlib.axes.Axes:
    '''Matplotlib axes with sandbox objects and waypoints plotted

    Returns
    -------
    matplotlib.axes.Axes
        Axes with given sandbox configuration
    '''
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(0, sandbox['box_width'])
    ax.set_ylim(0, sandbox['box_length'])
    plt.axis('scaled')

    for obs in sandbox['obstacles']:
        ax.add_patch(obs.get_patch())

    for wp in sandbox['way_points']:
        ax.scatter([wp[0]], [wp[1]], color='b')

    return ax

def read_sandbox(config_path:str) -> dict:
    '''JSON parser for sandbox config

    Parameters
    ----------
    config_path : str
        Path to sandbox config JSON file

    Returns
    -------
    dict
        Sandbox config with obstacles defined as shape objects
    '''
    with open(config_path, 'r') as cfg:
        config = json.load(cfg)
    config['obstacles'] = parse_obstacles(config['obstacles'])
    return config


def is_collision(point:Tuple[float,float], obstacles:List[Union[Circle, Square]]) -> bool:
    '''Test if given point is inside an obstacle

    Parameters
    ----------
    point : Tuple[float,float]
        Coordinate to check
    obstacles : List[Union[Circle, Square]]
        Obstacles of potential collision

    Returns
    -------
    bool
        True if given point is inside one of the obstacles
    '''
    for i, obs in enumerate(obstacles):
        if obs.is_inside(point):
            return True
    return False


def make_cost_map(sandbox:dict) -> np.ndarray:
    '''Convert sandbox config to 2D cost map

    Parameters
    ----------
    sandbox : dict
        Sandbox configuration

    Returns
    -------
    np.ndarray
        2D cost map of sandbox
    '''
    cost_map = np.ones((sandbox['box_width'],sandbox['box_length']))*0.5

    # This could be more efficient if the shapes could return what coords they cover instead...
    for i in range(sandbox['box_width']):
        for j in range(sandbox['box_length']):
            if is_collision((i,j), sandbox['obstacles']):
                cost_map[i,j] = 1.
    return cost_map


def show(cost_map:Optional[np.ndarray]=None, sandbox:Optional[dict]=None, path:Optional[Trajectory]=None):
    '''Wrapper to show multiple representations of the sandbox

    Parameters
    ----------
    cost_map : Optional[np.ndarray], optional
        2D array of obstacle cost, by default None
    sandbox : Optional[dict], optional
        Dictionary of sandbox configuration, by default None
    path : Optional[Trajectory], optional
        Path of rake through sandbox, by default None
    '''
    if sandbox is not None:
        _ = get_sandbox_plot(sandbox)
    if cost_map is not None:
        _ = plt.gca().imshow(cost_map.T, origin='lower')
    if path is not None:
        _ = plt.gca().plot(path.x,path.y,'-g.')
    plt.show()

def main():
    sandbox = read_sandbox(sys.argv[1])
    cost_map = make_cost_map(sandbox)

    planner = PathPlanner(cost_map)
    waypoints = sandbox['way_points']
    trajectory = Trajectory(coord_path=planner.plan_path(waypoints))

    print(trajectory)
    show(cost_map=cost_map, sandbox=sandbox, path=trajectory)



if __name__ == '__main__':
    main()
