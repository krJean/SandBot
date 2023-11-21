import json
import sys
import matplotlib.pyplot as plt
import matplotlib.axes

import obstacles

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
    config['obstacles'] = obstacles.parse_obstacles(config['obstacles'])
    return config

def main():
    sandbox = read_sandbox(sys.argv[1])
    ax = get_sandbox_plot(sandbox)
    plt.show()

if __name__ == '__main__':
    main()
