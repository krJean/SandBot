import pickle
import matplotlib.pyplot as plt
from itertools import groupby
import numpy as np

FACE_COLOR = 'xkcd:off white'
WAYPOINT_COLOR = 'xkcd:baby blue'
TINE_COLOR = 'xkcd:maize'
OBSTACLE_COLOR = 'xkcd:stone'

def plot_sandbox(sandbox: dict, path:list, num_tines:int=5, gradient:bool=False):
    ax : plt.Axes
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(0, sandbox['box_width'])
    ax.set_ylim(0, sandbox['box_length'])
    plt.axis('scaled')
    ax.set_facecolor(FACE_COLOR)

    # Plot obstacles
    for obs in sandbox['obstacles']:
        ax.add_patch(obs.get_patch(OBSTACLE_COLOR))

    path_no_dups = [k for k,_ in groupby(path) if k!=0]

    px = np.array([p[0] for p in path_no_dups])
    py = np.array([p[1] for p in path_no_dups])

    # Conditionally plot single path black->white gradient or ...
    if gradient:
        for i in range(len(px)):
            plt.scatter([px[i]], [py[i]], marker='.', c=f'{i*(1/len(px))}')

    # ... or plot tines
    else:

        # Calculate thetas
        pt = np.zeros(len(px))
        for i in range(len(px)-1):
            pt[i] = np.arctan2(py[i+1] - py[i], px[i+1] - px[i])
        pt[-1] = pt[-2]

        width = sandbox['rake_width']
        spacing = width / num_tines

        # Plot tines
        for i in range(num_tines//2 + 1):

            # Plot one side of the rake
            pt_0 = pt+(np.pi/2)
            px_0 = px + spacing*i*np.cos(pt_0)
            py_0 = py + spacing*i*np.sin(pt_0)
            plt.plot(px_0, py_0, color=TINE_COLOR)

            # Plot the other side of the rake
            pt_1 = pt-(np.pi/2)
            px_1 = px + spacing*i*np.cos(pt_1)
            py_1 = py + spacing*i*np.sin(pt_1)
            plt.plot(px_1, py_1, color=TINE_COLOR)

    # Plot waypoints
    for wp in sandbox['way_points']:
        ax.scatter([wp[0]], [wp[1]], color='xkcd:baby blue')

    plt.show()

# if __name__ == '__main__':
#     path = []
#     with open('data.pickle', 'rb') as f:
#         path = pickle.load(f)
#     from main import read_sandbox
#     sandbox = read_sandbox('configs/two_circles_spaced.json')
#     ax = plot_sandbox(sandbox, path)
#     plt.show()
