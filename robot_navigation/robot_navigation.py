import sys
import json

from utils import *


def find_path(start, finish, obstacles=None):
    obstacles = [] if len(obstacles) == 0 else obstacles

    start = np.array(start)
    finish = np.array(finish)
    obstacles = [np.array(ob) for ob in obstacles]

    print("Start:", start)
    print("Finish point:", finish)
    print("Obstacles:", obstacles)

    robot_width = 0.04
    obstacles_new = np.array([widther_obstacle(obs, robot_width / 2) for obs in obstacles])

    curr_start = start
    polyline = [curr_start]
    max_iteration = 1_000

    while max_iteration >= 0 and (curr_start != finish).all():
        curr_start = get_next_step(finish, curr_start, obstacles_new)
        polyline.append(curr_start)
        max_iteration -= 1

    plot_path(obstacles, polyline)
    return polyline


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise "USAGE EXAMPLE:\n\n    python robot_navigation.py robot_data.json\n"

    data_file = sys.argv[1]
    with open(data_file, 'r') as f:
        data = json.load(f)
    find_path(data["start"], data["finish"], data["obstacles"])
