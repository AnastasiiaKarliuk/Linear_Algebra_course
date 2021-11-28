import numpy as np
from matplotlib import pyplot as plt


def check_polyline(polyline, obstacles):
    """this function returns True if the polyline does not intersect obstacles
    Otherwise it returns False
    You can use it to verify your algorithm
    """
    for obstacle in obstacles:
        for i in range(len(obstacle)):
            obstacle_segment = (obstacle[i - 1], obstacle[i])
            for j in range(1, len(polyline)):
                path_segment = (polyline[j - 1], polyline[j])
                if is_segments_intersect(obstacle_segment, path_segment):
                    print("segments intersect:", obstacle_segment, path_segment)
                    return False
    return True


def is_segments_intersect(seg_1, seg_2):
    v1, v2 = seg_1
    u1, u2 = seg_2

    M = np.array([v1 - v2, u2 - u1]).T
    if np.linalg.matrix_rank(M) < 2:
        return False

    a, b = np.linalg.inv(M).dot(u2 - v2)

    return (0 < a < 1) and (0 < b < 1)


def calculateDistance(x1, y1, x2, y2):
    dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist


def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


def get_crossed_vec(start, curr_step, obstacles):
    crossed_vec, dists = [], []
    for obs in obstacles:
        for vec1, vec2 in zip(obs, np.vstack([obs[1:], obs[0]])):
            line1 = np.array((vec1, vec2))
            crossed = is_segments_intersect(line1, (start, curr_step))
            if crossed:
                l1 = vec1 - vec2
                l2 = start - curr_step
                l1 = line_intersection((vec1, vec2), (start, curr_step))
                dist = calculateDistance(x1=l1[0], y1=l1[1], x2=l2[0], y2=l2[1])
                crossed_vec.append(line1)
                dists.append(dist)
    if len(crossed_vec) > 0:
        crossed_vec = crossed_vec[np.argmin(dists)]
    return crossed_vec


def get_angle(vec1, vec2):
    norm_v1 = np.linalg.norm(vec1)
    norm_v2 = np.linalg.norm(vec2)
    cosA = np.cross(vec1, vec2) / (norm_v1 * norm_v2)
    alpha = np.rad2deg(np.arccos(cosA))
    return alpha


def get_new_direction(crossed_vec, pure_vector):
    a, b = crossed_vec
    angleAB = get_angle(pure_vector, b - a)
    angleBA = get_angle(pure_vector, a - b)
    new_dir = b - a if (angleAB < angleBA and ((b - a) > 0).any()) else a - b
    new_dir_normed = new_dir / np.linalg.norm(new_dir)
    return new_dir_normed


def get_the_nearest_point(curr_start, finish, next_step):
    dist_to_finish = calculateDistance(curr_start[0], curr_start[1], finish[0], finish[1])
    dist_to_next_point = calculateDistance(curr_start[0], curr_start[1], next_step[0], next_step[1])
    return finish if dist_to_finish < dist_to_next_point else next_step


def get_next_step(finish, curr_start, obstacles):
    pure_vector = finish - curr_start
    pure_vector_normed = pure_vector / np.linalg.norm(pure_vector)

    next_step = curr_start + pure_vector_normed
    crossed_vec = get_crossed_vec(curr_start, next_step, obstacles)

    if len(crossed_vec):
        new_dir_normed = get_new_direction(crossed_vec, pure_vector)
        next_stp = curr_start + new_dir_normed
    else:
        next_stp = curr_start + pure_vector_normed

    curr_start = get_the_nearest_point(curr_start, finish, next_stp)
    return curr_start


def plot_polyline(polyline: list, obstacles: list):
    plt.figure(figsize=(6, 6))
    plt.axis('equal')

    if len(obstacles) > 0:
        for ob in obstacles:
            ob = list(ob)
            ob.append(ob[0])
            x, y = zip(*ob)
            plt.plot(x, y, c='red')

    x, y = zip(*polyline)
    plt.scatter(x, y)
    plt.plot(x, y)
    plt.grid()
    plt.show()
