import math

import numpy
from numba import jit, f8

@jit(f8(f8), nopython=True, cache=True)
def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

scan_right = math.radians(-135)
scan_left = math.radians(135)
direction_adherence = math.radians(90)
max_obstacle_distance = 1.5
obstacle_influence = 1.2

# Best direction points roughly in desired_dir and does not come too close to any obstacle.
@jit(f8(f8, f8), nopython=True, cache=True)
def is_desired(desired_dir, direction):
    direction_delta = normalize_angle(direction - desired_dir)
    return math.exp(-(direction_delta / direction_adherence)**2)  # Fuzzy equality


@jit(f8[:,:](f8[:]), nopython=True, cache=True)
def make_obstacles(last_scan):
    obstacles = []
    for (i, measurement) in enumerate(last_scan):
        if measurement == 0:
            continue
        if measurement * 1e-3 > max_obstacle_distance:
            continue
        measurement_angle = scan_right + (scan_left - scan_right) * i / float(len(last_scan) - 1)
        measurement_vector = math.cos(measurement_angle), math.sin(measurement_angle)

        # Converting from tenths of milimeters to meters.
        obstacle_xy = [mv * measurement * 1e-3 for mv in measurement_vector]
        obstacles.append(obstacle_xy)

    ret = numpy.empty(shape=(len(obstacles),2))
    for i, o in enumerate(obstacles):
        ret[i] = o
    return ret


@jit(f8(f8[:,:], f8), nopython=True, cache=True)
def is_risky(obstacles, direction):
    direction_vector = math.cos(direction), math.sin(direction)
    riskiness = 0.
    for i in range(len(obstacles)):
        obstacle_xy = obstacles[i]
        # Obstacles behind the robot from the perspective of the desired direction do not matter.
        # The formula below computes cos(angle between the two vectors) * their_norms. Norms are positive, so a negative result implies abs(angle) > 90deg.
        if direction_vector[0] * obstacle_xy[0] + direction_vector[1] * obstacle_xy[1] < 0:
            continue

        # Distance between the obstacle and line defined by direction.
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        # Norm of direction_vector is 1.0, so we do not need to divide by it.
        off_track_distance = abs(direction_vector[1] * obstacle_xy[0] - direction_vector[0] * obstacle_xy[1])

        r = math.exp(-(off_track_distance / obstacle_influence)**2)
        if r > riskiness: # max as fuzzy OR.
            riskiness = r

    return riskiness


@jit(f8(f8[:,:], f8), nopython=True, cache=True)
def is_safe(obstacles, direction):
    return 1.0 - is_risky(obstacles, direction)  # Fuzzy negation.


@jit(f8(f8[:,:], f8, f8), nopython=True, cache=True)
def is_good(obstacles, desired_dir, direction):
    return min(is_safe(obstacles, direction), is_desired(desired_dir, direction))  # Fuzzy AND.


@jit((f8[:], f8), nopython=True, cache=True)
def recommend(last_scan, desired_dir):
    
    if len(last_scan) == 0:
        return 1.0, normalize_angle(desired_dir)
    
    obstacles = make_obstacles(last_scan)

    if len(obstacles) == 0:
        return 1.0, normalize_angle(desired_dir)

    best = (0.0, desired_dir)
    for direction in range(-180, 180, 3):
        direction = math.radians(direction)
        good = is_good(obstacles, desired_dir, direction)
        if good >= best[0]:
            best = good, direction
    return best

if __name__ == "__main__":
    scan = numpy.ones(100)*1000.0
    for _ in range(1000):
        for direction in range(-30, 30, 3):
            s, d = recommend(scan, math.radians(direction))
            print(s, d)

