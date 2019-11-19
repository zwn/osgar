import math
import concurrent.futures

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


DIRECTIONS = [math.radians(direction) for direction in range(-180, 180, 3)]


class LocalPlanner:
    def __init__(self, scan_right=math.radians(-135), scan_left=math.radians(135), direction_adherence=math.radians(90), max_obstacle_distance=1.5, obstacle_influence=1.2):
        self.last_scan = None
        self.scan_right = scan_right
        self.scan_left = scan_left
        self.direction_adherence = direction_adherence
        self.max_obstacle_distance = max_obstacle_distance
        self.obstacle_influence = obstacle_influence

    def update(self, scan):
        self.last_scan = scan

        obstacles = []
        for (i, measurement) in enumerate(self.last_scan):
            if measurement == 0:
                continue
            if measurement * 1e-3 > self.max_obstacle_distance:
                continue
            measurement_angle = self.scan_right + (self.scan_left - self.scan_right) * i / float(len(self.last_scan) - 1)
            measurement_vector = math.cos(measurement_angle), math.sin(measurement_angle)

            # Converting from tenths of milimeters to meters.
            obstacle_xy = [mv * measurement * 1e-3 for mv in measurement_vector]

            obstacles.append(obstacle_xy)

        self.obstacles = obstacles

    def _is_desired(self, direction):
        "Best direction points roughly in desired_dir and does not come too close to any obstacle."
        direction_delta = normalize_angle(direction - self.desired_dir)
        return math.exp(-(direction_delta / self.direction_adherence) ** 2)  # Fuzzy equality

    def _is_risky(self, direction):
        direction_vector = math.cos(direction), math.sin(direction)
        riskiness = 0.
        for obstacle_xy in self.obstacles:
            # Obstacles behind the robot from the perspective of the desired direction do not matter.
            # The formula below computes cos(angle between the two vectors) * their_norms. Norms are positive, so a negative result implies abs(angle) > 90deg.
            if sum(d * o for (d, o) in zip(direction_vector, obstacle_xy)) < 0:
                continue

            # Distance between the obstacle and line defined by direction.
            # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
            # Norm of direction_vector is 1.0, so we do not need to divide by it.
            off_track_distance = abs(direction_vector[1] * obstacle_xy[0] - direction_vector[0] * obstacle_xy[1])

            r = math.exp(-(off_track_distance / self.obstacle_influence) ** 2)
            if r > riskiness:  # max as fuzzy OR.
                riskiness = r

        return riskiness

    def _is_safe(self, direction):
        return 1.0 - self._is_risky(direction)  # Fuzzy negation.

    def _is_good(self, direction):
        return min(self._is_safe(direction), self._is_desired(direction)), direction  # Fuzzy AND.

    def recommend(self, desired_dir, map=False):
        self.desired_dir = normalize_angle(desired_dir)
        if self.last_scan is None:
            return 1.0, self.desired_dir

        if not self.obstacles:
            return 1.0, self.desired_dir

        if map:
            with concurrent.futures.ProcessPoolExecutor() as executor:
                return max(executor.map(self._is_good, DIRECTIONS))

        return max(self._is_good(direction) for direction in DIRECTIONS)

# vim: expandtab sw=4 ts=4
