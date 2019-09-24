import math
import pdb
import cv2
import numpy as np
from osgar.lib import camera_thresholding
from osgar.lib.serialize import deserialize
import pygame

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class LocalPlanner:
    def __init__(self, scan_right=math.radians(-135), scan_left=math.radians(135), direction_adherence=math.radians(90), max_obstacle_distance=5, obstacle_influence=1.2):
        self.last_scan = None
        self.last_image = None
        self.scan_right = scan_right
        self.scan_left = scan_left
        self.direction_adherence = direction_adherence
        self.max_obstacle_distance = max_obstacle_distance
        self.obstacle_influence = obstacle_influence
        self.cameraThresholding = camera_thresholding.CameraThresholding()
    
    def update(self, scan,image = None):
        if scan:
            self.last_scan = scan
        if image:
            self.last_image = image
    
    def recommend(self, desired_dir):
        pixelsPerMeter = 50
        obstaclesImageSize = 500
        obstaclesImage = np.zeros((obstaclesImageSize,obstaclesImageSize,3),dtype=np.uint8)            
        if self.last_scan is None:
            return 1.0, desired_dir

        
        obstacles = []
        cv2.circle(obstaclesImage,(int(obstaclesImageSize/2),int(obstaclesImageSize/2)),8,(0,255,255))
        for (i, measurement) in enumerate(self.last_scan):
            if measurement == 0:
                continue
            if measurement * 1e-3 > self.max_obstacle_distance:
                continue
            measurement_angle = self.scan_right + (self.scan_left - self.scan_right) * i / float(len(self.last_scan) - 1)
            measurement_vector = math.cos(measurement_angle), math.sin(measurement_angle)

            # Converting from tenths of milimeters to meters.
            obstacle_xy = [mv * measurement * 1e-3 for mv in measurement_vector]
            cv2.circle(obstaclesImage,(int(obstaclesImageSize/2 + obstacle_xy[0]*pixelsPerMeter),int(obstaclesImageSize/2 - obstacle_xy[1]*pixelsPerMeter)),3,(0,0,255))
            obstacles.append(obstacle_xy)
        
        if self.last_image:
            nparr = np.fromstring(self.last_image, np.uint8)
            img = cv2.imdecode(nparr, 1)
            cameraObstacles = self.cameraThresholding.update(img) 
            obstacles += cameraObstacles
            for obstacle in cameraObstacles:
                cv2.circle(obstaclesImage,(int(obstaclesImageSize/2 + obstacle[0]*pixelsPerMeter),int(obstaclesImageSize/2 - obstacle[1]*pixelsPerMeter)),3,(255,0,0))
            
#        cv2.imshow("obstacles", obstaclesImage)
        
#        cv2.waitKey(1)
            
        if not obstacles:
            return 1.0, normalize_angle(desired_dir)

        # Best direction points roughly in desired_dir and does not come too close to any obstacle.
        def is_desired(direction):
            direction_delta = normalize_angle(direction - desired_dir)
            return math.exp(-(direction_delta / self.direction_adherence)**2)  # Fuzzy equality

        def is_risky(direction):
            direction_vector = math.cos(direction), math.sin(direction)
            riskiness = 0.
            for obstacle_xy in obstacles:
                # Obstacles behind the robot from the perspective of the desired direction do not matter.
                # The formula below computes cos(angle between the two vectors) * their_norms. Norms are positive, so a negative result implies abs(angle) > 90deg.
                if sum(d * o for (d, o) in zip(direction_vector, obstacle_xy)) < 0:
                    continue

                # Distance between the obstacle and line defined by direction.
                # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
                # Norm of direction_vector is 1.0, so we do not need to divide by it.
                off_track_distance = abs(direction_vector[1] * obstacle_xy[0] - direction_vector[0] * obstacle_xy[1])

                r = math.exp(-(off_track_distance / self.obstacle_influence)**2)
                if r > riskiness: # max as fuzzy OR.
                    riskiness = r

            return riskiness

        def is_safe(direction):
            return 1.0 - is_risky(direction)  # Fuzzy negation.

        def is_good(direction):
            return min(is_safe(direction), is_desired(direction))  # Fuzzy AND.

        return max((is_good(math.radians(direction)), math.radians(direction)) for direction in range(-180, 180, 3))

# vim: expandtab sw=4 ts=4
