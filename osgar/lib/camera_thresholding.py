import math
import pdb
import sys
import os
import time
import cv2
import numpy as np
import numpy.linalg as linalg


FRAME_WIDTH = 320
FRAME_HEIGHT = 240

Y_OFFSET = 0 #m

#camera parameters
ROAD_LENGTH = 5.94 #m how long segment of road I want to see in the birdseye image
ROAD_WIDTH= 6.4 #m how wide road is in front of camera
BOTTOM_LEFT_X = -417 #where is the left most point of road on the bottom line if it would be vidible
BOTTOM_RIGHT_X = 760 #where is the right most point of road on the bottom line if it would be vidible
TOP_LEFT_X = 62 #where is the left point in the original image in the distance of ROAD_LENGTH
TOP_RIGHT_X = 270 #where is the right point in the original image in the distance of ROAD_LENGTH
TOP_Y = 133 #where is the y coord of the point in the original image in the distance of ROAD_LENGTH
FOV = 60 #camera FOV in degrees 
        


PIXELS_PER_METER = 320/ROAD_LENGTH
GRID_CELLS_PER_METER = 10

class CameraThresholding():
    def __init__(self):
        self.frameWidth = FRAME_WIDTH
        self.frameHeight = FRAME_HEIGHT
        pixelsPerMeter = self.frameHeight / ROAD_LENGTH
        objPts = np.array([(self.frameWidth / 2 - ROAD_WIDTH / 2 * pixelsPerMeter,0),
                  (self.frameWidth / 2 + ROAD_WIDTH / 2 * pixelsPerMeter,0),
                  (self.frameWidth / 2 + ROAD_WIDTH / 2 * pixelsPerMeter,self.frameHeight),
                  (self.frameWidth / 2 - ROAD_WIDTH / 2 * pixelsPerMeter,self.frameHeight)],dtype = "float32")
        imgPts = np.array([(TOP_LEFT_X,TOP_Y),(TOP_RIGHT_X,TOP_Y),(BOTTOM_RIGHT_X,self.frameHeight),(BOTTOM_LEFT_X,self.frameHeight)],dtype = "float32")
        self.perspectiveMatrix = cv2.getPerspectiveTransform( objPts, imgPts)
        self.inversePerspectiveMatrix = linalg.inv(self.perspectiveMatrix)
        
    
    def update(self,image):
        originalImage = cv2.medianBlur(image,5) 
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
        originalImage = cv2.dilate(originalImage,element)
        originalImage = cv2.dilate(originalImage,element)
        originalImage = cv2.erode(originalImage,element)
        originalImage = cv2.erode(originalImage,element)
        
        #grass
        b = originalImage[:,:,0]
        g = originalImage[:,:,1]
        r = originalImage[:,:,2]
        
        tmp1 = r < 75
        tmp2 = g < 75
        tmp3 = b < 60
        road = np.logical_and(tmp1,np.logical_and(tmp2,tmp3))
        
        road = np.array(road*255,dtype = np.uint8)
#        cv2.imshow("Road", road)
        birdsImage = cv2.warpPerspective(road,self.inversePerspectiveMatrix,(320,240),
                        flags = cv2.INTER_LINEAR  | cv2.WARP_FILL_OUTLIERS)
#        cv2.imshow("birdsImage", birdsImage)
        factor = GRID_CELLS_PER_METER/PIXELS_PER_METER
        resizedBirdsImage = cv2.resize(birdsImage,(int(self.frameWidth*factor),(int(self.frameHeight*factor))))
        #this is because of weird opencv polar transf. 
        forPolarBirdsImage = np.zeros(shape=(360,resizedBirdsImage.shape[1]), dtype="uint8")
        forPolarBirdsImage[360-resizedBirdsImage.shape[0]:,:] = resizedBirdsImage
        
        polar = cv2.linearPolar(forPolarBirdsImage, (forPolarBirdsImage.shape[1]/2,360),forPolarBirdsImage.shape[1], cv2.WARP_FILL_OUTLIERS)
#        cv2.imshow("polar", polar)
        cropedPolar = polar[180:,:]
        cropedPolarBGR = cv2.cvtColor(cropedPolar,cv2.COLOR_GRAY2BGR)
        obstaclesInRows = np.argmin(cropedPolar[:,1:],axis=1)
        row = 0
        obstacles = []
        for (angle,measurement) in enumerate(obstaclesInRows):
            
            if measurement == 0 or angle < 90-FOV/2 or angle > 90+FOV/2:
                #throw away obstacles that are too close or outside of camera FOV
                continue
            cv2.circle(cropedPolarBGR,(measurement,angle),3,(255,0,0))
            angle = math.radians(angle) - math.pi/2
            measurement_vector = math.cos(angle), math.sin(angle)
            obstacle_xy = [mv * measurement/GRID_CELLS_PER_METER for mv in measurement_vector]
            if obstacle_xy[0] > 4.3:
                #throw away obstacles behing perspective projection boundary
                continue
            obstacles.append((float(obstacle_xy[0] + Y_OFFSET),float(-obstacle_xy[1])))
            
#        cv2.imshow("cropedPolar", cropedPolarBGR)
#        cv2.imshow("OriginalImage", image)
#        cv2.waitKey(1)
        return obstacles
    
    
