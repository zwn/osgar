import math
import pdb
import sys
import os
import time
import cv2
import numpy as np
import numpy.linalg as linalg
import pygame

#camera parameters
ROAD_LENGTH = 10 #m how long segment of road I want to see in the birdseye image
ROAD_WIDTH= 8 #m how wide road is in front of camera
BOTTOM_LEFT_X = -180 #where is the left most point of road on the bottom line if it would be vidible
BOTTOM_RIGHT_X = 500 #where is the right most point of road on the bottom line if it would be vidible
TOP_LEFT_X = 70 #where is the left point in the original image in the distance of ROAD_LENGTH
TOP_RIGHT_X = 250 #where is the right point in the original image in the distance of ROAD_LENGTH
TOP_Y = 133 #where is the y coord of the point in the original image in the distance of ROAD_LENGTH

PIXELS_PER_METER = 320/ROAD_LENGTH
GRID_CELLS_PER_METER = 10

class CameraThresholding():
    def __init__(self):
        self.frameWidth = 320
        self.frameHeight = 240
        pixelsPerMeter = self.frameHeight / ROAD_LENGTH
        objPts = np.array([(self.frameWidth / 2 - ROAD_WIDTH / 2 * pixelsPerMeter,0),
                  (self.frameWidth / 2 + ROAD_WIDTH / 2 * pixelsPerMeter,0),
                  (self.frameWidth / 2 + ROAD_WIDTH / 2 * pixelsPerMeter,self.frameHeight),
                  (self.frameWidth / 2 - ROAD_WIDTH / 2 * pixelsPerMeter,self.frameHeight)],dtype = "float32")
        imgPts = np.array([(TOP_LEFT_X,TOP_Y),(TOP_RIGHT_X,TOP_Y),(BOTTOM_RIGHT_X,self.frameHeight),(BOTTOM_LEFT_X,self.frameHeight)],dtype = "float32")
        self.perspectiveMatrix = cv2.getPerspectiveTransform( objPts, imgPts)
        self.inversePerspectiveMatrix = linalg.inv(self.perspectiveMatrix)
        
    
    def update(self,image):
        #  create a copy of the surface
        view = pygame.surfarray.array3d(image)

        #  convert from (width, height, channel) to (height, width, channel)
        view = view.transpose([1, 0, 2])

        #  convert from rgb to bgr
        image = cv2.cvtColor(view, cv2.COLOR_RGB2BGR)
        
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
        cv2.imshow("Road", road)
        birdsImage = cv2.warpPerspective(road,self.inversePerspectiveMatrix,(320,240),
                        flags = cv2.INTER_LINEAR  | cv2.WARP_FILL_OUTLIERS)
        cv2.imshow("birdsImage", birdsImage)
        factor = GRID_CELLS_PER_METER/PIXELS_PER_METER
        resizedBirdsImage = cv2.resize(birdsImage,(int(self.frameWidth*factor),(int(self.frameHeight*factor))))
        #this is because of weird opencv polar transf. 
        forPolarBirdsImage = np.zeros(shape=(360,resizedBirdsImage.shape[1]), dtype="uint8")
        forPolarBirdsImage[360-resizedBirdsImage.shape[0]:,:] = resizedBirdsImage
        
        
        polar = cv2.linearPolar(forPolarBirdsImage, (forPolarBirdsImage.shape[1]/2,360),forPolarBirdsImage.shape[1], cv2.WARP_FILL_OUTLIERS)
        cv2.imshow("polar", polar)
        cropedPolar = polar[180:,:]
        cropedPolarBGR = cv2.cvtColor(cropedPolar,cv2.COLOR_GRAY2BGR)
        obstaclesInRows = np.argmin(cropedPolar[:,1:],axis=1)
        row = 0
        obstacles = []
        for index in obstaclesInRows:
            cv2.circle(cropedPolarBGR,(index,row),3,(255,0,0))
            obstacles.append((float(index/GRID_CELLS_PER_METER),float(row/GRID_CELLS_PER_METER)))
            row += 1
        
        cv2.imshow("cropedPolar", cropedPolarBGR)
        
        
        
        cv2.imshow("OriginalImage", image)
        
        cv2.waitKey(1)
        
        return obstacles
    
    