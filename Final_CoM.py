# Final version of the CV
import numpy as np
import cv2  # OpenCV module
import time
from tkinter import *

# Constants and other global variables
CAMERAID = 6 # CHANGE AS NECESSARY
camera = cv2.VideoCapture(CAMERAID) # define camera
colorID = [0,0,0,0]
OFFSET = .02
COLORS = [(0,255,120),(255,120,120),(0,69,255),(255,255,255)] # Yellow, Blue, Orange, Clear
#POINTS = [(.69154,-.600138),(.67307,-.135855),(.1763488,-.1731145598126),(.13237,-.63746)] # Calibrate from robot
#MYPOINTS = [(33,403),(350,400),(370,67),(25,25)] # Equivalent in pixels
POINTS = [[.4,-.600138],[.4,-.135855],[0.14355, -0.12404],[.13237,-.63746]]
MYPOINTS = [(29,200),(346,218),(368,47),(18,22)]
POINTS = [[_[0], _[1]-OFFSET] for _ in POINTS]
TOPCUTOFF = 90 # How far from top before sending data
BOTTCUTOFF = 300 # How far from bottom before beginning data collection
DT=.05 
src = np.array(MYPOINTS)
dst = np.array(POINTS)

#Threshold Values
# yellow 
lower_bound_HSV_yellow = np.array([24, 0, 220]) 
upper_bound_HSV_yellow = np.array([40, 255, 255])
# blue 
lower_bound_HSV_blue = np.array([71, 135, 68])
upper_bound_HSV_blue = np.array([105, 255, 255])
# orange 
lower_bound_HSV_orange = np.array([8, 0, 220])
upper_bound_HSV_orange = np.array([29, 255, 255])
thresholds = [[lower_bound_HSV_yellow,upper_bound_HSV_yellow],
              [lower_bound_HSV_blue,upper_bound_HSV_blue],
               [lower_bound_HSV_orange,upper_bound_HSV_orange],
               [np.array([255,255,255]),np.array([255,255,255])]]

#Using values, calculate H (one plane to another)
x, y, u, v = src[:,0], src[:,1], dst[:,0], dst[:,1]
A = np.zeros((9,9))
j = 0
for i in range(4):
    A[j,:] = np.array([-x[i], -y[i], -1, 0, 0, 0, x[i]*u[i], y[i]*u[i], u[i]])
    A[j+1,:] = np.array([0, 0, 0, -x[i], -y[i], -1, x[i]*v[i], y[i]*v[i], v[i]])
    j += 2
A[8, 8] = 1   # assuming h_9 = 1
b = [0]*8 + [1]
H = np.reshape(np.linalg.solve(A, b), (3,3))
#print(H)

print(H[0,0]*1+H[0,1]*1+H[0,2],H[1,0]*1+H[1,1]*1+H[1,2])
print(H[0,0]*439+H[0,1]*1+H[0,2],H[1,0]*439+H[1,1]*1+H[1,2])

#Defining Bottle Class
class Bottle:
    def __init__(self,clr: int, pos: list,rpos: list):
        self.color = clr
        self.pos = [pos]
        self.time = [time.time()]
        self.velX = 0
        self.velY = 0
        self.future = [0 for _ in range(100)]
        self.robot_pos=[rpos]
    def update_pos(self,new_pos: list, rob_pos: list):
        self.pos.append(new_pos)
        self.time.append(time.time())
        self.robot_pos.append(rob_pos)
    def find_velocity(self):
        self.velX = (self.pos[-1][0]-self.pos[0][0])/(self.time[-1]-self.time[0])
        self.velY = (self.pos[-1][1]-self.pos[0][1])/(self.time[-1]-self.time[0])
    def future_pos(self):
        dT = DT
        curr_time = time.time()
        for i in range(100):
            self.future[i] = (self.pos[-1][0]+self.velX*dT*i,self.pos[-1][1]+self.velY*dT*i,curr_time+dT*i)
    def send_data(self):
        rvelY = (self.robot_pos[-1][0]-self.robot_pos[-25][0])/(self.time[-1]-self.time[-25])
        print(len(self.robot_pos))
        distance = abs(self.robot_pos[-1][0]+.3)
        self.exp_time = distance/(abs(rvelY))
        return (self.exp_time,self.color,self.robot_pos[-1][1])

def find_contours(image):
    contoursss, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    min_area = 2500
    return [cnt for cnt in contoursss if (cv2.contourArea(cnt) > min_area and cv2.contourArea(cnt) < 9000)]

#Once bottle initiated, track just that one color
def track_one(bottle):
    cv2.destroyAllWindows()
    kernel = np.ones((7,7),np.uint8)
    num_iterations = 3
    bru=0
    while True:
        bru = bru+1
        ret, cv_image1 = camera.read()
        cv_image = cv_image1[0:440,138:540]
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        if bottle.color!=3:
            mask_HSV = cv2.inRange(hsv_image, thresholds[bottle.color][0], thresholds[bottle.color][1])
            opening = cv2.morphologyEx(mask_HSV, cv2.MORPH_OPEN, kernel, iterations = num_iterations)
        else:
            gry = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
            cannyIm = cv2.Canny(gry, 100, 150, apertureSize = 3)
            for i in range(10):
                for j in range(len(cannyIm)):
                    cannyIm[j][len(cannyIm[0])-(i+1)] = 0
            opening = cv2.morphologyEx(cannyIm, cv2.MORPH_CLOSE, kernel, iterations = num_iterations)
        contours = find_contours(opening)
        cv2.drawContours(cv_image, contours, -1, COLORS[bottle.color],3)
        if (len(contours)==1):
            cX,cY = find_CoM(bottle.color,contours,cv_image)
            # # Draw the contour and center of the shape on the image (our viewing)
            cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(cv_image, "center", (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            new = (H[0,0]*cX+H[0,1]*cY+H[0,2],H[1,0]*cX+H[1,1]*cY+H[1,2])
            if cY<TOPCUTOFF:
                if bru<5:
                    return None
                bottle.send_data()
                return bottle.send_data()
            elif cY>BOTTCUTOFF:
                return None
        else:
            return None

        bottle.update_pos((cX,cY),new)
        bottle.find_velocity()
        bottle.future_pos()
        
        for i in range(1, len(bottle.future)):
            pt1 = tuple(map(int, bottle.future[i - 1][0:2]))
            pt2 = tuple(map(int, bottle.future[i][0:2]))
            cv2.line(cv_image, pt1, pt2, COLORS[bottle.color], 2)
        
        cv2.imshow("Original",cv_image)
        if bottle.color == 0:
            cv2.imshow("Opening - Yellow", opening)
        elif bottle.color == 1:
            cv2.imshow("Opening - Blue", opening)
        elif bottle.color == 2:
            cv2.imshow("Opening - Orange", opening)
        elif bottle.color == 3:
            cv2.imshow("Opening - Clear", opening)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            return None

# The main loop of the function
def detect():
    # Open up the webcam
    while True:
        # Read from the camera frame by frame and crop
        ret, cv_image1 = camera.read()
        cv_image = cv_image1[0:430,138:540]
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # create four color masks
        mask_HSV_yellow = cv2.inRange(hsv_image, lower_bound_HSV_yellow, upper_bound_HSV_yellow)
        mask_HSV_blue = cv2.inRange(hsv_image, lower_bound_HSV_blue, upper_bound_HSV_blue)
        mask_HSV_orange = cv2.inRange(hsv_image, lower_bound_HSV_orange, upper_bound_HSV_orange)
        #cannyIm = cv2.Canny(cv_image, 50, 150, apertureSize = 3)
        gry = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        cannyIm = cv2.Canny(gry, 100, 150, apertureSize = 3)
        for i in range(10):
            for j in range(len(cannyIm)):
                cannyIm[j][len(cannyIm[0])-(i+1)] = 0
        kernel = np.ones((7,7),np.uint8)
        num_iterations = 3

        ################ Color Detection ####################
        opening_yellow = cv2.morphologyEx(mask_HSV_yellow, cv2.MORPH_OPEN, kernel, iterations = num_iterations)
        opening_blue = cv2.morphologyEx(mask_HSV_blue, cv2.MORPH_OPEN, kernel, iterations = num_iterations)
        opening_orange = cv2.morphologyEx(mask_HSV_orange, cv2.MORPH_OPEN, kernel, iterations = num_iterations)
        cn_clear = cv2.morphologyEx(cannyIm, cv2.MORPH_CLOSE, kernel, iterations = num_iterations)

        images = [opening_yellow,opening_blue,opening_orange,cn_clear]

        for img in range(len(images)):
            contours = find_contours(images[img])
            if len(contours)==1:
                cX,cY = find_CoM(img,contours,cv_image)
                
                cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(cv_image, "center", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                new = (H[0,0]*cX+H[0,1]*cY+H[0,2],H[1,0]*cX+H[1,1]*cY+H[1,2])
                if img == 3:
                    offset = 10
                else:
                    offset=0
                if cY<BOTTCUTOFF-offset and cY>TOPCUTOFF+50:
                    track = Bottle(img,(cX,cY),new)
                    data = track_one(track)
                    if data != None:
                        return data

        #cv2.circle(cv_image, (33,403), 7, (255, 255, 255), -1)
        #cv2.circle(cv_image, (350,400), 7, (255, 255, 255), -1)
        #cv2.circle(cv_image, (370,67), 7, (255, 255, 255), -1)
        #cv2.circle(cv_image, (25,25), 7, (255, 255, 255), -1)

        ## display image
        cv2.imshow("Original",cv_image)
        cv2.imshow("Canny_Image", cannyIm)
        cv2.imshow("Opening - Yellow", opening_yellow)
        cv2.imshow("Opening - Blue", opening_blue)
        cv2.imshow("Opening - Orange", opening_orange)
        cv2.imshow("Opening - Clear", cn_clear)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

def find_CoM(color,contours,cv_image):
    if color != 3:
        # Compute the center of the contour
        M = cv2.moments(contours[0])
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        rect = cv2.minAreaRect(contours[0])
        box = np.intp(cv2.boxPoints(rect))
        cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
        cX = 0
        cY = 0
        for point in box:
            cX = cX+point[0]/4
            cY = cY+point[1]/4
        cX = np.intp(cX)
        cY = np.intp(cY)
        print(len(contours))
    return cX, cY

if __name__=='__main__':
    detect()