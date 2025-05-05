
import numpy as np
import cv2  # OpenCV module
import time
from tkinter import *
import robot as rb

ID = 1
CAM = 6
neutral = [0.015466521999787283, -0.4899471595439902, 0.05152892504111392, -2.3417131655824277, 2.0681925358661806, -0.03821438995733724]
POINTS = [[.69154,-.600138],[.67307,-.135855],[0.14355, -0.12404],[.13237,-.63746]]
MYPOINTS = [(33,403),(338,400),(368,47),(18,22)]
POINTS = [[.4,-.600138],[.4,-.135855],[0.14355, -0.12404],[.13237,-.63746]]
MYPOINTS = [(29,200),(346,218),(368,47),(18,22)]

target = neutral.copy()
target[0:2] = POINTS[ID]
#print(target)

tk = Tk()
cX = Scale(tk, from_ = 0, to = 500, label = 'X', orient = HORIZONTAL)
cX.pack()
cY = Scale(tk, from_ = 0, to = 400, label = 'Y', orient = HORIZONTAL)
cY.pack()
cX.set(MYPOINTS[ID][0])
cY.set(MYPOINTS[ID][1])

def main():
    while True:
        tk.update()
        ret, cv_image1 = rb.cv.camera.read()
        cv2.waitKey(3)
        print(cv_image1)
        cv_image = cv_image1[0:440,138:540]
        cv2.circle(cv_image, (cX.get(), cY.get()), 7, (255, 255, 255), -1)
        cv2.imshow("Original_Image", cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
if __name__=='__main__':
    if ID!=2:
        rb.moveL(target)
        start_time = time.time()
        while time.time()-start_time<4:
            print('bru')
        rb.moveL(neutral)
    main()