#################################################################################################################################
### This file implements color specific sphere tracking. It can potentially be used for tracking recovery by object detection ###
#################################################################################################################################

#Requires refactoring. This is just a "proof of concept" implementation

import cv2
import numpy as np
import threading
import time


#####HSV Colour Ranges#################
#If the ball is red (0-10) or (170-180)
redLowMask = (0,50,50)
redHighMask = (10, 255, 255)

#If the ball is blue
blueLowMask = (100, 150, 0)
blueHighMask = (140, 255, 255)

#If the ball is orange
orangeLowMask = (5, 50, 50)
orangeHighMask = (20, 255, 255)

#If the ball is green
greenLowMask= (90, 50, 50)
greenHighMask= (150, 255, 255)

#If the ball is yellow
yellowLowMask = (25, 50, 70)
yellowHighMask = (35, 255, 255)


########################################

class Tracker:

    def __init__(self, pointColor, goalColor):
        self.point = (0,0,0)
        self.goal = (0,0,0)
        thread = threading.Thread(target=self.TrackerThread, args=(pointColor, goalColor), daemon=True)
        thread.start()

    def TrackerThread(self, pointColor, goalColor):
        print("Tracker Started")
        # Get the camera
        vc = cv2.VideoCapture(0)
        if vc.isOpened(): # try to get the first frame
            rval, frame = vc.read()
        else:
            rval = False
        while rval:
            # Handle current frame
            rval, frame = vc.read()
            circlesPoint = self.GetLocation(frame, pointColor)
            circlesGoal = self.GetLocation(frame, goalColor)
            self.DrawCircles(frame, circlesPoint, (255, 0, 0))
            self.DrawCircles(frame, circlesGoal, (0, 0, 255))

            if circlesPoint is not None:
                self.point = circlesPoint[0]
            
            if circlesGoal is not None:
                self.goal = circlesGoal[0]

            # Shows the original image with the detected circles drawn.
            cv2.imshow("Result", frame)

            # check if esc key pressed
            key = cv2.waitKey(20)
            if key == 27:
                break
        
        vc.release()
        cv2.destroyAllWindows()
        print("Tracker Ended")

    def GetLocation(self, frame, color):
        # Uncomment for gaussian blur
        #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        blurred = cv2.medianBlur(frame,11)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == 'r':
            # Red Tracking
            mask = cv2.inRange(hsv, redLowMask, redHighMask)
        if color == 'o':
            # Orange Tracking
            mask = cv2.inRange(hsv, orangeLowMask, orangeHighMask)
        if color == 'b':
            # Blue Tracking
            mask = cv2.inRange(hsv, blueLowMask, blueHighMask)
        if color == 'g':
            # Green Tracking
            mask = cv2.inRange(hsv, greenLowMask, greenHighMask)
        if color == 'y':
            # Yellow Tracking
            mask = cv2.inRange(hsv, yellowLowMask, yellowHighMask)
        # Perform erosion and dilation in the image (in 11x11 pixels squares) in order to reduce the "blips" on the mask
        mask = cv2.erode(mask, np.ones((11, 11),np.uint8), iterations=2)
        mask = cv2.dilate(mask, np.ones((11, 11),np.uint8), iterations=5)
        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv2.bitwise_and(blurred,blurred, mask= mask)
        # masked_blurred = cv2.bitwise_and(frame,frame, mask= mask)
        # Convert the masked image to gray scale (Required by HoughCircles routine)
        result = cv2.cvtColor(masked_blurred, cv2.COLOR_BGR2GRAY)
        # Detect circles in the image using Canny edge and Hough transform
        circles = cv2.HoughCircles(result, cv2.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=20, minRadius=30, maxRadius=200)
        return circles
            
    def DrawCircles(self, frame, circles, dotColor):
        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                #print("Circle: " + "("+str(x)+","+str(y)+")")
                # draw the circle in the output image, then draw a rectangle corresponding to the center of the circle
                # The circles and rectangles are drawn on the original image.
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), dotColor, -1)


class VisualServoing:
    def __init__(self, server):
        self.server = server
        self.queue = Queue()
        self.J = self.estimate_initial_jacobian()
        self.stop_threshold = 0.01
        self.cap = cv2.VideoCapture(0)  # Initialize camera

    def estimate_initial_jacobian(self):
        # Placeholder for initial Jacobian estimation using orthogonal motions
        # This should be replaced with actual measurements
        return np.eye(2)

    def capture_positions(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to capture image")

        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges for end-effector and target
        end_effector_color_lower = np.array([30, 150, 50])
        end_effector_color_upper = np.array([50, 255, 255])
        target_color_lower = np.array([0, 150, 50])
        target_color_upper = np.array([10, 255, 255])

        # Create masks for end-effector and target
        end_effector_mask = cv2.inRange(hsv, end_effector_color_lower, end_effector_color_upper)
        target_mask = cv2.inRange(hsv, target_color_lower, target_color_upper)

        # Find contours for end-effector and target
        contours_end_effector, _ = cv2.findContours(end_effector_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_target, _ = cv2.findContours(target_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Get the largest contour for end-effector and target
        if contours_end_effector:
            largest_contour_end_effector = max(contours_end_effector, key=cv2.contourArea)
            M = cv2.moments(largest_contour_end_effector)
            end_effector_pos = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            end_effector_pos = np.array([0, 0])

        if contours_target:
            largest_contour_target = max(contours_target, key=cv2.contourArea)
            M = cv2.moments(largest_contour_target)
            target_pos = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            target_pos = np.array([0, 0])

        return np.array(end_effector_pos), np.array(target_pos)

    def compute_error(self, current_pos, desired_pos):
        return desired_pos - current_pos

    def broyden_update(self, J, delta_x, delta_y):
        delta_y = delta_y.reshape(-1, 1)
        delta_x = delta_x.reshape(-1, 1)
        J += (delta_y - J @ delta_x) @ delta_x.T / (delta_x.T @ delta_x)
        return J

    def run(self):
        while True:
            end_effector_pos, target_pos = self.capture_positions()
            error = self.compute_error(end_effector_pos, target_pos)
            if np.linalg.norm(error) < self.stop_threshold:
                print("Target reached.")
                break
            delta_y = error
            delta_x = np.linalg.pinv(self.J) @ delta_y
            base_angle, joint_angle = delta_x
            self.server.sendAngles(base_angle, joint_angle, self.queue)
            reply = self.queue.get()
            print("Robot Reply:", reply)
            new_end_effector_pos, _ = self.capture_positions()
            new_error = self.compute_error(new_end_effector_pos, target_pos)
            delta_y_new = new_error - error
            self.J = self.broyden_update(self.J, delta_x, delta_y_new)
            time.sleep(1)
            
        

print("Tracker Setup")
tracker = Tracker('g', 'r')
print("Moving on")
while True:
    print("Point is at: "+str(tracker.point))
    print("Goal is at: "+str(tracker.goal))
    time.sleep(2)