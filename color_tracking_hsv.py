import cv2 as cv
import cv2 as cv2
import numpy as np
redLowMask = (0,160,110)
redHighMask = (60, 255, 255)

#If the circle is blue
blueLowMask = (100, 120, 90)
blueHighMask = (180, 255, 255)

# if the rect is black 
# these need to be double checked TODO
blackLowMask = (0, 0, 0)
blackHighMask = (180, 255, 30)

#If the ball is yellow
yellowLowMask = (30, 50, 70)
yellowHighMask = (35, 160, 255)

tireLowMask = (20, 0, 0)
tireHighMask = (180, 120, 100)

brownLowMask = (10, 15, 160)
brownHighMask = (60, 90, 210)
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
# optional argument for trackbars
def nothing(x):
    pass
def detect_rectangles(frame, color):
        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define color ranges
        if color == 'b':
            mask = cv2.inRange(hsv, blueLowMask, blueHighMask)
        elif color == 'y':
            mask = cv2.inRange(hsv, yellowLowMask, yellowHighMask)
        elif color == 'r':
            mask = cv2.inRange(hsv, redLowMask, redHighMask)
        elif color == 'k':
            mask = cv2.inRange(hsv, blackLowMask, blackHighMask)
        elif color == "Tire":
             mask = cv2.inRange(hsv, tireLowMask, tireHighMask)
        elif color == 'w':
            # brown Tracking
            mask = cv2.inRange(hsv, brownLowMask, brownHighMask)
        else:
            return []

        # Perform erosion and dilation to reduce noise
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=5)
        mask = cv2.dilate(mask, np.ones((6, 6), np.uint8), iterations=4)
        
        
        scale=0.35
        cv.resize(mask, (0, 0), fx=scale, fy=scale)
        cv.imshow("Erosion", cv.resize(mask, (0, 0), fx=scale, fy=scale))
        # Find contours in the masked image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        rectangles = []
        for contour in contours:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # If the polygon has 4 vertices, it is a rectangle
            if len(approx) == 4 or True:
                # Calculate the center point of the rectangle
                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    center = Point(cX, cY)
                    rectangles.append((approx, center))

        return rectangles

def draw_rectangles(frame, rectangles):
    #TODO this is rough and need to be checked
    if rectangles == []:
        return
    for rect in rectangles:
        cv2.drawContours(frame, [rect[0]], -1, (0, 0, 255), 2)



def GetLocation(frame, color): #gets circles
    # Uncomment for gaussian blur
    #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    blurred = cv2.medianBlur(frame,11)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if color == 'r':
        # Red Tracking
        mask = cv2.inRange(hsv, redLowMask, redHighMask)
    if color == 'b':
        # Blue Tracking
        mask = cv2.inRange(hsv, blueLowMask, blueHighMask)
    if color == 'k':
        # Green Tracking
        mask = cv2.inRange(hsv, blackLowMask, blackHighMask)
    if color == 'y':
        # Yellow Tracking
        mask = cv2.inRange(hsv, yellowLowMask, yellowHighMask)
    # Perform erosion and dilation in the image (in 11x11 pixels squares) in order to reduce the "blips" on the mask
    mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=1)
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=3)
    
    
    # Mask the blurred image so that we only consider the areas with the desired colour
    masked_blurred = cv2.bitwise_and(blurred,blurred, mask= mask)
    # masked_blurred = cv2.bitwise_and(frame,frame, mask= mask)
    # Convert the masked image to gray scale (Required by HoughCircles routine)
    result = cv2.cvtColor(masked_blurred, cv2.COLOR_BGR2GRAY)
    
    # Detect circles in the image using Canny edge and Hough transform
    circles = cv2.HoughCircles(result, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=8, minRadius=3, maxRadius=20)
    result = cv.resize(result, (0, 0), fx=0.5, fy=0.5)
    cv.imshow("Circle mask", result)
    return circles #returns (x, y r) of circles


def DrawCircles(frame, circles, dotColor = (255, 0, 0)):
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

# named ites for easy reference
barsWindow = 'Bars'
hl = 'H Low'
hh = 'H High'
sl = 'S Low'
sh = 'S High'
vl = 'V Low'
vh = 'V High'

# set up for video capture on camera 0
cap = cv.VideoCapture(1, cv.CAP_DSHOW)

cap.set(cv.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 2160)

width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
print(f"Resolution set to: {width}x{height}")
scale = 0.35
# create window for the slidebars
cv.namedWindow(barsWindow, flags = cv.WINDOW_AUTOSIZE)

# create the sliders
cv.createTrackbar(hl, barsWindow, 0, 179, nothing)
cv.createTrackbar(hh, barsWindow, 0, 179, nothing)
cv.createTrackbar(sl, barsWindow, 0, 255, nothing)
cv.createTrackbar(sh, barsWindow, 0, 255, nothing)
cv.createTrackbar(vl, barsWindow, 0, 255, nothing)
cv.createTrackbar(vh, barsWindow, 0, 255, nothing)

# set initial values for sliders
cv.setTrackbarPos(hl, barsWindow, 0)
cv.setTrackbarPos(hh, barsWindow, 179)
cv.setTrackbarPos(sl, barsWindow, 0)
cv.setTrackbarPos(sh, barsWindow, 255)
cv.setTrackbarPos(vl, barsWindow, 0)
cv.setTrackbarPos(vh, barsWindow, 255)

while(True):
    ret, frame = cap.read()

    center = frame.shape
    y_center = center[1]/2
    x_center = center[0]/2
    scale = 0.6

    y_low = y_center - y_center*scale
    y_high = y_center + y_center*scale
    x_low = x_center - x_center*scale
    x_high = x_center + x_center*scale
    #frame = frame[int(x_low-100):int(x_high+100), int(y_low+60):int(y_high-60)]
    frame = frame[int(x_low-100):int(x_high+100), int(y_low+100):int(y_high-100)]
    

    # convert to HSV from BGR
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # read trackbar positions for all
    hul = cv.getTrackbarPos(hl, barsWindow)
    huh = cv.getTrackbarPos(hh, barsWindow)
    sal = cv.getTrackbarPos(sl, barsWindow)
    sah = cv.getTrackbarPos(sh, barsWindow)
    val = cv.getTrackbarPos(vl, barsWindow)
    vah = cv.getTrackbarPos(vh, barsWindow)

    # make array for final values
    HSVLOW = np.array([hul, sal, val])
    HSVHIGH = np.array([huh, sah, vah])

    # apply the range on a mask
    mask = cv.inRange(hsv, HSVLOW, HSVHIGH)
    maskedFrame = cv.bitwise_and(frame, frame, mask = mask)

    # display the camera and masked images
    #tire_rectangles = detect_rectangles(frame, "Tire")
    #draw_rectangles(maskedFrame, tire_rectangles)

    goal = detect_rectangles(frame, "w")
    draw_rectangles(frame, goal)
    resizedMasked = cv.resize(maskedFrame, (0, 0), fx=0.5, fy=0.5)
    resizedFrame = cv.resize(frame, (0, 0), fx=0.5, fy=0.5)
    
    cv.imshow("Frame",resizedFrame)

    cv.imshow('Masked', resizedMasked)
    #cv.imshow('Camera', resizedFrame)
    
	# check for q to quit program with 5ms delay
    if cv.waitKey(5) & 0xFF == ord('q'):
        break

# clean up our resources
cap.release()
cv.destroyAllWindows()