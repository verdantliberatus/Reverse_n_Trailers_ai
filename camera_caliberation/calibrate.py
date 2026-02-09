import cv2
import numpy as np

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0)
# (points in the 3D space)
chessboard_size = (9, 6)  # Change this to your chessboard pattern size
square_size = 1  # Use real measurements if available

objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# List of calibration images
import os
directory = "./camera_caliberation/caliberation_photos"
images = os.listdir(directory)



for image_file in images:
    img = cv2.imread(directory+"/"+image_file)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None, flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
    print(ret)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# Calibrate the camera
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


for image_file in images:
    img = cv2.imread(directory+"/"+image_file)
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs)
    cv2.imwrite("camera_caliberation/undistorted_images/"+image_file, undistorted_img)

cv_file = cv2.FileStorage("camera_caliberation/calibration.yaml", cv2.FILE_STORAGE_WRITE)
cv_file.write("camera_matrix", camera_matrix)
cv_file.write("dist_coeff", dist_coeffs)
cv_file.release()