#shape of the chessboard of our dataset is 7x10
#load and store calibration parameters
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import os
import numpy as np
import cv2
import glob
import argparse

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def calibrate(path):
    """ Apply camera calibration operation for images in the given directory path. """
    #We prepare object points, like (x,y,z) -> (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:6, 0:9].T.reshape(-1, 2) ## [0. 0. 0.]  [1. 0. 0.]  [2. 0. 0.]  [3. 0. 0.] ....  [0. 1. 0.]  [1. 1. 0.]  .........

    # Arrays to store object points and image points from all the images. They are used in the calibration phase
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane (corners of the image)

    images = glob.glob(path)
    for fname in images:
        img = cv2.imread(fname)
        dim = (640, 360)
        img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners (last parameter can be used for "any flags")
        ret, corners = cv2.findChessboardCorners(gray, (6,9), None)

        # If we find corners in the chessboard, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (6,9), corners2, ret)
            cv2.imshow('img',img)
            cv2.waitKey(50)

    #This function takes the objpoints and imgpoints, and computes the camera matrix and the distortion coefficient that we need to transform 3D images to 2D images.
    #This function returns also the camera position in the world (rotation and translation vectors)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


    return [ret, mtx, dist, rvecs, tvecs]

######https://github.com/opencv/opencv/blob/master/samples/python/calibrate.py
#https://www.programcreek.com/python/example/89320/cv2.calibrateCamera

def save_undistorted(mtx, dist, path_load,path_save):
    i=0
    images = glob.glob(path_load)
    for fname in images:

        img = cv2.imread(fname)
        dim = (640, 360)
        img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        outfile = os.path.join(path_save+str(i) + '_undistorted.jpg')
        i=i+1

        h, w = 360,640
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, dim, 1, dim)

        #This function undistorts the image using the camera matrix and the distortion coefficients
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

        # crop and save the image
        #x, y, w, h = roi
        #dst = dst[y:y+h, x:x+w]

        print('Undistorted image written to: %s' % outfile)
        cv2.imwrite(outfile, dst)


def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    print("Saving the coefficients")
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()


def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]

#python3 cam_calibration.py
if __name__ == '__main__':

    file_coefficients="/coefficients.yaml"
    path_load='./imgs_1/*.jpg'
    path_save='./imgs_saved/'

    #mtx   Matrix to transform 3D objects to 2D image points
    #dist  Distortion coefficient
    #rvecs Rotation vectors
    #tvecs Translation vectors
    ret, mtx, dist, rvecs, tvecs = calibrate(path_load)
    save_coefficients(mtx, dist, file_coefficients)
    #mtx, dist=load_coefficients(file_coefficients)
    print ("Saving images...")
    save_undistorted(mtx,dist,path_load,path_save)
    print("Calibration is finished. RMS: ", ret)
