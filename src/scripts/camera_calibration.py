import numpy as np
import json
import cv2
import os

import glob
import argparse

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def calibrate(dirpath, prefix, image_format, square_size, file_name_save, width=8, height=6):
    """ Apply camera calibration operation for images in the given directory path. """
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    if dirpath[-1:] == '/':
        dirpath = dirpath[:-1]

    images = glob.glob(dirpath+'/' + prefix + '*.' + image_format)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    save_calibration_json(file_name_save,mtx,dist,rvecs,tvecs)
    
    mtx2, dist2, rvecs2, tvecs2 = load_calibration_json(file_name_save)
    print('these are the saved values \n', mtx2)
    print('dist ',dist2)

    return [ret, mtx, dist, rvecs, tvecs]

def load_calibration_json(filename):
    with open(filename, "r") as f:
        data = json.load(f)
    
    mtx = np.array(data["mtx"])
    dist = np.array(data["dist"])
    rvecs = [np.array(r) for r in data["rvecs"]]
    tvecs = [np.array(t) for t in data["tvecs"]]
    
    return mtx, dist, rvecs, tvecs

def save_calibration_json(filename, mtx, dist, rvecs, tvecs):
    data = {
        "mtx": mtx.tolist(),  ## tolist to convert the python array to list without changing the dimension of the array
        "dist": dist.tolist(),
        "rvecs": [r.tolist() for r in rvecs],
        "tvecs": [t.tolist() for t in tvecs]
    }
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)

def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
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

if __name__ == "__main__":

    dir_path = fr'/home/aditya/btp_kortex_ros/src/pkg_1/checker_images'
    prefix = 'img_'
    image_format = 'png'
    square_size = 0.024
    file_name_save = os.path.join(dir_path,'cam_300.json')
    ret, mtx, dist, rvecs, tvecs = calibrate(dir_path,prefix,image_format,square_size,file_name_save,width=8,height=6)
    print('ret ', ret)
    print('matrix ',mtx)
    print('distortion ', dist) #, rvecs, tvecs)

    