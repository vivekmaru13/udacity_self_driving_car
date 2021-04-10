import cv2
import numpy as np
import os.path as path
import glob


def calibrate(images_dir):
    """
    Calibrate the camera given a directory containing calibration chessboard images.

    :param images_dir: directory containing chessboard frames
    :return: calibration parameters
    """

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0), The image is 9*6.
    objp = np.zeros((9 * 6, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    obj_points = []
    img_points = []

    images = glob.glob(path.join(images_dir, 'calibration*.jpg'))

    for image in images:
        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

        if ret:
            obj_points.append(objp)
            img_points.append(corners)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points,
                                                       gray.shape[::-1],
                                                       None,
                                                       None)

    return ret, mtx, dist, rvecs, tvecs


def correct_distortion(image, mtx, dist):
    """
    Undistort an image given camera matrix and distortion coefficients.

    :param image: input frame
    :param mtx: camera matrix
    :param dist: distortion coefficients
    :return: undistorted image
    """
    image_undistorted = cv2.undistort(image, mtx, dist, newCameraMatrix=mtx)

    return image_undistorted
