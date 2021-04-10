import numpy as np
import cv2


def perspective_transform(img):
    """

    :param img:
    :return:
    """
    h, w = img.shape[:2]

    src = np.float32([[588, 470], [245, 719], [1142, 719], [734, 470]])
    dst = np.float32([[320, 0], [320, 720], [960, 720], [960, 0]])

    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)

    warped = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)

    return warped, M, Minv


# Code to output images for writeup

# import matplotlib.pyplot as plt
# import matplotlib.image as mpimg
#
# image = mpimg.imread('test_images/test3.jpg')
# warped, M, Minv = perspective_transform(image)
#
# src = np.float32([[588, 470], [245, 719], [1142, 719], [734, 470]])
# dst = np.float32([[320, 0], [320, 720], [960, 720], [960, 0]])
#
# pts1 = np.array(src, np.int32)
# pts1 = pts1.reshape((-1, 1, 2))
# cv2.polylines(image, [pts1], True, (255, 0, 0), 3)
#
# pts2 = np.array(dst, np.int32)
# pts2 = pts2.reshape((-1, 1, 2))
# cv2.polylines(warped, [pts2], True, (255, 0, 0), 3)
#
# f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
# ax1.imshow(image)
# ax1.set_title('Original Image', fontsize=15)
# ax2.imshow(warped, cmap='gray')
# ax2.set_title('Undistorted Image', fontsize=15)
# # plt.savefig('output_images/perspective_transform.png')
# plt.show()
