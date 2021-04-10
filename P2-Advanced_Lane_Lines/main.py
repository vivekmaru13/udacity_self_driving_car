import collections

import numpy as np
from moviepy.editor import VideoFileClip

from calibration import calibrate, correct_distortion
from curvature import (
    fit_by_windows,
    fit_by_previous,
    project_back
)
from perspective_transform import perspective_transform
from thresholding import (
    mag_thresh,
    s_select,
    abs_sobel_thresh
)

counter = 0
past_frames_left = collections.deque(maxlen=10)
past_frames_right = collections.deque(maxlen=10)
pipe_last_fit_left = []
pipe_last_fit_right = []


def image_pipeline(image_frame):
    """

    :param image_frame:
    :return:
    """
    global counter, pipe_last_fit_left, pipe_last_fit_right

    # values of fits of the line for previous frame

    # polynomial coefficients averaged over the last n iterations
    pipe_best_fit_left = None
    pipe_best_fit_right = None

    # 1 undistort image
    img_undistorted = correct_distortion(image_frame, mtx, dist)

    # 2 thresholding
    kernel_size = 5
    sobel_binary = abs_sobel_thresh(img_undistorted, orient='x', sobel_kernel=kernel_size,
                                    thresh=(20, 100))
    mag_binary = mag_thresh(img_undistorted, sobel_kernel=kernel_size, mag_thresh=(50, 100))
    s_binary = s_select(img_undistorted, thresh=(150, 255))

    # create binary file
    combined = np.zeros_like(sobel_binary)
    combined[(((sobel_binary == 1) & (mag_binary == 1)) | (s_binary == 1))] = 1

    # 3 Perspective transform
    warped, M, Minv = perspective_transform(combined)

    # 4 find the curvature
    if counter < 5000000:
        left_fit, right_fit, curvature, offset = fit_by_windows(warped_image=warped, n_windows=20)
        past_frames_left.append(left_fit)
        past_frames_right.append(right_fit)
        pipe_last_fit_left = left_fit
        pipe_last_fit_right = right_fit
        pipe_best_fit_left = np.mean(past_frames_left, axis=0)
        pipe_best_fit_right = np.mean(past_frames_right, axis=0)

        ploty = np.linspace(0, warped.shape[0] - 1, warped.shape[0])
        left_fitx = pipe_best_fit_left[0] * ploty ** 2 + pipe_best_fit_left[1] * ploty + pipe_best_fit_left[2]
        right_fitx = pipe_best_fit_right[0] * ploty ** 2 + pipe_best_fit_right[1] * ploty + pipe_best_fit_right[2]
        out = project_back(Minv, warped, img_undistorted, left_fitx, right_fitx, ploty, curvature,
                           offset)
    else:
        left_fit, right_fit, curvature, offset = fit_by_previous(
            warped_image=warped,
            left_fit=pipe_last_fit_left,
            right_fit=pipe_last_fit_right
        )

        past_frames_left.append(left_fit)
        past_frames_right.append(right_fit)
        pipe_best_fit_left = np.mean(past_frames_left, axis=0)
        pipe_best_fit_right = np.mean(past_frames_right, axis=0)

        ploty = np.linspace(0, warped.shape[0] - 1, warped.shape[0])
        left_fitx = pipe_best_fit_left[0] * ploty ** 2 + pipe_best_fit_left[1] * ploty + pipe_best_fit_left[2]
        right_fitx = pipe_best_fit_right[0] * ploty ** 2 + pipe_best_fit_right[1] * ploty + pipe_best_fit_right[2]
        out = project_back(Minv, warped, img_undistorted, left_fitx, right_fitx, ploty, curvature,
                           offset)

    counter += 1

    return out


def main():
    """

    :return:
    """
    global mtx, dist, past_frames_left, past_frames_right

    # calibrate camera
    ret, mtx, dist, rvecs, tvecs = calibrate(images_dir='camera_cal')

    # To save images for output
    # image = mpimg.imread('test_images/test1.jpg')
    # out = image_pipeline(image)
    # f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
    # ax1.imshow(image)
    # ax1.set_title('Original Image', fontsize=15)
    # ax2.imshow(out, cmap='gray')
    # ax2.set_title('Inference Image', fontsize=15)
    # # plt.savefig('output_images/final.png')
    # plt.show()

    output = 'project_video_output_v2_45.mp4'
    clip = VideoFileClip("harder_challenge_video.mp4")

    white_clip = clip.fl_image(image_pipeline)

    white_clip.write_videofile(output, audio=False)


if __name__ == '__main__':
    main()
