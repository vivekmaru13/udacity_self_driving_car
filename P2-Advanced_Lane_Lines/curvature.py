import numpy as np
import cv2
import matplotlib.pyplot as plt


def fit_by_windows(warped_image, n_windows, margin=100, minpixels=50):
    """

    :param warped_image:
    :param n_windows:
    :param margin:
    :param minpixels:
    :return:
    """
    # Define conversions in x and y from pixels space to real world meters
    ym_per_pix = 30.0 / 720  # meters per pixel in y dimension
    xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

    # histogram
    histogram = np.sum(warped_image[warped_image.shape[0] // 2:, :], axis=0)

    # Find the peak of the left and right halves of the histogram ,
    # starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Create an output image to draw on
    out_img = np.dstack((warped_image, warped_image, warped_image)) * 255

    # Set height of windows
    window_height = np.int(warped_image.shape[0] / n_windows)

    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = warped_image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    for window in range(n_windows):

        # Identify window boundaries in x and y (and right and left)
        win_y_low = warped_image.shape[0] - (window + 1) * window_height
        win_y_high = warped_image.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        # Draw the windows on the visualization image
        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
                      (0, 255, 0), 2)
        cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high),
                      (0, 255, 0), 2)

        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        # If you found > minpixels pixels, recenter next window on their mean position
        if len(good_left_inds) > minpixels:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpixels:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(lefty * ym_per_pix, leftx * xm_per_pix, 2)
    right_fit_cr = np.polyfit(righty * ym_per_pix, rightx * xm_per_pix, 2)

    # Calculate radius of curvature
    y_eval = warped_image.shape[0] - 1  # position at which curvature is calculated
    left_curve_radius = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[
        1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])
    right_curve_radius = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[
        1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])

    curvature = np.mean([left_curve_radius, right_curve_radius])

    # Calculate offset of car assuming camera is mounted at car in center
    offset = xm_per_pix * 0.5 * (warped_image.shape[1] - (leftx_base + rightx_base))

    # ploty = np.linspace(0, warped_image.shape[0] - 1, warped_image.shape[0])
    # left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    # right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    # out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 9))
    # ax1.imshow(warped_image, cmap='gray')
    # ax1.set_title('Binary warped', fontsize=20)
    # ax2.imshow(out_img)
    # ax2.plot(left_fitx, ploty, color='yellow')
    # ax2.plot(right_fitx, ploty, color='yellow')
    # ax2.set_title('Lane lines identified', fontsize=20)
    # plt.savefig('output_images/lines_by_windows.png')
    # plt.show()

    return left_fit, right_fit, curvature, offset


def fit_by_previous(warped_image, left_fit, right_fit):
    """

    :param warped_image:
    :param left_fit:
    :param right_fit:
    :return:
    """
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30.0 / 720  # meters per pixel in y dimension
    xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

    # histogram
    histogram = np.sum(warped_image[warped_image.shape[0] // 2:, :], axis=0)

    # Find the peak of the left and right halves of the histogram ,
    # starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # HYPERPARAMETER
    # Choose the width of the margin around the previous polynomial to search
    # The quiz grader expects 100 here, but feel free to tune on your own!
    margin = 100

    # Grab activated pixels
    nonzero = warped_image.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy +
                                   left_fit[2] - margin)) & (
                              nonzerox < (left_fit[0] * (nonzeroy ** 2) +
                                          left_fit[1] * nonzeroy + left_fit[2] + margin)))
    right_lane_inds = ((nonzerox > (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy +
                                    right_fit[2] - margin)) & (
                               nonzerox < (right_fit[0] * (nonzeroy ** 2) +
                                           right_fit[1] * nonzeroy + right_fit[2] + margin)))

    # Again, extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    left_fit_cr = np.polyfit(lefty * ym_per_pix, leftx * xm_per_pix, 2)
    right_fit_cr = np.polyfit(righty * ym_per_pix, rightx * xm_per_pix, 2)

    # Calculate radius of curvature
    y_eval = warped_image.shape[0] - 1  # position at which curvature is calculated
    left_curve_radius = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[
        1]) ** 2) ** 1.5) / np.absolute(2 * left_fit_cr[0])
    right_curve_radius = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[
        1]) ** 2) ** 1.5) / np.absolute(2 * right_fit_cr[0])

    curvature = np.mean([left_curve_radius, right_curve_radius])

    # Calculate offset of car assuming camera is mounted at car in center
    offset = xm_per_pix * 0.5 * (warped_image.shape[1] - (leftx_base + rightx_base))

    return left_fit, right_fit, curvature, offset


def project_back(Minv, warped, img_undistorted, left_fitx, right_fitx, ploty, curvature, offset):
    """

    :param Minv:
    :param warped:
    :param img_undistorted:
    :param left_fitx:
    :param right_fitx:
    :param ploty:
    :param curvature:
    :param offset:
    :return:
    """
    # Create an image to draw the lines on
    warp_zero = np.zeros_like(warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    img_warp = cv2.warpPerspective(color_warp, Minv,
                                   (img_undistorted.shape[1], img_undistorted.shape[0]))

    # Combine the result with the original image
    result = cv2.addWeighted(img_undistorted, 1, img_warp, 0.3, 0)

    curvature_text = 'Radius of curvature is: ' + str(curvature)
    offset_text = 'Offset from the center: ' + str(abs(offset))
    cv2.putText(result, curvature_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0,
                                                                                0), 2,
                cv2.LINE_AA)
    cv2.putText(result, offset_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                cv2.LINE_AA)

    return result
