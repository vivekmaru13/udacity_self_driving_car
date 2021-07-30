### Project-2 -  Advanced Lane Finding Project
---

[//]: # (Image References)

[image1]: ./output_images/test_calibration_after.jpg "After Calibration"
[image2]: ./output_images/test_calibration_before.jpg "Before Calibration"
[image3]: ./output_images/perspective_transform.png "Perspective transform"
[image4]: ./output_images/final.png "Final Image"
[video1]: ./output_videos/project_video_output.mp4 "Video"
[video2]: ./output_videos/project_video_output_v2_45.mp4 "Video2"

This project contains multiple steps which are as below.
* Camera calibration: Using a set of given Chessboard images, Compute the camera calibration matrix and distortion coefficients.
* Distortion Correction: Correct distortion to given set of images.

<table style="width:100%">
  <tr>
    <th>
      <p align="center">
           <img src="./output_images/test_calibration_before.jpg" alt="calibration_before" width="60%" height="60%">
           <br>Test image before calibration
      </p>
    </th>
    <th>
      <p align="center">
           <img src="./output_images/test_calibration_after.jpg" alt="calibration_after" width="60%" height="60%">
           <br>Test image after calibration
      </p>
    </th>
  </tr>
</table>

* Thresholding : Create a threshold binary image using gradients or color transformation or combination of both.
* Perspective Transform: Create bird-eye view like image by applying perspective transform.

![Perspective transform][image3]
* Lane lines boundary: Detect and fit to find the lane boundary using lane pixels.
* Curvature and reverse wrap: Determine the curvature of the lane and vehicle position with respect to center of the road and warp the detected lane boundaries back onto the original image.

![Final image][image4]

---
#### Pipeline Videos

* Pipeline Video: Please find the main project [video][video1]
* Extra pipeline [video][video2] with shadows and curvature