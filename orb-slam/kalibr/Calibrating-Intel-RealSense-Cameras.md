

# Intel RealSense D455: A Complete Tutorial and Discussion

This video walks through the process of performing visual-inertial sensor calibration. This calibration is crucial for downstream applications which try to fuse the two sources of information. The video was recorded in a single session from start to finish, so please use the chapters to skip to the sections which are of interest. The sensor used is the Intel Realsense D455 color camera and internal IMU. The key software used is Kalibr and allan_variance_ros.

[![Youtube Video](https://img.youtube.com/vi/BtzmsuJemgI/0.jpg)](https://www.youtube.com/watch?v=BtzmsuJemgI)


Key software we are going to use:
- kalibr: https://github.com/ethz-asl/kalibr
- allan_variance_ros: https://github.com/ori-drs/allan_variance_ros

Calibration slides (slide 51):
- https://pgeneva.com/downloads/notes/2021_vins_tutorial_v6_split.pdf

We are going to be using the calibration from here (d455 realsense)
- https://github.com/rpng/ar_table_dataset/

Multi-camera calibration guide:
- https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration
- https://github.com/ethz-asl/kalibr/wiki/supported-models

How to calibrate the IMU intrinsics? What is the IMU noise model?
- https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
- https://github.com/ori-drs/allan_variance_ros

Lets use in kalibr to calibrate the IMU and camera jointly:
- https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration

When collecting data need to avoid degenerate directions (e.g. planar).
Excite two axis motion with non-constant accelerations is crucial!
- https://pgeneva.com/downloads/preprints/Yang2019RAL.pdf
- https://pgeneva.com/downloads/preprints/Yang2023TRO.pdf

The different IMU intrinsic models are covered in here:
> Rehder, Joern, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, and Roland Siegwart.
> "Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes."
> In 2016 IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311. IEEE, 2016.
> https://timohinzmann.com/publications/icra_2016_rehder.pdf

How to interpret results?
- https://docs.openvins.com/gs-calibration.html
- Inspect the IMU time dt plot carefully in the PDF report!
- Your accelerometer and gyroscope errors are within their 3-sigma bounds (if not then your IMU noise or the dataset are incorrect)
- Ensure that your estimated biases do not leave your 3-sigma bounds. If they do leave then your trajectory was too dynamic, or your noise values are not good.
- Sanity check your final rotation and translation with hand-measured values.

Example of [bad IMU timestamps](https://github.com/ethz-asl/kalibr/pull/582) where a burst of IMU are send at a rate of 1ms with gaps of 6ms between (caused by sending messages in buffers and directly republishing them).


<img src="https://github.com/ethz-asl/kalibr/assets/2222562/35d0bc93-5a98-455b-902c-461288a17bda"  width="40%" />



# Intel RealSense T265: Live OpenVINS State Estimation Demo

In this video takes from having a sensor, to collecting data, performing calibration, and finally processing that data live with [OpenVINS](https://docs.openvins.com/) to recover a 6dof pose estimate. First we create a launch file for the Intel Realsense T265 sensor, after which we perform calibration. Finally we use the calibration to process data with [OpenVINS](https://docs.openvins.com/) and demo the recovered trajectory.

[![Youtube Video](https://img.youtube.com/vi/rBT5O5TEOV4/0.jpg)](https://www.youtube.com/watch?v=rBT5O5TEOV4)


Key software needed to have built in your ROS workspace
- kalibr: https://github.com/ethz-asl/kalibr
- allan_variance_ros: https://github.com/ori-drs/allan_variance_ros
- open_vins: https://github.com/rpng/open_vins/

Ensure you have realsense driver installed:
- https://www.intelrealsense.com/tracking-camera-t265/
- https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
```bash
sudo apt install ros-noetic-realsense2-camera
sudo apt-get install librealsense2-udev-rules
sudo apt-get install librealsense2-utils
```

Check that you can see your sensor with the following
```bash
rs-enumerate-devices
```

Launch for realsense driver (examples seem to be on ros1-legacy).
Need to ensure unite_imu_method is linear_interpolation!
- https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy
- https://github.com/IntelRealSense/realsense-ros/blob/ros1-legacy/realsense2_camera/launch/rs_t265.launch

Calibration with pinhole-equi camera model and scale-misalignment
- https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration
- https://github.com/ethz-asl/kalibr/wiki/supported-models
- https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration

How to get good calibration?
- Limit motion blur by decreasing exposure time
- Publish at high-ish framerate (20-30hz)
- Publish your inertial reading at a reasonable rate (200-500hz)

The different IMU intrinsic model are described in:
> Rehder, Joern, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, and Roland Siegwart.
> "Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes."
> In 2016 IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311. IEEE, 2016.
> https://timohinzmann.com/publications/icra_2016_rehder.pdf


How to run this with OpenVINS with live data?
- https://docs.openvins.com/gs-installing.html
- https://github.com/rpng/open_vins/pull/337
- https://github.com/rpng/open_vins/blob/master/ov_msckf/launch/subscribe.launch

Need to create configuration file:
- Need the IMU noises and intrinsics
- Copy over the camera intrinsics and calibration

Now we can run the following commands:
```bash
rviz -d ./src/open_vins/ov_msckf/launch/display.rviz
roslauch ov_msckf subscribe.launch config:=path_estimator.yaml
```
