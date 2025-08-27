The camera-imu calibration tool estimates the spatial and temporal parameters of a camera system with respect to an intrinsically calibrated IMU. Image and IMU data has to be provided in a [ROS](https://www.ros.org) bag. 

The calibration parameters are estimated in a full batch optimization using splines to model the pose of the system. Detailed information about the approach can be found in the following papers: (see [1](#paul1), [2](#paul2))

## Tutorial: IMU-camera calibration
A video tutorial for the IMU-camera calibration can be found here:

[![alt text](https://user-images.githubusercontent.com/5337083/44033014-50208b8a-9f09-11e8-8e9a-d7d6d3c69d97.png)](https://m.youtube.com/watch?v=puNXsnrYWTY "imu cam calib")

(Credits: @indigomega)

## How to use it

### 1) Requirements
The intrinsic parameters of the IMU (e.g. scales, axis misalignment, nonlinearities,...) need to be calibrated beforehand and its correction applied to the raw measurements. If not, one should use an IMU model which calibrates these values.

Further an IMU configuration YAML has to be created containing the following statistical properties for the accelerometers and gyroscopes:

* noise density
* bias random walk

Please refer to the [YAML formats](yaml-formats) page for the data format and the [IMU Noise Model](IMU-Noise-Model) page for the theory.
The simplest way to recover these parameters is to compute an Allan Deviation using the [ori-drs/allan_variance_ros](https://github.com/ori-drs/allan_variance_ros) tool.

### 2) Collect images
Create a ROS bag containing the raw image streams either by directly recording from ROS sensor streams or by using the _[bagcreater](bag-format)_ script on a list of image files and a CSV file containing the IMU measurements.

The calibration target is fixed in this calibration and the camera-imu system is moved in front of the target to excite all IMU axes. It is important to ensure good and even illumination of the calibration target and to keep the camera shutter times low to avoid excessive motion blur.

Good results have been obtained by using a camera rate of 20 Hz and an IMU rate of 200 Hz. 

**Tips:**
* try to excite all IMU axes (rotation and translation)
* avoid shocks, especially at the beginning/end when you pick up the sensor
* keep the motion blur low:
    * low shutter times
    * good illumination 
* ensure low jitter timestamps in same clock

**WARNING:**
If you are using a calibration target with symmetries (checkerboard, circlegrid), movements which could lead to flips in the target pose estimates have to be avoided. The use of an [Aprilgrid](calibration-targets) is recommended to avoid this problem entirely.

### 3) Running the calibration
The tool must be provided with the following input:

* **--bag filename.bag**<br>
    ROS bag containing the image and IMU data<br>
* **--cam camchain.yaml**<br>
    intrinsic and extrinsic calibration parameters of the camera system. The output of the multiple-camera-calibration tool can be used here. (see [YAML formats](yaml-formats))<br>
* **--imu imu.yaml**<br>
    contains the IMU statistics and the IMU's topic (see [YAML formats](yaml-formats))<br>
* **--target target.yaml**<br>
    the calibration target configuration (see [Calibration targets](#calibration-target))

The calibration can be run using:
```
kalibr_calibrate_imu_camera --bag [filename.bag] --cam [camchain.yaml] --imu [imu.yaml] --target [target.yaml]
```

The temporal calibration is turned on by default and can be disabled using the **--no-time-calibration** argument. More information about options is available using the help argument:
```
kalibr_calibrate_imu_camera --h
```

Example command of sample bag file ([download it here](downloads)):
```
rosrun kalibr kalibr_calibrate_imu_camera \
	--target april_6x6.yaml \
	--imu imu_adis16448.yaml \
	--imu-models calibrated \
	--cam cam_april-camchain.yaml \
	--bag imu_april.bag
```

### 4) The output
The calibration will produce the following output files:

* **report-imucam-%BAGNAME%.pdf**: Report in PDF format. Contains all plots for documentation.
* **results-imucam-%BAGNAME%.txt**: Result summary as a text file.
* **camchain-imucam-%BAGNAME%.yaml**: Results in YAML format. This file is based on the input _camchain.yaml_ with added transformations (and optionally time shifts) for all cameras with respect to the IMU. Please check the format on the [YAML formats](yaml-formats) page.


One should make sure to inspect the IMU timestamp DTs. One example of [bad IMU timestamps](https://github.com/ethz-asl/kalibr/pull/582) where a burst of IMU are send at a rate of 1ms with gaps of 6ms between (caused by sending messages in buffers and directly republishing them).

<img src="https://github.com/ethz-asl/kalibr/assets/2222562/35d0bc93-5a98-455b-902c-461288a17bda"  width="70%" />

If you have this, than likely your hardware is messed up and you will run into calibration problems. You will need to rectify the timestamp problems.


## References
Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
