![VI-Sensor](https://raw.githubusercontent.com/wiki/ethz-asl/kalibr/images/visensor.png)

This page will guide you through the calibration of the VI-Sensor (visual-inertial sensor). The intrinsics and extrinsics of the camera system and the transformation of each camera w.r.t. the IMU will be estimated.

More information about the VI-Sensor can be found [here](http://www.skybotix.com/).

## Procedure

1. prepare the sensor
1. setting the focus
1. collect calibration data
    1. in-/extrinsics calibration (static calibration)
    1. imu-camera calibration (dynamic calibration)
1. run the calibration
    1. in-/extrinsic camera calibration
    1. imu-camera calibration
1. collect results


## Requirements

* ROS sensor driver is running (image/imu data)
* good Aprilgrid target ([pdf](https://drive.google.com/file/d/0B0T1sizOvRsUdjFJem9mQXdiMTQ/edit?usp=sharing), [yaml](https://drive.google.com/file/d/0B0T1sizOvRsUU2lGMTdWYWhPaWc/edit?usp=sharing))
* Siemens star (or similar camera focus test pattern)
* IMU configuration for ADIS16448 ([yaml](https://drive.google.com/file/d/0B0T1sizOvRsUSk9ReDlid0VSY3M/edit?usp=sharing))


## 1) Sensor preparation

1. make sure the sensor publishes all image and imu streams to ROS
1. minimize the motion blur with a good light source and by reducing the shutter times.

    Shutter times can be set using the following commands:

    >rosrun dynamic_reconfigure dynparam set /visensor_node "{'cam0_agc_enable': 0, 'cam0_aec_enable': 0, 'cam0_coarse_shutter_width': 300}"<br>
    >rosrun dynamic_reconfigure dynparam set /visensor_node "{'cam1_agc_enable': 0, 'cam1_aec_enable': 0, 'cam1_coarse_shutter_width': 300}"

    Observe the result on an image window and tweak the shutter until you get a good image:

    > rosrun image_view image_view image:=/cam0/image_raw &<br>
rosrun image_view image_view image:=/cam1/image_raw &


## 2) Setting the focus

1. point the cameras on a Siemens star (or similar pattern)
1. start the focus tool
    >kalibr_camera_focus --topic /cam0/image_raw /cam1/image_raw

1. set the focus of both cameras by:
    1. reducing the interference visible around the center of the Siemens star
    1. minimizing the focus measure provided by the tool

Make sure a Teflon band or thread-locking glue prevents unintentional focus changes after this step.


## 2) Collect calibration data
In this step we need to collect two calibration datasets with the following properties:

1. **static dataset (in-/extrinsic calibration of the cameras)**
    * attach the sensor somewhere and move the target
    * limit the camera streams to ~4 Hz
    * make sure to cover the entire field of view of the camera
    * use skewed views and varying distances to the calibration target

    view images with:
    >rosrun image_view image_view image:=/cam0/image_raw &<br>
rosrun image_view image_view image:=/cam1/image_raw &

    record bag with:
    >rosbag record /cam0/image_raw  /cam1/image_raw -O static.bag


1. **dynamic dataset (spatial camera-imu calibration)**
    * move the sensor (target is fixed)
    * cameras should run at 20 Hz and IMU at 200 Hz
    * try to excite all rotation and acceleration axes of the IMU
    * avoid shocks (e.g. while picking up the sensor)
    * good illumination and shutter times are crucial here (to avoid motion blur while exciting the IMU)

    view images with:
    >rosrun image_view image_view image:=/cam0/image_raw &<br>
rosrun image_view image_view image:=/cam1/image_raw &

    record bag with:
    >rosbag record /cam0/image_raw  /cam1/image_raw /imu0 -O dynamic.bag

## 3) Run the calibration
1. calibration of camera in/extrinsics
    1. run calibration
    > kalibr_calibrate_cameras --models pinhole-equi pinhole-equi --topics /cam0/image_raw /cam1/image_raw --bag static.bag --target aprilgrid_6x6.yaml

    1. inspect the result plots
    1. verify calibration on the live image stream<br>
       reprojection errors should be in a normal range (0.1-0.2 px for a good calibration)

    > kalibr_camera_validator --chain chain.yaml --target aprilgrid_6x6.yaml


1. camera-imu calibration
    1. run calibration

    > kalibr_calibrate_imu_camera --cam chain.yaml --target aprilgrid_6x6.yaml --imu imu0.yaml --bag dynamic.bag

    1. inspect the result plots
        * make sure the predicted accelerations & angular velocities fit the IMU measurements
        * reprojection errors should be in a normal range (0.1-0.2 px for a good calibration)

## 4) Collect results
Both calibrators write reports to the working directory containing the plots shown at the end of the calibration. Further a _camchain.yaml_ has been written by the camera calibrator and is extended by the imu-camera calibrator with imu-camera transformations to the file _camchain_cimu.yaml_. Please refer to the [YAML formats](yaml-formats) page for the format.
