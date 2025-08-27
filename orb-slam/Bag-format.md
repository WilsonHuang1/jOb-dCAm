All applications in Kalibr use ROS bags as a source for the image and IMU data.

### How to use ROS2 bag file format?
Kalibr requires ROS1 bags, thus you need to convert your bags.
You can follow the [ROS2 Calibration Using Kalibr](ROS2-Calibration-Using-Kalibr) guide or convert through the static files using the below bagcreater.

### How to use image files and IMU data as CSV files?
A bagcreater script is provided to use image files and/or IMU data for the calibration

#### bagcreater
This script allows you to create a ROS bag from raw image files and optionally IMU data. The files have to be organized in folders as illustrated below. The example uses a system with two cameras and one IMU:

```
+-- dataset-dir
    +-- cam0
    │   +-- 1385030208726607500.png
    │   +--      ...
    │   \-- 1385030212176607500.png
    +-- cam1
    │   +-- 1385030208726607500.png
    │   +--      ...
    │   \-- 1385030212176607500.png
    \-- imu0.csv
```
The *imu0.csv* file uses the format below: (timestamps=[ns], omega=[rad/s], alpha=[m/s^2])
```
timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
1385030208736607488,0.5,-0.2,-0.1,8.1,-1.9,-3.3
 ...
1386030208736607488,0.5,-0.1,-0.1,8.1,-1.9,-3.3
```

To create the ROS bag run the following command:

> kalibr_bagcreater --folder dataset-dir/. --output-bag awsome.bag

In the example above the data would be written to the following topics:

* /cam0/image_raw
* /cam1/image_raw
* /imu0

#### bagextractor
The bagextractor exports a ROS bag containing image and/or IMU data to image files and an IMU CSV file.

Example usage:
> kalibr_bagextractor --image-topics /cam0/image_raw /cam1/image_raw --imu-topics /imu0 --output-folder dataset-dir --bag awsome.bag

**NOTE:** If you are using the Docker container the _output-folder_ should be relative to the `/data` volume directory which is mounted of launch of the container.
