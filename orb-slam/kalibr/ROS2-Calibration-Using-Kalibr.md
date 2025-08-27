
Kalibr requires ROS1 to run. If you are on a ROS2 system and do not want to corrupt your environmental variables, we recommend using the [docker container](installation#using-the-docker-images) to run all Kalibr commands. Either using a ROS1 system or the docker will require getting your camera and IMU data into the ROS1 [rosbag](http://wiki.ros.org/rosbag) format. This guide will have you install a helper toolbox called [rosbags](https://gitlab.com/ternaris/rosbags) to handle this conversion. The ROS2 bag must *only* have ROS2 standard types in it (such as Image and IMU standard messages). Please checkout the official documentation and project:

- Documentation: https://ternaris.gitlab.io/rosbags/
- Source Code: https://gitlab.com/ternaris/rosbags



First convert the ROS2 folder (contains the database and metadata file) into the ROS1 bag file.
If you recorded other non-image IMU topics (or non-standard message types) use `--exclude-topic` to not convert them.
See the [official documentation](https://ternaris.gitlab.io/rosbags/topics/convert.html) for what limitations there are.
```
pip3 install rosbags>=0.9.12 # might need -U
rosbags-convert <ros2_bag_folder> --dst calib_01.bag --exclude-topic <non_img_and_imu_topics>
```

From here you can enter your docker (this assumes you have [clone and build](installation#using-the-docker-images) the docker already):
```
FOLDER=$(pwd)
xhost +local:root
docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" kalibr
```

and then run your calibration commands:
```
source devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/calib_01.bag --target /data/april_6x6.yaml \
    --models pinhole-radtan pinhole-radtan \
    --topics /cam0/image_raw /cam1/image_raw
```


To go the opposite way from [bagcreater](bag-format#bagcreater), we can do the following:
```
pip3 install rosbags>=0.9.11
rosbags-convert calib_01.bag --dst <ros2_bag_folder>
```