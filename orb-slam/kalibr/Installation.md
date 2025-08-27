
# Using the Docker Images

1. Clone and build the docker image <br/>

First make sure that you have install docker on your system using the official Docker [Get Docker](https://docs.docker.com/get-docker/) guide.
We can then clone and build the docker container using:
```
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
docker build -t kalibr -f Dockerfile_ros1_20_04 . # change this to whatever ubuntu version you want
```

2. Mounting a data folder for use in the container <br/>

We can now mount the data folder in the container `/data` path and enter the command prompt.
Some more details can be found on the [ROS wiki](http://wiki.ros.org/docker/Tutorials/GUI) for Docker.
```
FOLDER=/path/to/your/data/on/host
xhost +local:root
docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" kalibr
```

3. Inside the docker, running commands <br>

Using the above command you should have entered the docker container bash prompt.
From here you should be able to run kalibr on any files that are in your `/data` directory.
You will want to first load your ROS environment variables.
```
source devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/cam_april.bag --target /data/april_6x6.yaml \
    --models pinhole-radtan pinhole-radtan \
    --topics /cam0/image_raw /cam1/image_raw
```




# Building Kalibr from Source
The codebase is built on top of the [Robot Operating System (ROS)](https://www.ros.org/) and has been tested building on Ubuntu 16.04, 18.04, 20.04 systems with ROS Kinetic, Melodic, and Noetic.
We also recommend installing the [catkin_tools](https://github.com/catkin/catkin_tools) build for easy ROS building.
More information on building with catkin and ROS can be found [here](http://wiki.ros.org/catkin/Tutorials).
Please see the official instructions to install ROS:

- [Ubuntu 16.04 ROS 1 Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (uses OpenCV 3.3)
- [Ubuntu 18.04 ROS 1 Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (uses OpenCV 3.2)
- [Ubuntu 20.04 ROS 1 Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (uses OpenCV 4.2)

1. Install ROS 1 on your system <br>

Here are some example commands that will install both the ROS 1 desktop environment and catkin tools.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
export ROS1_DISTRO=noetic # kinetic=16.04, melodic=18.04, noetic=20.04
sudo apt-get install ros-$ROS1_DISTRO-desktop-full
sudo apt-get install python-catkin-tools # ubuntu 16.04, 18.04
sudo apt-get install python3-catkin-tools python3-osrf-pycommon # ubuntu 20.04
```


2. Install the build and run dependencies <br>

The general requirements common to all version of Ubuntu are the following:
```
sudo apt-get install -y \
    git wget autoconf automake nano \
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    doxygen libopencv-dev \
    libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev
```

Then due to different Python versions, you will need to install the following:
```
# Ubuntu 16.04
sudo apt-get install -y python2.7-dev python-pip python-scipy \
    python-matplotlib ipython python-wxgtk3.0 python-tk python-igraph python-pyx
# Ubuntu 18.04
sudo apt-get install -y python3-dev python-pip python-scipy \
    python-matplotlib ipython python-wxgtk4.0 python-tk python-igraph python-pyx
# Ubuntu 20.04
sudo apt-get install -y python3-dev python3-pip python3-scipy \
    python3-matplotlib ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx
```

3. Create a catkin workspace and clone the project<br>

First we can create a workspace.
It is important to configure this to build in *release mode* otherwise optimization will be slow.
```
mkdir -p ~/kalibr_workspace/src
cd ~/kalibr_workspace
export ROS1_DISTRO=noetic # kinetic=16.04, melodic=18.04, noetic=20.04
source /opt/ros/$ROS1_DISTRO/setup.bash
catkin init
catkin config --extend /opt/ros/$ROS1_DISTRO
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

We can then clone the project:
```
cd ~/kalibr_workspace/src
git clone https://github.com/ethz-asl/kalibr.git
```

4. Build the code using the _Release_ configuration.
Depending on the available memory, you might need to reduce the build threads (e.g. add -j2 to catkin_make) <br>

```
cd ~/kalibr_workspace/
catkin build -DCMAKE_BUILD_TYPE=Release -j4
```


5. Once the build is finished you have to source the catkin workspace setup to use Kalibr

```
source ~/kalibr_workspace/devel/setup.bash
rosrun kalibr <command_you_want_to_run_here>
```



# References
Please cite the appropriate papers when using this library or parts of it in an academic publication.

1. <a name="othlu"></a>L. Oth, P. Furgale, L. Kneip, R. Siegwart (2013). Rolling Shutter Camera Calibration, In Proc. of the IEEE Computer Vision and Pattern Recognition (CVPR)
1. <a name="jmaye"></a> J. Maye, P. Furgale, R. Siegwart (2013). Self-supervised Calibration for Robotic Systems, In Proc. of the IEEE Intelligent Vehicles Symposium (IVS)
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="joern1"></a>Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311, Stockholm, Sweden.
