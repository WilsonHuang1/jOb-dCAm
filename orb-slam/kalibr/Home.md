![Kalibr](https://raw.githubusercontent.com/wiki/ethz-asl/kalibr/images/kalibr_small.png)

## Introduction
Kalibr is a toolbox that solves the following calibration problems:

1. **Multi-Camera Calibration**: Intrinsic and extrinsic calibration of a camera-systems with non-globally shared overlapping fields of view 
1. **Visual-Inertial Calibration (CAM-IMU)**: Spatial and temporal calibration of an IMU w.r.t a camera-system along with IMU intrinsic parameters
1. **Multi-Inertial Calibration (IMU-IMU)**: Spatial and temporal calibration of an IMU w.r.t a base inertial sensor along with IMU intrinsic parameters (requires 1-aiding camera sensor).
1. **Rolling Shutter Camera Calibration**: Full intrinsic calibration (projection, distortion and shutter parameters) of rolling shutter cameras.
 
To make the calibration task more convenient and reproducible the following tools are available:

1. **Camera Focus**: Tool to set the camera focus in a reproducible way
1. **Calibration Validator**: Validation tool that computes the reprojection error statistics of the calibrated camera-system on live ROS image streams

**For questions or comments, please open an issue on Github.**

## Tutorial: IMU-camera calibration
A video tutorial for the IMU-camera calibration can be found here:

[![alt text](https://user-images.githubusercontent.com/5337083/44033014-50208b8a-9f09-11e8-8e9a-d7d6d3c69d97.png)](https://m.youtube.com/watch?v=puNXsnrYWTY "imu cam calib")

(Credits: @indigomega)

## Tutorial: Start-to-end Guide and Discussion
An informal recording of performing static, IMU noise, and dynamic calibration and a discussion of results.
Full details can be found on the [Calibrating Intel RealSense Cameras](Calibrating-Intel-RealSense-Cameras) page.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/BtzmsuJemgI/0.jpg)](https://www.youtube.com/watch?v=BtzmsuJemgI)

(Credits: @goldbattle)

## Authors
* Paul Furgale
* Jérôme Maye
* Jörn Rehder
* Thomas Schneider ([email](schneith@ethz.ch))
* Luc Oth
* Timo Hinzmann ([email](hitimo@ethz.ch))

## References
The calibration approaches used in Kalibr are based on the following papers. Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="joern1"></a>Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the extrinsics of multiple IMUs and of individual axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 4304-4311, Stockholm, Sweden.
1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
1. <a name="jmaye"></a> J. Maye, P. Furgale, R. Siegwart (2013). Self-supervised Calibration for Robotic Systems, In Proc. of the IEEE Intelligent Vehicles Symposium (IVS)
1. <a name="othlu"></a>L. Oth, P. Furgale, L. Kneip, R. Siegwart (2013). Rolling Shutter Camera Calibration, In Proc. of the IEEE Computer Vision and Pattern Recognition (CVPR)


## License (BSD)
Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland<br>
Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland<br>
All rights reserved.<br>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

1. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

1. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Autonomous Systems Lab and Skybotix AG.

1. Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
