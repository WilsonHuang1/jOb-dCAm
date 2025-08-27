
## Sample datasets

| Bag File | Configs | Description |   
|:-:|:-:|:--|
| [Multi-CAM](https://drive.google.com/file/d/1H59OnBmVrzMWR9yHfFwVWJwzViP9AX-4/view?usp=sharing) | [april_6x6.yaml](https://drive.google.com/file/d/1wsa9oL2kTLwjWQbkATUDyki1R06-14EY/view?usp=sharing) | Sample calibration bag from the EuRoC MAV dataset and configurations for the multiple-camera calibrator |
| [IMU-CAM](https://drive.google.com/file/d/1yrrXtaJdjLr55qasihmM9pQe205slNxm/view?usp=sharing) | [imu_adis16448.yaml](https://drive.google.com/file/d/1_AcSjfgfmqNF_mAA1vJrZVaMuw-BJUdF/view?usp=sharing)<br/>[cam_april-camchain.yaml](https://drive.google.com/file/d/1B33bw5d5QIdIjz1QgspVtzNiWTGFaJNk/view?usp=sharing) | Sample calibration bag from the EuRoC MAV dataset and configurations for the imu-camera calibrator |
| [Multi-IMU](https://drive.google.com/file/d/1HtCBseYRVIZ6AuVydNacpCkR6QS4ucs_/view?usp=sharing) | [april_6x6_80x80cm.yaml](https://drive.google.com/file/d/1amrf8m8Kpnshysf9gzzKYf6nj2K3Lkjp/view?usp=sharing)<br/> [camchain-static_02.yaml](https://drive.google.com/file/d/1orPJtcMLJPyLqtLU4djEcIBsByPJXIaK/view?usp=sharing)<br/> [imu_adis16448.yaml](https://drive.google.com/file/d/1LI5yEVNOKrucHMS5sQYDlpUzhrx8pfMb/view?usp=sharing)<br/>[imu_mti100.yaml](https://drive.google.com/file/d/1Z0xwinH3cq6ObAtFPMchGWbyk156heWO/view?usp=sharing)<br/>[imu_mtig710.yaml](https://drive.google.com/file/d/1GtT3z8CKFixe-akZjThfykU1uClTTFix/view?usp=sharing)| Three IMU and stereo camera recording. Can be used to test the multi-IMU calibration ability. A picture of the rig can be found in [this](https://pgeneva.com/downloads/papers/Eckenhoff2019ICRAb.pdf) paper for those interested. |
| [Rolling Shutter](https://drive.google.com/file/d/1_Dx6FdzAqLc-zNY0rEoBapr9SkVBBuE-/view?usp=sharing) | [april_6x6_80x80cm.yaml](https://drive.google.com/file/d/1co-9NCCk-LbwC2S0WVR7FpM4hU8yb0cL/view?usp=sharing) | Recording of a ELP-960P2CAM-V90-VC USB 2.0 rolling shutter stereo pair recorded using [this](https://github.com/mdkennedy3/elp-synchronized-stereo-camera-ros-pkg) ROS package. There is no IMU data. |

A folder with all calibration files and scripts can be found [here](https://drive.google.com/drive/folders/1rz07gY2Mbmci5xpJEvKKRnPX8VcMijFN?usp=sharing).

## Calibration targets

| Name | Target | Config |
|:-|:-:|:-:|
| Aprilgrid 6x6 0.8x0.8 m (A0 page) | [pdf](https://drive.google.com/file/d/14dY7z8pDb2iEBdveTviDXsoi5H9AaQP1/view?usp=sharing) | [yaml](https://drive.google.com/file/d/1zXfr48_OY0RafwJalBLjqkqgnme-r7Gd/view?usp=sharing) |
| Aprilgrid 6x6 & Checkerboard 7x6 0.5x0.5 m (A0 page) | ~~[pdf]()~~ |  ~~[checker-yaml]()<br> [april-yaml]()~~ |
| Aprilgrid 6x6 0.8x0.8 m (unscaled) | ~~[pdf]()~~ | ~~[yaml]()~~ |
| Aprilgrid 6x6 0.5x0.5 m (unscaled) | ~~[pdf]()~~ | ~~[yaml]()~~ |
| Checkerboard 7x6 0.5x0.5 m (unscaled) | ~~[pdf]()~~ | ~~[yaml]()~~ |

Kalibr provides a [script](calibration-targets) to generate custom targets.

## IMU configurations

| Name | Description |   
|:-:|:--|
| [ADIS 16448](https://drive.google.com/file/d/1oucvH3FABHPUmzkyEH3rX8n4lp8_AH8P/view?usp=sharing) | IMU configuration for the  Analog Devices ADIS16448 IMU |

How to recover IMU noise parameters is covered by the [inertial noise model](IMU-Noise-Model) page.

