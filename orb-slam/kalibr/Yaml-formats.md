### Camera-system calibration (camchain.yaml)
This file stores the calibration of the camera intrinsic and extrinsic parameters as well as the spatial and temporal calibration parameters of the IMU with respect to the cameras.

Each camera has the following parameters:

* **camera_model**<br>
    camera projection type (pinhole / omni)
* **intrinsics**<br>
    vector containing the intrinsic parameters for the given projection type. elements are as follows:<br>
    pinhole: [fu fv pu pv] <br>
    omni: [xi fu fv pu pv] <br>
    ds: [xi alpha fu fv pu pv] <br>
    eucm: [alpha beta fu fv pu pv] <br>
    see [Supported models](supported-models) for more information<br>
* **distortion_model**<br>
    lens distortion type (radtan / equidistant)<br>
* **distortion_coeffs**<br>
    parameter vector for the distortion model<br>
    see [Supported models](supported-models) for more information<br>
* **T_cn_cnm1**<br>
    camera extrinsic transformation, always with respect to the last camera in the chain<br>
    (e.g. cam1: T_cn_cnm1 = T_c1_c0, takes cam0 to cam1 coordinates)<br>
* **T_cam_imu**<br>
    IMU extrinsics: transformation from IMU to camera coordinates (T_c_i)<br>
* **timeshift_cam_imu**<br>
    timeshift between camera and IMU timestamps in seconds (t_imu = t_cam + shift)<br>
* **rostopic**<br>
    topic of the camera's image stream
* **resolution**<br>
    camera resolution [width,height]

**Example chain.yaml**
```
cam0:
  camera_model: pinhole
  intrinsics: [461.629, 460.152, 362.680, 246.049]
  distortion_model: radtan
  distortion_coeffs: [-0.27695497, 0.06712482, 0.00087538, 0.00011556]
  T_cam_imu:
  - [0.01779318, 0.99967549,-0.01822936, 0.07008565]
  - [-0.9998017, 0.01795239, 0.00860714,-0.01771023]
  - [0.00893160, 0.01807260, 0.99979678, 0.00399246]
  - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: -8.121e-05
  rostopic: /cam0/image_raw
  resolution: [752, 480]
cam1:
  camera_model: omni
  intrinsics: [0.80065662, 833.006, 830.345, 373.850, 253.749]
  distortion_model: radtan
  distortion_coeffs: [-0.33518750, 0.13211436, 0.00055967, 0.00057686]
  T_cn_cnm1:
  - [ 0.99998854, 0.00216014, 0.00427195,-0.11003785]
  - [-0.00221074, 0.99992702, 0.01187697, 0.00045792]
  - [-0.00424598,-0.01188627, 0.99992034,-0.00064487]
  - [0.0, 0.0, 0.0, 1.0]
  T_cam_imu:
  - [ 0.01567142, 0.99978002,-0.01393948,-0.03997419]
  - [-0.99966203, 0.01595569, 0.02052137,-0.01735854]
  - [ 0.02073927, 0.01361317, 0.99969223, 0.00326019]
  - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: -8.681e-05
  rostopic: /cam1/image_raw
  resolution: [752, 480]
```

### IMU configuration (imu.yaml)
IMUs are configured using a YAML file. Please refer to the example below for details:

**imu.yaml**
```
#Accelerometers
accelerometer_noise_density: 1.86e-03   #Noise density (continuous-time)
accelerometer_random_walk:   4.33e-04   #Bias random walk

#Gyroscopes
gyroscope_noise_density:     1.87e-04   #Noise density (continuous-time)
gyroscope_random_walk:       2.66e-05   #Bias random walk

rostopic:                    /imu0      #the IMU ROS topic
update_rate:                 200.0      #Hz (for discretization of the values above)
```

### Calibration target configuration (target.yaml)
Please refer to the [Calibration targets](calibration-targets) page.
