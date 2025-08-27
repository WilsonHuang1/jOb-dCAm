The extended version of _kalibr_ supports (temporal-)[<sup>1</sup>](#temporal)spatial calibration of sensor suites comprising multiple cameras and multiple IMUs. In addition, it allows for estimating IMU intrinsics as well as the displacement of the accelerometer y- and z-axis with respect to its x-axis.

This page documents changes with respect to the default camera-IMU calibration. Please make sure to read the respective [documentation](Camera-IMU-calibration) first. To perform multi-IMU or single IMU calibration you will need at least one camera (aiding sensor).

## How to use it

### 1) Requirements
Applies analogously to [here](Camera-IMU-calibration#1-requirements).

### 2) Dataset Collection
Applies analogously to [here](Camera-IMU-calibration#2-collect-images).

### 3) Running the calibration
Please see [here](Camera-IMU-calibration#3-running-the-calibration) for the required options governing camera configuration as well as ROS-bag and target selection.

Options specific to the extended framework are
* **--imu IMU_YAMLS [IMU_YAMLS ...]**<br>
 This option now accepts a list of yaml files, one for each IMU comprised in the sensor suite. The first IMU will be the reference IMU (IMU0).
* **--imu-models IMU_MODELS [IMU_MODELS ...]**<br>
This option holds a list of IMU models of the same length as the list provided to **--imu**. Currently supported models are *calibrated*, *scale-misalignment* and *scale-misalignment-size-effect*. The default is *calibrated*, which will also be assumed when no model is provided. For more details on the models, please see [3](#joern1). 

New optional  arguments are:
* **--imu-delay-by-correlation**<br>
In case your set of IMUs is not perfectly synchronized, the framework is able to determine a temporal offset of each IMU with respect to IMU0. However, the limitations documented [here](#temporal) apply.
* **--reprojection-sigma REPROJECTION_SIGMA**<br>
This option allows for providing an estimated uncertainty in pixels as to how accurate the features of the calibration target could be localized within the calibration images.
* **--recompute-camera-chain-extrinsics**<br>
This option enables the estimation of camera chain extrinsics as part of the camera-IMU calibration process. We suspect that datasets suited for camera-IMU calibration are not necessarily optimal for camera chain extrinsic calibration, mostly due to an unbalanced selection of views of the calibration target. Hence, this option is only recommended for assessing issues with the camera chain calibration as a possible source of suboptimal camera-IMU calibration results.
* **--timeoffset-padding TIMEOFFSET_PADDING**<br>
This option is only in effect in conjunction with the option **--time-calibration**. It allows for varying the time offsets padding, bounds within which the estimated camera-IMU temporal offset may vary during calibration without the framework raising an exception. Note that an initial guess for the temporal offset is obtained from correlating absolute angular velocities as perceived independently by camera and IMU. Hence, this padding should not reflect the estimated temporal offset but merely a guess for the bounds within which this offsets will vary during calibration. Increasing this value will carry a runtime penalty. Note that trouble finding an appropriate value likely hints to issues in the dataset.
* **--perform-synchronization**<br>
The most common failure mode of _kalibr_ appears to be incorrect timestamps provided by the user. The framework exclusively employs header timestamps and normally does not apply any corrections in order to preserve the original timing of your system. This option enables the use of the timestamp correction algorithm proposed by [Zhang et al.](#zhang) to correct for jitter and skew in the device clocks of the attached sensors. Please note that this option will **alter your timing**. It will hence void the significance of the estimated temporal offsets for your system and is solely recommended for assessing potential improvements through more adequate system design.

The calibration can be run using:
```
kalibr_calibrate_imu_camera --bag [filename.bag] --cam [camchain.yaml] --imu [imu0.yaml ... imuN.yaml] --imu-models [MODEL0 ... MODELN] --target [target.yaml]
```

Example command of sample bag file ([download it here](downloads)):
```
rosrun kalibr kalibr_calibrate_imu_camera \
	--target april_6x6_80x80cm.yaml \
	--imu imu_adis16448.yaml imu_mtig710.yaml imu_mti100.yaml \
	--imu-models calibrated calibrated calibrated \
	--cam camchain-static_02.yaml \
	--bag dynamic_fish_03.bag
```

### 4) Calibration Output
Again, much of the description [here](Camera-IMU-calibration#4-the-output) applies.

However, there are a couple of small changes:
* **report-imucam-%BAGNAME%.pdf**: The report pdf now contains the calibration summary in text form as well as result plots. Where residuals are plotted, 3 sigma bounds given the *assumed* noise process strengths provided in the respective yaml files or through the option **--reprojection-sigma** are displayed to foster an intuition about the correctness of the noise parameters and models.
* **results-imucam-%BAGNAME%.txt**: The summary of results now also contains results specific to the chosen IMU models. The summary is identical to the one found in the pdf.
* **imu-%BAGNAME%.yaml**: IMU calibration results in YAML format. The content of this file depends on the models chosen via the option **--imu-models**. Naming corresponds to quantities documented in [this paper](#joern1).  

<a name="temporal"></a><sup>1)</sup> Please note that only the fixed temporal offset of the cameras with respect to IMU0 is estimated in a maximum-likelihood fashion. IMUs are assumed to be correctly synchronized with respect to each other. If this is not the case, the option **--imu-delay-by-correlation** will estimate temporal offsets of additional IMUs with respect to IMU0. Note however that this offset is determined from correlating angular velocities in a preceding step and hence will not be as accurate as the maximum-likelihood estimate.

### References
1. <a name="paul1"></a>Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and Spatial Calibration for Multi-Sensor Systems. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Tokyo, Japan.
1. <a name="paul2"></a>Paul Furgale, T D Barfoot, G Sibley (2012). Continuous-Time Batch Estimation Using Temporal Basis Functions. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 2088–2095, St. Paul, MN.
1. <a name="joern1"></a>Joern Rehder, Janosch Nikolic, Thomas Schneider, Timo Hinzmann, Roland Siegwart (2016). Extending kalibr: Calibrating the Extrinsics of Multiple IMUs and of Individual Axes. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden
1. <a name="zhang"></a>Zhang, Li, Zhen Liu, and C. Honghui Xia. (2002). Clock synchronization algorithms for network measurements. In the Proceedings of the IEEE Twenty-First Annual Joint Conference of the IEEE Computer and Communications Societies.