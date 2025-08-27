Kalibr provides limited, experimental support for spatio-temporal laser range finder (LRF) calibration as detailed on in [[1](#references)].

### Usage
This work marks an _extension_ to camera/IMU calibration. Accordingly, it extends the command line interface with a few LRF specific options, while all camera and IMU options remain valid (please see [here](Multi-IMU-and-IMU-intrinsic-calibration#3-running-the-calibration) and [here](Camera-IMU-calibration#3-running-the-calibration) for these options).

These additional options are
* **--lrf-topic** The topic of the ROS scan message.
* **--q_bl** An initial guess for the rotation from the laser into the body frame as quaternion.
* **--t_bl** The corresponding, relative translation.

Note that the initial guess needs to be sufficiently accurate to yield an initial point cloud that allows for automatic identification of planes. Please consider [[2](#references)] for determining an initial guess in case coarse estimates fail to produce plane detections.

The following command spells out the options used to process the [example dataset](#dataset):
```
kalibr_calibrate_imu_camera_laser --cam camchain-calibration_2015-10-09-10-48-02.yaml --imu imu_adis16448_ident.yaml --target april_6x6.yaml --imu-models calibrated --bag calibration_2015-11-05-13-39-04.bag --reprojection-sigma 0.1 --time-calibration --timeoffset-padding 0.1 --lrf-topic /scan --q_bl -0.5 0.5 -0.5  0.5 --t_bl 0. 0. 0.
```

### Dataset
The approach uses planes present in the environment to formulate a probabilistic model of range measurements. For this method to produce correct results, please collect a dataset as described for [camera/IMU calibration](Camera-IMU-calibration#2-collect-images) but additionally in an environment where (preferably multiple) planes are present and mostly unobstructed to the LRF during data collection.

An example dataset can be found [here](https://drive.google.com/file/d/0B4rISk5dxJScOEhXQ3loMUw1SGM/view?usp=sharing). Please note that this dataset was selected for its size, not because it is particularly well suited for the task. For best results, please prefer less cluttered environments and avoid scenarios where subtle non-planarities are present since these may bias the estimate.

### General Comments
* The approach is non-deterministic in the sense that it employs a random selection of points for establishing plane hypotheses. Accordingly, different executions of the toolbox will yield different results. Given that the example dataset provided here is less than optimal, the calibration was observed to fail in a small number of cases due to slow convergence (Not reaching the convergence criterion within a limited number of iterations is considered a failure.). In these cases, rerunning the application may yield appropriate calibrations. 
* The implementation provides visualizations that let you inspect the point cloud for detected planes color coded by the absolute residual of each point.
* Error checks are not in place everywhere. In case the approach fails to detect any planes, it may just crash without further notice.
* The approach exhibits a number of free parameters, mostly in place to allow for automatic plane detection. Since currently no configuration trough a separate file is supported, these will have to be adapted [in the code](https://github.com/ethz-asl/kalibr/blob/experimental/lrf-calibration/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/LrfSensor.py#L48-L56).
* The example employs hardware synchronization using a timestamped trigger. For general use, you will have to adapt [these lines](https://github.com/ethz-asl/kalibr/blob/experimental/lrf-calibration/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/LrfSensor.py#L33-L35).
* Calibration output is limited to [the terminal](https://github.com/ethz-asl/kalibr/blob/experimental/lrf-calibration/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera_laser#L244-L258) at this point.
* The initial guess for the temporal offset between LRF and IMU [is zero](https://github.com/ethz-asl/kalibr/blob/experimental/lrf-calibration/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/LrfSensor.py#L280).
* The extension was implemented specifically with the idea in mind of facilitating research in more complete LRF models. Feel free to extend [the model](https://github.com/ethz-asl/kalibr/blob/experimental/lrf-calibration/aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/LrfSensor.py#L329-L341). 
* The current implementation employs a [two step procedure](https://github.com/ethz-asl/kalibr/blob/experimental/lrf-calibration/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera_laser#L229-L231), estimating an initial set of LRF parameters followed by outlier removal and a second optimization on the inlier set alone. 

## References
Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="general2016"></a>Rehder, Joern, Roland Siegwart, and Paul Furgale. "A General Approach to Spatiotemporal Calibration in Multisensor Systems." IEEE Transactions on Robotics 32.2 (2016): 383-398.
1. <a name="extrinsic2004"></a>Zhang, Qilong, and Robert Pless. "Extrinsic calibration of a camera and laser range finder (improves camera calibration)." Intelligent Robots and Systems, 2004.(IROS 2004). Proceedings. 2004 IEEE/RSJ International Conference on. Vol. 3. IEEE, 2004.