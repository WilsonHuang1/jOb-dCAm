The _rolling shutter calibration_ tool provides full intrinsic calibration (projection, distortion and shutter parameters) of rolling shutter cameras [1]. The codebase currently only supports a single camera, and *cannot* calibrate an IMU alongside it.


## How to use?

### 1) Collect rosbag / images
Create a ROS bag containing the raw image data either by directly recording a rosbag from a ROS sensor stream or by using the _[bagcreater](bag-format)_ script on a sequence of image files.

The camera system is fixed and the calibration target is moved in front of the cameras to obtain the calibration images. 

### 2) Running the calibration


* **--model pinhole-equi-rs**<br>
    he camera model to estimate. Currently supported models are `pinhole-radtan-rs`, `pinhole-equi-rs`, `omni-radtan-rs`<br>
* **--inverse-feature-variance 1**<br>
    Estimated inverse variance of the feature detector.<br>
* **--frame-rate [FRAMERATE]**<br>
    Approximate framerate of the camera.<br>


The calibration can be run using:
```
rosrun kalibr kalibr_calibrate_rs_cameras 
    --bag [filename.bag] \
    --model [MODEL_0 ... MODEL_N] \
    --target [target.yaml] \
    --topic  [TOPIC_0 ... TOPIC_N]  \
    --inverse-feature-variance 1 \
    --frame-rate [FRAMERATE]
```


Example command of sample bag file ([download it here](downloads)):
```
rosrun kalibr kalibr_calibrate_rs_cameras \
	--model pinhole-radtan-rs \
	--target april_6x6_80x80cm.yaml \
	--topic /img_pub/camera/image/left \
	--bag ELP_rs_stereo.bag \
	--inverse-feature-variance 1 \
	--max-iter 10 \
	--frame-rate 30
```

### 3) The output
The calibration will be printed to the screen.

## References
Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="othlu"></a>L. Oth, P. Furgale, L. Kneip, R. Siegwart (2013). Rolling Shutter Camera Calibration, In Proc. of the IEEE Computer Vision and Pattern Recognition (CVPR)

