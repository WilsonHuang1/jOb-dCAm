The validation tool extracts calibration targets on ROS image streams and displays the image overlaid with the reprojections of the extracted corners. Further the reprojection error statistics are calculated and displayed for mono and inter-camera reprojection errors.

The tool must be provided with a camera-system calibration file and a configuration for the calibration target. The output YAML of the multi-camera calibrator can be used as the camera-system configuration.

Usage:
> kalibr_camera_validator --cam camchain.yaml --target target.yaml

![mono](https://user-images.githubusercontent.com/2222562/166345168-362ba2c4-3f30-4b1a-8f8c-a49087fe0245.png)
![stereo](https://user-images.githubusercontent.com/2222562/166346133-c1b3e678-e660-490c-addf-2740661ab6ca.png)

