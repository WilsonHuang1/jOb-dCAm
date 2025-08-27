### Camera models
Kalibr supports the following projection models:

* **pinhole camera model (pinhole)**[<sup>1</sup>](../wiki/supported-models#references)  <br>
    (_intrinsics vector_: [fu fv pu pv])
* **omnidirectional camera model (omni)**[<sup>2</sup>](../wiki/supported-models#references) <br>
    (_intrinsics vector_: [xi fu fv pu pv])
* **double sphere camera model (ds)**[<sup>3</sup>](../wiki/supported-models#references) <br>
    (_intrinsics vector_: [xi alpha fu fv pu pv])
* **extended unified camera model (eucm)**[<sup>4</sup>](../wiki/supported-models#references) <br>
    (_intrinsics vector_: [alpha beta fu fv pu pv])

The _intrinsics vector_ contains all parameters for the model:

* **fu, fv**: focal-length
* **pu, pv**: principal point
* **xi**: mirror parameter (only omni) 
* **xi, alpha**: double sphere model parameters (only ds) 
* **alpha, beta**: extended unified model parameters (only eucm) 

### Distortion models
Kalibr supports the following distortion models:

* **radial-tangential (radtan)**[<sup>*</sup>](https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a) <br>
    (_distortion_coeffs_: [k1 k2 r1 r2])
* **equidistant (equi)**[<sup>**</sup>](https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html#details)<br>
    (_distortion_coeffs_: [k1 k2 k3 k4])
* **fov (fov)**[<sup>5</sup>](../wiki/supported-models#references)<br>
    (_distortion_coeffs_: [w])
* **none (none)**<br>
    (_distortion_coeffs_: [])

## References
Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="models"></a> J. Kannala and S. Brandt (2006). A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses, IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 28, no. 8, pp. 1335-1340

2. Mei, Christopher, and Patrick Rives. "Single view point omnidirectional camera calibration from planar grids." Proceedings 2007 IEEE International Conference on Robotics and Automation. IEEE, 2007.

3. Usenko, Vladyslav, Nikolaus Demmel, and Daniel Cremers. "The double sphere camera model." 2018 International Conference on 3D Vision (3DV). IEEE, 2018.

4. Khomutenko, Bogdan, Gaëtan Garcia, and Philippe Martinet. "An enhanced unified camera model." IEEE Robotics and Automation Letters 1.1 (2015): 137-144.

5. Devernay, Frederic, and Olivier Faugeras. "Straight lines have to be straight." Machine vision and applications 13.1 (2001): 14-24.
