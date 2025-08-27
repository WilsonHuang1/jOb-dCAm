Kalibr supports three different calibration targets. It is recommended to use the Aprilgrid due to the following benefits:

* partially visible calibration boards can be used
* pose of the target is fully resolved (no flips)

The targets are configured using YAML configuration files which have to be provided to the calibration tools. Please find the configuration templates for the supported targets below.

Grids can be downloaded on the [Downloads](downloads) page or created using the following script:
> kalibr_create_target_pdf --h

### A) Aprilgrid
Partially visible Aprilgrid targets can be detected without problems. This greatly simplifies the data collection and makes this grid the recommended choice.

Aprilgrids can be downloaded on the [Downloads](downloads) page or custom grids can be created using the following script:
> kalibr_create_target_pdf --type apriltag --nx [NUM_COLS] --ny [NUM_ROWS] --tsize [TAG_WIDTH_M] --tspace [TAG_SPACING_PERCENT]

**aprilgrid.yaml**

![kalibr](https://user-images.githubusercontent.com/5337083/41458381-be379c6e-7086-11e8-9291-352445140e88.png)

```
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.088           #size of apriltag, edge to edge [m]
tagSpacing: 0.3          #ratio of space between tags to tagSize
                         #example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]
```

**Make sure to hide all external Apriltags while collecting the calibration dataset.**

Kalibr uses the awsome Apriltag implementation of M. Kaess for tag extraction. [[1](#olson),[2](#kaess)]

### B) Checkerboard
The standard checkerboard pattern is supported using the following configuration:

**checkerboard.yaml**
```
target_type: 'checkerboard' #gridtype
targetCols: 6               #number of internal chessboard corners
targetRows: 7               #number of internal chessboard corners
rowSpacingMeters: 0.06      #size of one chessboard square [m]
colSpacingMeters: 0.06      #size of one chessboard square [m]
```

### C) Circlegrid
The standard OpenCV symmetric and asymmetric circle grids are supported using the following configuration:

**circlegrid.yaml**
```
target_type: 'circlegrid'  #gridtype
targetCols: 6              #number of circles (cols)
targetRows: 7              #number of circles (rows)
spacingMeters: 0.02        #distance between circles [m]
asymmetricGrid: False      #use asymmetric grid (opencv) [bool]
```

## Problems with the target extraction?
The *--show-extraction* argument can be used with the calibration tools to visualize the calibration target extraction process. This might help to find problems with the target configuration and extraction.

## Tips/Problems
* Make sure the calibration target is as flat as possible. Best is to glue it to a rigid plate such as aluminum or acrylic glass.
* Most printers will scale the target during the printing process. Make sure to remeasure the important sizes and adjust the target configuration accordingly.
* Keep a white border around the calibration target of min. the size of one grid element (or the detection might be unsuccessful in certain lighting conditions)

## References
Please cite the appropriate papers when using this toolbox or parts of it in an academic publication.

1. <a name="olson"></a>Edwin Olson (2011). AprilTag: A robust and flexible visual fiducial system. Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pp. 3400–3407
1. <a name="kaess"></a>Michael Kaess. [http://people.csail.mit.edu/kaess/apriltags/](http://people.csail.mit.edu/kaess/apriltags/), Nov. 2013
