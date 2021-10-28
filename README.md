TBD
# Multi Camera Calibration

1. 'modules/camera_calibration/cv_calib' is the conventional camera calibration module.
2. 'modules/camera_calibration/dbscan' is the DBSCAN cluster algorithm.
3. 'modules/camera_calibration/event' is the base classes for events.
4. 'modules/camera_calibration/event_camera_calib' is the proposed event camera calibration algorithm.

## Usage
1. Store events in binary file, an event is represented as
   ```
    double timestamp; // second
    double x;
    double y;
    bool polarity;
   ```
   There is an example in modules/camera_calibration/event/tool/txt2bin.cpp.
2. Modify the config file. There is an example in parameter/event_calibration/example.yaml.
3. Run: `unit_test_eventCameraCalib configFilePath binFilePath SaveDirPath`

## Video
[Experiments Video](https://www.bilibili.com/video/BV1ey4y1j7Ke)

## References

<a id="1">[1]</a>
Kun Huang, Yifu Wang, and Laurent Kneip. "Dynamic Event Camera Calibration." 2021 IEEE International Conference on
Intelligent Robots and Systems (IROS). IEEE, 2021.
