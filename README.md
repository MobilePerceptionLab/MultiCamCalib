# Multi Camera Calibration

1. 'modules/camera_calibration/app/camera_calibration.cpp' is for calculating extrinsic parameter of each individual cameras.
2. 'modules/camera_calibration/app/HandEyeCalibration.cpp' is the implmentation of multi-camera hand-eye calibration algorithm.

## Usage
1. Pre-calibrate the intrinsic parameters of each camera with arbitary open-sourced methods, e.g. rosrun camera_calibration cameracalibrator.py ...
2. Use modified config file (e.g parameter/AGV_calib/extrinsic_17023550.xml), change the input path, pattern size, number of corners, number of images to be extrinsically calibrated and intrinsics. Select the options of using pre-computed camera model or not, and moving cameras or calibration target.
3. Run: `./camera_calibration settingFilePath SavePath`, rename the `TrajectoryByCV.txt` as you wish.
4. Formulate the captured trajectory from tracking system as `timestamp tx ty tz qx qy qz qw` into file `TrajectoryByGT.txt`:
   ```
    double timestamp; // second
    double tx ty tz; // give the position
    double qx qy qz qw; // give the orientation in quaternion format
   ```
5. Modify the config file (e.g parameter/multi_extrinsic_handeye.yaml). 
   ```
    int NumOfMeasures; // How many measurements will be used for calibration
    bool UseMultiCam; // Use our proposed method or existing single hand-eye/robot-world algorithms
    bool HandeyeSolver; // Type of Solver: Shah[0] Li[1]
   ```
6. Run: `./handeye_camera_calibration settingFilePath GTFilePath_1 TrajectoryFile_1 GTFilePath_2 TrajectoryFile_2 ...`

## Video
[Experiments Video](coming soon)

## References

<a id="1">[1]</a>
Yifu Wang*, Wenqing Jiang*, Kun Huang, S oren Schwertfeger and Laurent Kneip. "Accurate calibration ofmulti-perspective cameras from a generalization of the hand-eye constraint" ,accepted by the 2022 IEEE International conference on robotics and automation (ICRA).
