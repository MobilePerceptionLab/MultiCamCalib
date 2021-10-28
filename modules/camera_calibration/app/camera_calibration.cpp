#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <experimental/filesystem>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opengv2/utility/utility.hpp>
#include <opengv2/sensor/PinholeCamera.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;
using namespace opengv2;
using namespace std::experimental;

static void help()
{
    cout << "This is a camera calibration sample." << endl
         << "Usage: camera_calibration [configuration_file -- default ./default.xml]" << endl
         << "Near the sample file you'll find the configuration file, which has detailed help of "
            "how to edit it.  It may be any OpenCV supported file format XML/YAML."
         << endl;
}

class Settings
{
public:
    Settings() : goodInput(false) {}

    enum Pattern
    {
        NOT_EXISTING,
        CHESSBOARD,
        CIRCLES_GRID,
        ASYMMETRIC_CIRCLES_GRID
    };
    enum InputType
    {
        INVALID,
        CAMERA,
        VIDEO_FILE,
        IMAGE_LIST
    };

    void write(FileStorage &fs) const //Write serialization for this class
    {
        fs << "{"
           << "BoardSize_Width" << boardSize.width
           << "BoardSize_Height" << boardSize.height
           << "Square_Size" << squareSize
           << "Calibrate_Pattern" << patternToUse
           << "Calibrate_NrOfFrameToUse" << nrFrames
           << "Calibrate_FixAspectRatio" << aspectRatio
           << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
           << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

           << "Write_DetectedFeaturePoints" << writePoints
           << "Write_extrinsicParameters" << writeExtrinsics
           << "Write_outputFileName" << outputFileName

           << "Show_UndistortedImage" << showUndistorsed

           << "Input_FlipAroundHorizontalAxis" << flipVertical
           << "Input_Delay" << delay
           << "Input" << input
           << "}";
    }

    void read(const FileNode &node) //Read serialization for this class
    {
        node["BoardSize_Width"] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"] >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> writePoints;
        node["Write_extrinsicParameters"] >> writeExtrinsics;
        //node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Calibrate_UseFisheyeModel"] >> useFisheye;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        node["Fix_K1"] >> fixK1;
        node["Fix_K2"] >> fixK2;
        node["Fix_K3"] >> fixK3;
        node["Fix_K4"] >> fixK4;
        node["Fix_K5"] >> fixK5;

        node["Extrinsic_Calibration"] >> calibExtrinsics;
        node["Move_Chessboard"] >> moveChessboard;
        node["camera_matrix"] >> preCameraMatrix;
        node["distortion_coefficients"] >> preDistCoeffs;

        validate();
    }

    void validate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        /*if (nrFrames <= 0) {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }*/

        if (input.empty()) // Check for valid input
            inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (isListOfImages(input) && readStringList(input, imageList))
                {
                    inputType = IMAGE_LIST;
                    nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                    //nrFrames = imageList.size();
                }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
                inputCapture.open(cameraID);
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Input does not exist: " << input;
            goodInput = false;
        }

        flag = 0;
        if (calibFixPrincipalPoint)
            flag |= CALIB_FIX_PRINCIPAL_POINT;
        if (calibZeroTangentDist)
            flag |= CALIB_ZERO_TANGENT_DIST;
        if (aspectRatio)
            flag |= CALIB_FIX_ASPECT_RATIO;
        if (fixK1)
            flag |= CALIB_FIX_K1;
        if (fixK2)
            flag |= CALIB_FIX_K2;
        if (fixK3)
            flag |= CALIB_FIX_K3;
        if (fixK4)
            flag |= CALIB_FIX_K4;
        if (fixK5)
            flag |= CALIB_FIX_K5;

        if (useFisheye)
        {
            // the fisheye model has its own enum, so overwrite the flags
            flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
            if (fixK1)
                flag |= fisheye::CALIB_FIX_K1;
            if (fixK2)
                flag |= fisheye::CALIB_FIX_K2;
            if (fixK3)
                flag |= fisheye::CALIB_FIX_K3;
            if (fixK4)
                flag |= fisheye::CALIB_FIX_K4;
            if (calibFixPrincipalPoint)
                flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
        }

        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD"))
            calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID"))
            calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID"))
            calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
        {
            cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
            goodInput = false;
        }
        atImageList = 0;
    }

    Mat nextImage(double &timestamp)
    {
        Mat result;
        if (inputCapture.isOpened())
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if (atImageList < imageList.size())
        {
            size_t lastIndex = imageList[atImageList].find_last_of(".");
            std::size_t beginIndex = imageList[atImageList].find_last_of("/");
            string rawName = imageList[atImageList].substr(beginIndex + 1, lastIndex - beginIndex - 1);
            long long tmp = std::stoll(rawName);
            //timestamp = ((tmp - startTime_) * 1e-6);
            timestamp = tmp;
            result = imread(imageList[atImageList++], IMREAD_COLOR);
        }

        return result;
    }

    static bool readStringList(const string &path, vector<string> &l)
    {
        l.clear();
        /*FileStorage fs(filename, FileStorage::READ);
        if (!fs.isOpened())
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if (n.type() != FileNode::SEQ)
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for (; it != it_end; ++it)
            l.push_back((string) *it);*/
        set<filesystem::path> sorted_by_name;
        for (const auto &entry : filesystem::directory_iterator(path))
        {
            sorted_by_name.insert(entry.path());
        }

        std::move(sorted_by_name.begin(), sorted_by_name.end(), std::back_inserter(l));

        return true;
    }

    static bool isListOfImages(const string &filename)
    {
        /*string s(filename);
        // Look for file extension
        if (s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos)
            return false;
        else
            return true;*/
        return true;
    }

public:
    Size boardSize;              // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;                // The number of frames to use from the input for calibration
    float aspectRatio;           // The aspect ratio
    int delay;                   // In case of a video input
    bool writePoints;            // Write detected feature points
    bool writeExtrinsics;        // Write extrinsic parameters
    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    bool flipVertical;           // Flip the captured images around the horizontal axis
    string outputFileName;       // The name of the file where to write
    bool showUndistorsed;        // Show undistorted images after calibration
    string input;                // The input ->
    bool useFisheye;             // use fisheye camera model for calibration
    bool fixK1;                  // fix K1 distortion coefficient
    bool fixK2;                  // fix K2 distortion coefficient
    bool fixK3;                  // fix K3 distortion coefficient
    bool fixK4;                  // fix K4 distortion coefficient
    bool fixK5;                  // fix K5 distortion coefficient

    bool calibExtrinsics;
    bool moveChessboard;
    Mat eventCameraMatrix, eventDistCoeffs;
    Mat preCameraMatrix, preDistCoeffs;

    int cameraID;
    vector<string> imageList;
    size_t atImageList;
    //long long startTime_;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;
};

static inline void read(const FileNode &node, Settings &x, const Settings &default_value = Settings())
{
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}

enum
{
    DETECTION = 0,
    CAPTURING = 1,
    CALIBRATED = 2
};

bool runCalibrationAndSave(Settings &s, Size imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                           vector<vector<Point2f>> imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                           const vector<double> &timestampSet);

bool runExtCalibAndSave(Settings &s, Size imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                        vector<vector<Point2f>> imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                        const vector<double> &timestampSet);

static void compareResult(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                          const vector<vector<Point2f>> &imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                          const Mat &eventCameraMatrix, const Mat &eventDistCoeffs);

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f> &corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/);

int main(int argc, char *argv[])
{
    help();

    if (argc != 3)
    {
        cerr << endl
             << "Usage: ./camera_calibration settingFilePath SavePath"
             << endl;
        return 1;
    }

    //! [file_read]
    Settings s;
    const string inputSettingsFile = argv[1];
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release(); // close Settings file
    //! [file_read]

    //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
    //fout << "Settings" << s;

    s.outputFileName = std::string(argv[2]);
    auto lastIndex = s.outputFileName.find_last_of('/');
    string imageFolder = s.outputFileName.substr(0, lastIndex) + "/cvImage/";
    experimental::filesystem::remove_all(imageFolder);
    experimental::filesystem::create_directories(imageFolder);

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f>> imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
    const char ESC_KEY = 27;

    vector<Mat> rvecs, tvecs;
    vector<double> timestampSet;

    int counter = 0;

    if (s.calibExtrinsics)
    {
        cout << "Extrinsic Calibration Mode." << endl;
        cameraMatrix = s.preCameraMatrix;
        distCoeffs = s.preDistCoeffs;
    }
    else
    {
        cout << "Intrinsic Calibration Mode." << endl;
    }

    //! [get_input]
    for (;;)
    {
        Mat view;
        bool blinkOutput = false;

        double timestamp;
        view = s.nextImage(timestamp);
        counter++;
        cout<<"Current Image: "<<timestamp<<endl;
        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if (!s.calibExtrinsics && mode == CAPTURING && (imagePoints.size() >= (size_t)s.nrFrames || counter >= s.imageList.size()))
        {
            if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, timestampSet))
            {
                mode = CALIBRATED;
                std::cout << "Camera has been calibrated." << std::endl;
            }
            else
            {
                mode = DETECTION;
                std::cout << "Camera has not been calibrated." << std::endl;
            }
        }

        //-----  If do extrinsic calibration, collect enough images -------------
        if (s.calibExtrinsics && mode == CAPTURING && (imagePoints.size() >= (size_t)s.nrFrames || counter >= s.imageList.size()))
        {
            std::cout << "Do Extrinsic Calibration." << std::endl;
            runExtCalibAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, timestampSet);
            mode = CALIBRATED;

            // vector<vector<Point3f>> objectPoints(1);
            // calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
            // objectPoints.resize(imagePoints.size(), objectPoints[0]);

            // for (size_t i = 0; i < imagePoints.size(); i++)
            // {
            //     Mat rvec_tmp, tvec_tmp;
            //     solvePnP(objectPoints[i], imagePoints[i], cameraMatrix, distCoeffs, rvec_tmp, tvec_tmp);
            //     cout << "rvec_tmp[" << i << "]: " << endl
            //          << rvec_tmp << endl;
            //     cout << "tvec_tmp[" << i << "]: " << endl
            //          << tvec_tmp << endl;
            //     cout << endl;
            //     tvecs.push_back(tvec_tmp);
            //     rvecs.push_back(rvec_tmp);
            // }
        }

        if (view.empty()) // If there are no more images stop the loop
        {
            // if calibration threshold was not reached yet, calibrate now
            if (!s.calibExtrinsics && mode != CALIBRATED && !imagePoints.empty())
                runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, timestampSet);

            if (s.calibExtrinsics && mode != CALIBRATED && !imagePoints.empty())
                std::cout << "Do Extrinsic Calibration." << std::endl;
            break;
        }
        //! [get_input]

        imageSize = view.size(); // Format input image.
        if (s.flipVertical)
            flip(view, view, 0);

        //! [find_pattern]
        vector<Point2f> pointBuf;

        bool found;

        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

        if (!s.useFisheye)
        {
            // fast check erroneously fails with high distortions like fisheye
            chessBoardFlags |= CALIB_CB_FAST_CHECK;
        }

        switch (s.calibrationPattern) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners(view, s.boardSize, pointBuf, chessBoardFlags);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid(view, s.boardSize, pointBuf);
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid(view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
            break;
        default:
            found = false;
            break;
        }
        //! [find_pattern]
        //! [pattern_found]
        if (found) // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if (s.calibrationPattern == Settings::CHESSBOARD)
            {
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                cornerSubPix(viewGray, pointBuf, Size(11, 11),
                             Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            }

            // Draw the corners.
            drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);

            // output image
            cv::imwrite(imageFolder + std::to_string(timestamp) + "_cv.png", view);
        }
        //! [pattern_found]

        //----------------------------- Output Text ------------------------------------------------
        //! [output_text]
        string msg = (mode == CAPTURING) ? "100/100" : mode == CALIBRATED ? "Calibrated"
                                                                          : "Press 'g' to start";

        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

        if (mode == CAPTURING)
        {
            if (s.showUndistorsed)
                msg = format("%d/%d Undist", (int)imagePoints.size(), s.nrFrames);
            else
                msg = format("%d/%d", (int)imagePoints.size(), s.nrFrames);
        }

        putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

        if (blinkOutput)
            bitwise_not(view, view);
        //! [output_text]
        //------------------------- Video capture  output  undistorted ------------------------------
        //! [output_undistorted]
        if (mode == CALIBRATED && s.showUndistorsed)
        {
            Mat temp = view.clone();
            if (s.useFisheye)
                cv::fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs);
            else
                undistort(temp, view, cameraMatrix, distCoeffs);
        }
        //! [output_undistorted]
        //------------------------------ Show image and check for input commands -------------------
        //! [await_input]
        imshow("Image View", view);
        char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if (key == ESC_KEY)
            break;

        if (key == 'u' && mode == CALIBRATED)
            s.showUndistorsed = !s.showUndistorsed;

        if (s.inputCapture.isOpened() && key == 'g')
        {
            mode = CAPTURING;
            imagePoints.clear();
        }

        if (key == 's')
        {
            if (found) // If done with success,
            {
                if (mode == CAPTURING && // For camera only take new samples after delay time
                    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay * 1e-3 * CLOCKS_PER_SEC))
                {
                    imagePoints.push_back(pointBuf);
                    timestampSet.push_back(timestamp);
                    prevTimestamp = clock();
                    blinkOutput = s.inputCapture.isOpened();
                }
            }
            //! [pattern_found]
        }

        //! [await_input]
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    //! [show_results]
    if (s.inputType == Settings::IMAGE_LIST && s.showUndistorsed)
    {
        /*compareResult(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, s.eventCameraMatrix,
                      s.eventDistCoeffs);*/
    }
    //! [show_results]

    return 0;
}

//! [compute_errors]
static double computeReprojectionErrors(const vector<vector<Point3f>> &objectPoints,
                                        const vector<vector<Point2f>> &imagePoints,
                                        const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                                        const Mat &cameraMatrix, const Mat &distCoeffs,
                                        vector<float> &perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

static void computeBackProjectionErrors(const vector<vector<Point3f>> &objectPoints,
                                        const vector<vector<Point2f>> &imagePoints,
                                        const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                                        const Mat &cameraMatrix, const Mat &distCoeffs,
                                        vector<vector<float>> &errors, bool fisheye, bool inversePoly)
{
    errors.resize(objectPoints.size());

    Eigen::Matrix3f invK;
    cv2eigen(cameraMatrix.inv(), invK);
    Eigen::VectorXf inverseRadialPoly;
    cv2eigen(distCoeffs, inverseRadialPoly);
    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        cv::Mat cvRsw;
        cv::Rodrigues(rvecs[i], cvRsw);
        Eigen::Matrix3f Rsw;
        Eigen::Vector3f tsw;
        cv::cv2eigen(cvRsw, Rsw);
        cv::cv2eigen(tvecs[i], tsw);
        Eigen::Matrix3f Rws = Rsw.transpose();
        Eigen::Vector3f tws = -Rws * tsw;

        vector<Point3f> objectPoints2;
        objectPoints2.reserve(objectPoints[i].size());
        if (inversePoly)
        {
            for (const auto &p : imagePoints[i])
            {
                Eigen::Vector3f Xc;
                Xc = invK * Eigen::Vector3f(p.x, p.y, 1);
                Xc /= Xc[2];

                Eigen::VectorXf r_coeff(inverseRadialPoly.size());
                r_coeff[0] = Xc[0] * Xc[0] + Xc[1] * Xc[1];
                for (int k = 1; k < inverseRadialPoly.size(); ++k)
                {
                    r_coeff[k] = r_coeff[k - 1] * r_coeff[0];
                }
                Xc[0] *= 1 + r_coeff.transpose() * inverseRadialPoly;
                Xc[1] *= 1 + r_coeff.transpose() * inverseRadialPoly;

                double depth = -tws[2] / (Rws.row(2).dot(Xc));
                Xc *= depth;

                Eigen::Vector3f Xw = Rws * Xc + tws;
                objectPoints2.emplace_back(Xw[0], Xw[1], Xw[2]);
            }
        }
        else
        {
            std::vector<cv::Point2f> dst;
            if (fisheye)
            {
                fisheye::undistortPoints(imagePoints[i], dst, cameraMatrix, distCoeffs);
            }
            else
            {
                undistortPoints(imagePoints[i], dst, cameraMatrix, distCoeffs);
            }

            for (const auto &p : dst)
            {
                Eigen::Vector3f Xc(p.x, p.y, 1);

                double depth = -tws[2] / (Rws.row(2).dot(Xc));
                Xc *= depth;

                Eigen::Vector3f Xw = Rws * Xc + tws;
                objectPoints2.emplace_back(Xw[0], Xw[1], Xw[2]);
            }
        }

        errors[i].resize(objectPoints2.size());
        for (int k = 0; k < objectPoints2.size(); ++k)
        {
            errors[i][k] = norm(objectPoints[i][k] - objectPoints2[k]);
        }
    }
}

//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f> &corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch (patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; ++i)
            for (int j = 0; j < boardSize.width; ++j)
                corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
                corners.push_back(Point3f((2 * j + i % 2) * squareSize, i * squareSize, 0));
        break;
    default:
        break;
    }
}

double evaluateStraightness(const vectorofEigenMatrix<Eigen::Vector3d> &centers, int height, int width,
                            bool isAsymmetric)
{
    std::vector<std::vector<int>> lineSet;

    // each row
    for (int i = 0; i < height; ++i)
    {
        lineSet.emplace_back();
        for (int j = 0; j < width; ++j)
        {
            lineSet.back().push_back(i * width + j);
        }
    }

    if (isAsymmetric)
    {
        // each col
        for (int j = 0; j < width; ++j)
        {
            lineSet.emplace_back();
            for (int i = 0; i < height; i += 2)
            {
                lineSet.back().push_back(i * width + j);
            }
            lineSet.emplace_back();
            for (int i = 1; i < height; i += 2)
            {
                lineSet.back().push_back(i * width + j);
            }
        }
    }
    else
    {
        // each col
        for (int j = 0; j < width; ++j)
        {
            lineSet.emplace_back();
            for (int i = 0; i < height; i++)
            {
                lineSet.back().push_back(i * width + j);
            }
        }
    }

    double err = 0;
    for (const auto &pointIdSet : lineSet)
    {
        Eigen::MatrixXd A(pointIdSet.size(), 3);
        for (int i = 0; i < pointIdSet.size(); ++i)
        {
            A.row(i) = centers[pointIdSet[i]].transpose();
        }

        Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
        Eigen::Vector3d line = svd.matrixV().col(2);

        double sum = 0;
        for (const auto &pointId : pointIdSet)
        {
            sum += std::abs(line.transpose() * centers[pointId]);
        }
        sum /= pointIdSet.size();

        err += sum;
    }
    err /= lineSet.size();

    return err;
}

static void compareResult(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                          const vector<vector<Point2f>> &imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                          const Mat &eventCameraMatrix, const Mat &eventDistCoeffs)
{
    vector<vector<Point3f>> objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    std::vector<float> cvErr;
    std::vector<float> splineErr;
    vector<vector<float>> errors;
    computeBackProjectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                distCoeffs, errors, s.useFisheye, false);
    for (auto &&v : errors)
    {
        std::move(v.begin(), v.end(), std::back_inserter(cvErr));
    }
    computeBackProjectionErrors(objectPoints, imagePoints, rvecs, tvecs, eventCameraMatrix,
                                eventDistCoeffs, errors, s.useFisheye, true);
    for (auto &&v : errors)
    {
        std::move(v.begin(), v.end(), std::back_inserter(splineErr));
    }

    float cvMean, cvSdtdev, splineMean, splineSdtdev;
    fitNormal(cvErr, cvMean, cvSdtdev);
    fitNormal(splineErr, splineMean, splineSdtdev);

    std::cout << "OpenCV back-projection error (mean, stddev): " << cvMean << " " << cvSdtdev << std::endl;
    std::cout << "Ours back-projection error (mean, stddev): " << splineMean << " " << splineSdtdev << std::endl;

    /*** evaluateStraightness ***/
    // ours
    Eigen::VectorXd inverseRadialPoly;
    Eigen::Matrix3d K;
    Eigen::Vector2d cs(imageSize.width, imageSize.height);
    cv2eigen(eventDistCoeffs, inverseRadialPoly);
    cv2eigen(eventCameraMatrix, K);
    auto camera = make_shared<PinholeCamera>(cs, K);
    camera->inverseRadialPoly() = inverseRadialPoly;

    // cv
    Eigen::VectorXd distortion;
    cv2eigen(distCoeffs, distortion);
    cv2eigen(cameraMatrix, K);
    auto cvCamera = make_shared<PinholeCamera>(cs, K, distortion);

    // evaluateStraightness
    std::vector<float> ourErrSet, cvErrSet;
    for (const auto &imagePointSet : imagePoints)
    {
        opengv2::vectorofEigenMatrix<Eigen::Vector3d> normalizedCenters, cv_normalizedCenters;
        for (const auto &imagePoint : imagePointSet)
        {
            Eigen::Vector2d p(imagePoint.x, imagePoint.y);
            Eigen::Vector3d pc;
            pc.block<2, 1>(0, 0) = camera->undistortPoint(p);
            pc[2] = 1;
            normalizedCenters.push_back(pc);

            Eigen::Vector3d cv_pc;
            cv_pc.block<2, 1>(0, 0) = cvCamera->undistortPoint(p);
            cv_pc[2] = 1;
            cv_normalizedCenters.push_back(cv_pc);
        }
        ourErrSet.push_back(evaluateStraightness(normalizedCenters, s.boardSize.height, s.boardSize.width,
                                                 s.calibrationPattern == Settings::ASYMMETRIC_CIRCLES_GRID));
        cvErrSet.push_back(evaluateStraightness(cv_normalizedCenters, s.boardSize.height, s.boardSize.width,
                                                s.calibrationPattern == Settings::ASYMMETRIC_CIRCLES_GRID));
    }
    fitNormal(cvErrSet, cvMean, cvSdtdev);
    fitNormal(ourErrSet, splineMean, splineSdtdev);

    std::cout << "OpenCV undistortion straightness error (mean, stddev): " << cvMean << " " << cvSdtdev << std::endl;
    std::cout << "Ours undistortion straightness error (mean, stddev): " << splineMean << " " << splineSdtdev
              << std::endl;

    /*** show undistorted images ***/
    Mat view, rview, map1, map2;
    if (s.useFisheye)
    {
        Mat newCamMat;
        fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                                                            Matx33d::eye(), newCamMat, 1);
        fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize,
                                         CV_32FC1, map1, map2);
    }
    else
    {
        initUndistortRectifyMap(
            cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,
            CV_32FC1, map1, map2);
    }

    cv::namedWindow("Debug_undistorted");
    cv::namedWindow("Debug_undistorted_cv");
    for (size_t i = 0; i < s.imageList.size(); i++)
    {
        view = imread(s.imageList[i], IMREAD_COLOR);
        if (view.empty())
            continue;
        remap(view, rview, map1, map2, INTER_LINEAR);
        cv::Mat ourView = camera->undistortImage(view);
        imshow("Debug_undistorted_cv", rview);
        imshow("Debug_undistorted", ourView);
        waitKey(10);
    }
}

//! [board_corners]
static bool runCalibration(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                           vector<vector<Point2f>> imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                           vector<float> &reprojErrs, double &totalAvgErr)
{
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (s.flag & CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0, 0) = s.aspectRatio;
    //! [fixed_aspect]
    if (s.useFisheye)
    {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    }
    else
    {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f>> objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye)
    {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                                 _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for (int i = 0; i < int(objectPoints.size()); i++)
        {
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    }
    else
    {
        rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                              s.flag | CALIB_USE_LU);
    }

    cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

//! [board_corners]
static bool runExtrinsicCalibration(Settings &s, Mat &cameraMatrix, Mat &distCoeffs,
                                    vector<vector<Point2f>> imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                                    vector<float> &reprojErrs, double &totalAvgErr)
{
    // if (s.useFisheye)
    // {
    //     distCoeffs = Mat::zeros(4, 1, CV_64F);
    // }
    // else
    // {
    //     distCoeffs = Mat::zeros(8, 1, CV_64F);
    // }

    vector<vector<Point3f>> objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye)
    {
        cout << "use Fisheye model.";
    }
    else
    {
        cout << "use Pinhole model.";
        for (size_t i = 0; i < imagePoints.size(); i++)
        {
            Mat rvec_tmp, tvec_tmp;
            solvePnP(objectPoints[i], imagePoints[i], cameraMatrix, distCoeffs, rvec_tmp, tvec_tmp);
            cout << "rvec_tmp[" << i << "]: " << endl
                 << rvec_tmp << endl;
            cout << "tvec_tmp[" << i << "]: " << endl
                 << tvec_tmp << endl;
            cout << endl;
            tvecs.push_back(tvec_tmp);
            rvecs.push_back(rvec_tmp);
        }
    }

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                             const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                             const vector<float> &reprojErrs, const vector<vector<Point2f>> &imagePoints,
                             double totalAvgErr, const vector<double> &timestampSet)
{
    FileStorage fs(s.outputFileName, FileStorage::WRITE);

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if (s.flag & CALIB_FIX_ASPECT_RATIO)
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag)
    {
        std::stringstream flagsStringStream;
        if (s.useFisheye)
        {
            flagsStringStream << "flags:"
                              << (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                              << (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                              << (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                              << (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                              << (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                              << (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            flagsStringStream << "flags:"
                              << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                              << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                              << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                              << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                              << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                              << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                              << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                              << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                              << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if (s.writeExtrinsics && !rvecs.empty() && !tvecs.empty())
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

        std::ofstream f;
        auto lastIndex = s.outputFileName.find_last_of('/');
        f.open(s.outputFileName.substr(0, lastIndex) + "/TrajectoryByCV.txt");
        f << std::fixed;
        for (size_t i = 0; i < rvecs.size(); i++)
        {
            Mat r = bigmat(Range(int(i), int(i + 1)), Range(0, 3));
            Mat t = bigmat(Range(int(i), int(i + 1)), Range(3, 6));

            if (needReshapeR)
                rvecs[i].reshape(1, 1).copyTo(r);
            else
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                r = rvecs[i].t();
            }

            if (needReshapeT)
                tvecs[i].reshape(1, 1).copyTo(t);
            else
            {
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                t = tvecs[i].t();
            }

            cv::Mat cvRcw;
            cv::Rodrigues(rvecs[i], cvRcw);
            Eigen::Matrix3d Rcw;
            Eigen::Vector3d tcw;
            cv::cv2eigen(cvRcw, Rcw);
            cv::cv2eigen(tvecs[i], tcw);
            Eigen::Quaterniond Qwc(Rcw.transpose());
            Qwc.normalize();
            Eigen::Quaterniond Qcw(Rcw);
            Qcw.normalize();
            Eigen::Vector3d twc = -(Qwc * tcw);

            //timestamp tx ty tz qx qy qz qw
            if (s.moveChessboard){
                cout<<"Save Chessboard Trajectory!"<<endl;
                f << std::setprecision(10) << timestampSet[i] << " " << tcw[0] << " " << tcw[1] << " "
                  << tcw[2] << " " << Qcw.x() << " " << Qcw.y() << " " << Qcw.z() << " " << Qcw.w() << std::endl;
            }
                
            else
            {
                cout<<"Save Camera Trajectory!"<<endl;
                f << std::setprecision(10) << timestampSet[i] << " " << twc[0] << " " << twc[1] << " "
                  << twc[2] << " " << Qwc.x() << " " << Qwc.y() << " " << Qwc.z() << " " << Qwc.w() << std::endl;
            }
                
        }
        //fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
        //fs << "extrinsic_parameters" << bigmat;
        f.close();
    }

    if (s.writePoints && !imagePoints.empty())
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for (size_t i = 0; i < imagePoints.size(); i++)
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

//! [run_and_save]
bool runCalibrationAndSave(Settings &s, Size imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                           vector<vector<Point2f>> imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                           const vector<double> &timestampSet)
{
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                             totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr, timestampSet);
    return ok;
}
//! [run_and_save]

//! [run_and_save]
bool runExtCalibAndSave(Settings &s, Size imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                        vector<vector<Point2f>> imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                        const vector<double> &timestampSet)
{
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runExtrinsicCalibration(s, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                                      totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr, timestampSet);
    return ok;
}