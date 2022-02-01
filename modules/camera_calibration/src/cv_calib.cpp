//
// Created by yifu wang on 2021/8/27.
//

#include <cv_calib.hpp>
#include <circlesgrid.hpp>

bool cv::findCirclesGrid(const std::vector<Point2f> &points_, Size patternSize, OutputArray _centers, int flags,
                         const CirclesGridFinderParameters &parameters_)
{
    CirclesGridFinderParameters parameters = parameters_; // parameters.gridType is amended below

    bool isAsymmetricGrid = (flags & CALIB_CB_ASYMMETRIC_GRID) ? true : false;
    bool isSymmetricGrid = (flags & CALIB_CB_SYMMETRIC_GRID) ? true : false;
    CV_Assert(isAsymmetricGrid ^ isSymmetricGrid);

    std::vector<Point2f> centers;
    std::vector<Point2f> points(points_);

    if (flags & CALIB_CB_ASYMMETRIC_GRID)
        parameters.gridType = CirclesGridFinderParameters::ASYMMETRIC_GRID;
    if (flags & CALIB_CB_SYMMETRIC_GRID)
        parameters.gridType = CirclesGridFinderParameters::SYMMETRIC_GRID;

    if (flags & CALIB_CB_CLUSTERING)
    {
        // uses a special algorithm for grid detection. It is more robust to perspective distortions
        // but much more sensitive to background clutter.
        CirclesGridClusterFinder circlesGridClusterFinder(parameters);
        circlesGridClusterFinder.findGrid(points, patternSize, centers);
        Mat(centers).copyTo(_centers);
        return !centers.empty();
    }

    const int attempts = 2;
    const size_t minHomographyPoints = 4;
    Mat H;
    for (int i = 0; i < attempts; i++)
    {
        centers.clear();
        CirclesGridFinder boxFinder(patternSize, points, parameters);
        bool isFound = false;
        try
        {
            isFound = boxFinder.findHoles();
        }
        catch (const cv::Exception &)
        {
        }

        if (isFound)
        {
            switch (parameters.gridType)
            {
            case CirclesGridFinderParameters::SYMMETRIC_GRID:
                boxFinder.getHoles(centers);
                break;
            case CirclesGridFinderParameters::ASYMMETRIC_GRID:
                boxFinder.getAsymmetricHoles(centers);
                break;
            default:
                CV_Error(Error::StsBadArg, "Unknown pattern type");
            }

            // add by huangkun
            if (centers.empty())
            {
                /*std::cout << "debug" << std::endl;
                FileStorage fout("/home/huangkun/debug.yml", FileStorage::WRITE);
                fout << "candidateCenters" << points_;
                fout << "patternSize" << patternSize;
                fout << "flags" << flags;
                fout.release();*/
                return false;
            }

            if (i != 0)
            {
                Mat orgPointsMat;
                transform(centers, orgPointsMat, H.inv());
                convertPointsFromHomogeneous(orgPointsMat, centers);
            }
            Mat(centers).copyTo(_centers);
            return true;
        }

        boxFinder.getHoles(centers);
        if (i != attempts - 1)
        {
            if (centers.size() < minHomographyPoints)
                break;
            H = CirclesGridFinder::rectifyGrid(boxFinder.getDetectedGridSize(), centers, points, points);
        }
    }
    Mat(centers).copyTo(_centers);
    return false;
}

static void writeMatToFile(const Eigen::MatrixXd &m, std::string DIR)
{
    std::ofstream fout(DIR, std::ofstream::trunc);
    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    for (int i = 0; i < m.rows(); i++)
    {
        for (int j = 0; j < m.cols(); j++)
        {
            fout << m(i, j) << "\t";
        }
        fout << std::endl;
    }

    fout.close();
}

static void writeCvMatToFile(const cv::Mat &matData, std::string DIR)
{
    std::ofstream fout(DIR, std::ofstream::trunc);
    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    for (int r = 0; r < matData.rows; r++)
    {
        for (int c = 0; c < matData.cols; c++)
        {
            int data = matData.at<uchar>(r, c);
            fout << data << "\t";
        }
        fout << std::endl;
    }

    fout.close();
}

// Kronecker product or tensor product
// https://stackoverflow.com/a/36552682
static cv::Mat kron(const cv::Mat &A, const cv::Mat &B)
{
    CV_Assert(A.channels() == 1 && B.channels() == 1);

    cv::Mat1d Ad, Bd;
    A.convertTo(Ad, CV_64F);
    B.convertTo(Bd, CV_64F);

    cv::Mat1d Kd(Ad.rows * Bd.rows, Ad.cols * Bd.cols, 0.0);
    for (int ra = 0; ra < Ad.rows; ra++)
    {
        for (int ca = 0; ca < Ad.cols; ca++)
        {
            Kd(cv::Range(ra * Bd.rows, (ra + 1) * Bd.rows), cv::Range(ca * Bd.cols, (ca + 1) * Bd.cols)) = Bd.mul(Ad(ra, ca));
        }
    }

    cv::Mat K;
    Kd.convertTo(K, A.type());
    return K;
}

static cv::Mat_<double> normalizeRotation(const cv::Mat_<double> &R_)
{
    // Make R unit determinant
    cv::Mat_<double> R = R_.clone();
    double det = determinant(R);
    if (std::fabs(det) < FLT_EPSILON)
    {
        CV_Error(cv::Error::StsNoConv, "Rotation normalization issue: determinant(R) is null");
    }
    R = std::cbrt(std::copysign(1, det) / std::fabs(det)) * R;

    // Make R orthogonal
    cv::Mat w, u, vt;
    cv::SVDecomp(R, w, u, vt);
    R = u * vt;

    // Handle reflection case
    if (determinant(R) < 0)
    {
        cv::Matx33d diag(1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, -1.0);
        R = u * diag * vt;
    }
    return R;
}

//Reference:
//M. Shah, "Solving the robot-world/hand-eye calibration problem using the kronecker product"
//Journal of Mechanisms and Robotics, vol. 5, p. 031007, 2013.
//Matlab code: http://math.loyola.edu/~mili/Calibration/
static void calibrateRobotWorldHandEyeYifu(const std::vector<std::vector<cv::Mat_<double>>> &multi_cRw, const std::vector<std::vector<cv::Mat_<double>>> &multi_ctw,
                                           const std::vector<std::vector<cv::Mat_<double>>> &multi_gRb, const std::vector<std::vector<cv::Mat_<double>>> &multi_gtb,
                                           std::vector<cv::Matx33d> &multi_wRb, std::vector<cv::Matx31d> &multi_wtb, cv::Matx33d &cRg, cv::Matx31d &ctg)
{
    //Get the number of views
    const int nv = static_cast<int>(multi_cRw.size());

    //Define the total number of pairs
    int pairs = 0;

    //Define the Size of Mat
    Eigen::MatrixXd Large = Eigen::MatrixXd::Zero(nv * 18, (1 + nv) * 9);
    Eigen::MatrixXd I9 = Eigen::MatrixXd::Identity(9, 9);

    //Construct the Ax=0 for rotations
    for (size_t i = 0; i < nv; i++)
    {
        //Define the R and t which would be used
        auto &cRw = multi_cRw[i];
        auto &ctw = multi_ctw[i];
        auto &gRb = multi_gRb[i];
        auto &gtb = multi_gtb[i];
        auto &wRb = multi_wRb[i];
        auto &wtb = multi_wtb[i];

        //Get the number of pairs
        const int np = static_cast<int>(cRw.size());
        pairs = pairs + np;

        //Sum of Kronecker product of RBj and RAj,
        cv::Mat_<double> cv_sumMat = cv::Mat_<double>::zeros(9, 9);
        for (size_t j = 0; j < cRw.size(); j++)
        {
            cv_sumMat += kron(gRb[j], cRw[j]);
        }
        Eigen::MatrixXd sumMat;
        cv::cv2eigen(cv_sumMat, sumMat);

        Large.block<9, 9>(i * 9, 0) = np * I9;
        Large.block<9, 9>((nv + i) * 9, (1 + i) * 9) = np * I9;
        Large.block<9, 9>(i * 9, (1 + i) * 9) = -sumMat;
        Large.block<9, 9>((nv + i) * 9, 0) = -sumMat.transpose();
    }

    cv::Mat_<double> cv_Large;
    cv::eigen2cv(Large, cv_Large);

    cv::Mat vecZX;
    cv::SVD::solveZ(cv_Large, vecZX);

    //Retrive RZ from vector
    cv::Mat_<double> RZ(3, 3);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            RZ(j, i) = vecZX.at<double>(0, i * 3 + j);
        }
    }
    cRg = normalizeRotation(RZ);

    //Retrive multiple RX from vector
    for (size_t i = 0; i < nv; i++)
    {
        cv::Mat_<double> RX(3, 3);
        for (int k = 0; k < 3; k++)
        {
            for (int j = 0; j < 3; j++)
            {
                RX(j, k) = vecZX.at<double>(0, (i + 1) * 9 + k * 3 + j);
            }
        }
        multi_wRb[i] = normalizeRotation(RX);
    }

    //Construct the Ax=0 for translations
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * pairs, 3 * (nv + 1));
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(3 * pairs, 1);
    Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
    int nrows = 0;

    for (size_t i = 0; i < nv; i++)
    {
        //Define the R and t which would be used
        auto &cRw = multi_cRw[i];
        auto &ctw = multi_ctw[i];
        auto &gRb = multi_gRb[i];
        auto &gtb = multi_gtb[i];
        auto &wRb = multi_wRb[i];
        auto &wtb = multi_wtb[i];

        //Get the number of pairs
        const int np = static_cast<int>(cRw.size());

        for (size_t j = 0; j < np; j++)
        {
            Eigen::MatrixXd eig_cRw;
            cv::cv2eigen(-cRw[j], eig_cRw);
            A.block<3, 3>(nrows * 3, 0) = I3;
            A.block<3, 3>(nrows * 3, (1 + i) * 3) = eig_cRw;

            Eigen::MatrixXd eig_partb;
            cv::cv2eigen(ctw[j] - cRg * gtb[j], eig_partb);
            b.block<3, 1>(nrows * 3, 0) = eig_partb;

            nrows = nrows + 1;
        }
    }

    cv::Mat_<double> cv_A, cv_b;
    cv::eigen2cv(A, cv_A);
    cv::eigen2cv(b, cv_b);

    cv::Mat_<double> t;
    cv::solve(cv_A, cv_b, t, cv::DECOMP_SVD);

    //Retrive tZ from vector
    for (int i = 0; i < 3; i++)
    {
        ctg(i) = t(i);
    }

    //Retrive multiple tX from vector
    for (size_t i = 0; i < nv; i++)
    {
        auto &wtb = multi_wtb[i];
        for (int j = 0; j < 3; j++)
        {
            wtb(j) = t(i * 3 + j + 3);
        }
    }
}

void cv::calibrateRobotWorldHandEyeMulti(std::vector<std::vector<cv::Mat>> &multi_R_world2cam, std::vector<std::vector<cv::Mat>> &multi_t_world2cam,
                                         std::vector<std::vector<cv::Mat>> &multi_R_base2gripper, std::vector<std::vector<cv::Mat>> &multi_t_base2gripper,
                                         std::vector<cv::Mat> &multi_R_base2world, std::vector<cv::Mat> &multi_t_base2world,
                                         cv::Mat &R_gripper2cam, cv::Mat &t_gripper2cam)
{
    std::cout << "ours method" << std::endl;

    std::vector<std::vector<cv::Mat_<double>>> multi_R_base2gripper_, multi_t_base2gripper_;
    std::vector<std::vector<cv::Mat_<double>>> multi_R_world2cam_, multi_t_world2cam_;

    for (size_t i = 0; i < multi_R_world2cam.size(); i++)
    {
        cv::InputArrayOfArrays &R_world2cam = multi_R_world2cam[i];
        cv::InputArrayOfArrays &t_world2cam = multi_t_world2cam[i];
        cv::InputArrayOfArrays &R_base2gripper = multi_R_base2gripper[i];
        cv::InputArrayOfArrays &t_base2gripper = multi_t_base2gripper[i];

        CV_Assert(R_base2gripper.isMatVector() && t_base2gripper.isMatVector() &&
                  R_world2cam.isMatVector() && t_world2cam.isMatVector());

        std::vector<cv::Mat> R_base2gripper_tmp, t_base2gripper_tmp;
        R_base2gripper.getMatVector(R_base2gripper_tmp);
        t_base2gripper.getMatVector(t_base2gripper_tmp);

        std::vector<cv::Mat> R_world2cam_tmp, t_world2cam_tmp;
        R_world2cam.getMatVector(R_world2cam_tmp);
        t_world2cam.getMatVector(t_world2cam_tmp);

        CV_Assert(R_base2gripper_tmp.size() == t_base2gripper_tmp.size() &&
                  R_world2cam_tmp.size() == t_world2cam_tmp.size() &&
                  R_base2gripper_tmp.size() == R_world2cam_tmp.size());
        CV_Check(R_base2gripper_tmp.size(), R_base2gripper_tmp.size() >= 3, "At least 3 measurements are needed");

        // Convert to double
        std::vector<cv::Mat_<double>> R_base2gripper_, t_base2gripper_;
        std::vector<cv::Mat_<double>> R_world2cam_, t_world2cam_;

        R_base2gripper_.reserve(R_base2gripper_tmp.size());
        t_base2gripper_.reserve(R_base2gripper_tmp.size());
        R_world2cam_.reserve(R_world2cam_tmp.size());
        t_world2cam_.reserve(R_base2gripper_tmp.size());

        // Convert to rotation matrix if needed
        for (size_t i = 0; i < R_base2gripper_tmp.size(); i++)
        {
            {
                cv::Mat rot = R_base2gripper_tmp[i];
                cv::Mat R(3, 3, CV_64FC1);
                if (rot.size() == cv::Size(3, 3))
                {
                    rot.convertTo(R, CV_64F);
                    R_base2gripper_.push_back(R);
                }
                else
                {
                    Rodrigues(rot, R);
                    R_base2gripper_.push_back(R);
                }
                cv::Mat tvec = t_base2gripper_tmp[i];
                cv::Mat t;
                tvec.convertTo(t, CV_64F);
                t_base2gripper_.push_back(t);
            }
            {
                cv::Mat rot = R_world2cam_tmp[i];
                cv::Mat R(3, 3, CV_64FC1);
                if (rot.size() == cv::Size(3, 3))
                {
                    rot.convertTo(R, CV_64F);
                    R_world2cam_.push_back(R);
                }
                else
                {
                    Rodrigues(rot, R);
                    R_world2cam_.push_back(R);
                }
                cv::Mat tvec = t_world2cam_tmp[i];
                cv::Mat t;
                tvec.convertTo(t, CV_64F);
                t_world2cam_.push_back(t);
            }
        }

        CV_Assert(R_world2cam_.size() == t_world2cam_.size() &&
                  R_base2gripper_.size() == t_base2gripper_.size() &&
                  R_world2cam_.size() == R_base2gripper_.size());

        //push sets back into vector
        multi_R_world2cam_.push_back(R_world2cam_);
        multi_t_world2cam_.push_back(t_world2cam_);
        multi_R_base2gripper_.push_back(R_base2gripper_);
        multi_t_base2gripper_.push_back(t_base2gripper_);
    }

    std::vector<cv::Matx33d> multi_wRb;
    std::vector<cv::Matx31d> multi_wtb;
    multi_wRb.resize(multi_R_world2cam.size());
    multi_wtb.resize(multi_R_world2cam.size());
    cv::Matx33d cRg;
    cv::Matx31d ctg;

    calibrateRobotWorldHandEyeYifu(multi_R_world2cam_, multi_t_world2cam_, multi_R_base2gripper_, multi_t_base2gripper_,
                                   multi_wRb, multi_wtb, cRg, ctg);

    for (size_t i = 0; i < multi_R_world2cam_.size(); i++)
    {
        cv::Mat(multi_wRb[i]).copyTo(multi_R_base2world[i]);
        cv::Mat(multi_wtb[i]).copyTo(multi_t_base2world[i]);
    }
    cv::Mat(cRg).copyTo(R_gripper2cam);
    cv::Mat(ctg).copyTo(t_gripper2cam);
}

void cv::RecoverBase2World(std::vector<std::vector<cv::Mat>> &multi_R_world2cam, std::vector<std::vector<cv::Mat>> &multi_t_world2cam,
                           std::vector<std::vector<cv::Mat>> &multi_R_base2gripper, std::vector<std::vector<cv::Mat>> &multi_t_base2gripper,
                           std::vector<cv::Mat> &multi_R_base2world, std::vector<cv::Mat> &multi_t_base2world,
                           cv::Mat &R_gripper2cam, cv::Mat &t_gripper2cam)
{
    std::cout << "Recover Base2World from known Tsb" << std::endl;

    std::vector<std::vector<cv::Mat_<double>>> multi_R_base2gripper_, multi_t_base2gripper_;
    std::vector<std::vector<cv::Mat_<double>>> multi_R_world2cam_, multi_t_world2cam_;

    for (size_t i = 0; i < multi_R_world2cam.size(); i++)
    {
        cv::InputArrayOfArrays &R_world2cam = multi_R_world2cam[i];
        cv::InputArrayOfArrays &t_world2cam = multi_t_world2cam[i];
        cv::InputArrayOfArrays &R_base2gripper = multi_R_base2gripper[i];
        cv::InputArrayOfArrays &t_base2gripper = multi_t_base2gripper[i];

        CV_Assert(R_base2gripper.isMatVector() && t_base2gripper.isMatVector() &&
                  R_world2cam.isMatVector() && t_world2cam.isMatVector());

        std::vector<cv::Mat> R_base2gripper_tmp, t_base2gripper_tmp;
        R_base2gripper.getMatVector(R_base2gripper_tmp);
        t_base2gripper.getMatVector(t_base2gripper_tmp);

        std::vector<cv::Mat> R_world2cam_tmp, t_world2cam_tmp;
        R_world2cam.getMatVector(R_world2cam_tmp);
        t_world2cam.getMatVector(t_world2cam_tmp);

        CV_Assert(R_base2gripper_tmp.size() == t_base2gripper_tmp.size() &&
                  R_world2cam_tmp.size() == t_world2cam_tmp.size() &&
                  R_base2gripper_tmp.size() == R_world2cam_tmp.size());
        CV_Check(R_base2gripper_tmp.size(), R_base2gripper_tmp.size() >= 3, "At least 3 measurements are needed");

        // Convert to double
        std::vector<cv::Mat_<double>> R_base2gripper_, t_base2gripper_;
        std::vector<cv::Mat_<double>> R_world2cam_, t_world2cam_;

        R_base2gripper_.reserve(R_base2gripper_tmp.size());
        t_base2gripper_.reserve(R_base2gripper_tmp.size());
        R_world2cam_.reserve(R_world2cam_tmp.size());
        t_world2cam_.reserve(R_base2gripper_tmp.size());

        // Convert to rotation matrix if needed
        for (size_t i = 0; i < R_base2gripper_tmp.size(); i++)
        {
            {
                cv::Mat rot = R_base2gripper_tmp[i];
                cv::Mat R(3, 3, CV_64FC1);
                if (rot.size() == cv::Size(3, 3))
                {
                    rot.convertTo(R, CV_64F);
                    R_base2gripper_.push_back(R);
                }
                else
                {
                    Rodrigues(rot, R);
                    R_base2gripper_.push_back(R);
                }
                cv::Mat tvec = t_base2gripper_tmp[i];
                cv::Mat t;
                tvec.convertTo(t, CV_64F);
                t_base2gripper_.push_back(t);
            }
            {
                cv::Mat rot = R_world2cam_tmp[i];
                cv::Mat R(3, 3, CV_64FC1);
                if (rot.size() == cv::Size(3, 3))
                {
                    rot.convertTo(R, CV_64F);
                    R_world2cam_.push_back(R);
                }
                else
                {
                    Rodrigues(rot, R);
                    R_world2cam_.push_back(R);
                }
                cv::Mat tvec = t_world2cam_tmp[i];
                cv::Mat t;
                tvec.convertTo(t, CV_64F);
                t_world2cam_.push_back(t);
            }
        }

        CV_Assert(R_world2cam_.size() == t_world2cam_.size() &&
                  R_base2gripper_.size() == t_base2gripper_.size() &&
                  R_world2cam_.size() == R_base2gripper_.size());

        //push sets back into vector
        multi_R_world2cam_.push_back(R_world2cam_);
        multi_t_world2cam_.push_back(t_world2cam_);
        multi_R_base2gripper_.push_back(R_base2gripper_);
        multi_t_base2gripper_.push_back(t_base2gripper_);
    }

    std::vector<cv::Matx33d> multi_wRb;
    std::vector<cv::Matx31d> multi_wtb;
    multi_wRb.resize(multi_R_world2cam.size());
    multi_wtb.resize(multi_R_world2cam.size());
    cv::Matx33d cRg;
    cv::Matx31d ctg;

    calibrateRobotWorldHandEyeYifu(multi_R_world2cam_, multi_t_world2cam_, multi_R_base2gripper_, multi_t_base2gripper_,
                                   multi_wRb, multi_wtb, cRg, ctg);

    for (size_t i = 0; i < multi_R_world2cam_.size(); i++)
    {
        cv::Mat(multi_wRb[i]).copyTo(multi_R_base2world[i]);
        cv::Mat(multi_wtb[i]).copyTo(multi_t_base2world[i]);
    }
    cv::Mat(cRg).copyTo(R_gripper2cam);
    cv::Mat(ctg).copyTo(t_gripper2cam);
}
