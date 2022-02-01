//
// Created by yifu wang on 2021/8/27.
//

#include <fstream>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include <opengv2/frame/Bodyframe.hpp>
#include <opengv2/system/SystemBase.hpp>
#include <opengv2/viewer/PointLandmarkViewer.hpp>
#include <cv_calib.hpp>
#include <time_measurement.hpp>
#include <chrono>

using namespace opengv2;
using namespace std;

void writeTofile(const string &file, double time, Eigen::Quaterniond &unitQ, Eigen::Vector3d &t);

void calibrateHandEye(const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tws,
                      const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tw1b,
                      Eigen::Quaterniond &unitQsb, Eigen::Vector3d &tsb,
                      std::vector<Eigen::Quaterniond> &multi_unitQww1, std::vector<Eigen::Vector3d> &multi_tww1,
                      char **argv);

void HandEyeErrors(const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tws,
                   const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tw1b,
                   Eigen::Quaterniond &unitQsb, Eigen::Vector3d &tsb,
                   std::vector<Eigen::Quaterniond> &multi_unitQww1, std::vector<Eigen::Vector3d> &multi_tww1,
                   char **argv, double &R_error, double &t_error);

double generateRandomNum(double factor)
{
    double randomize = 1 + ((((double)rand()) / ((double)RAND_MAX)) - 0.5) * factor * 2;
    return randomize;
}

void writeTofile(const string &file, Eigen::Quaterniond &unitQ, Eigen::Vector3d &t, bool AddToEnd = false)
{
    if (AddToEnd)
    {
        std::ofstream f(file, std::ofstream::app);
        f << std::fixed << std::setprecision(10);
        //timestamp tx ty tz qx qy qz qw
        f << t[0] << " " << t[1] << " " << t[2] << " "
          << unitQ.x() << " " << unitQ.y() << " " << unitQ.z() << " " << unitQ.w() << std::endl;
        f.close();
    }
    else
    {
        std::ofstream f(file, std::ofstream::trunc);
        f << std::fixed << std::setprecision(10);
        //timestamp tx ty tz qx qy qz qw
        f << t[0] << " " << t[1] << " " << t[2] << " "
          << unitQ.x() << " " << unitQ.y() << " " << unitQ.z() << " " << unitQ.w() << std::endl;
        f.close();
    }
}

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        cerr << endl
             << "Usage: ./handEyeCalib settingFilePath GTFilePath TrajectoryFile"
             << endl;
        return 1;
    }

    //Check settings file
    cv::FileStorage fsSettings(string(argv[1]), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << string(argv[1]) << endl;
        exit(-1);
    }

    // map part
    MapBase::Ptr map = make_shared<MapBase>();
    // viewer part
    PointLandmarkViewer::Ptr viewer = make_shared<PointLandmarkViewer>(string(argv[1]), map, nullptr);
    // Extrinsic parameters, to be calibrated
    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> Qsb;
    std::vector<Eigen::Vector3d> tsb;
    Qsb.emplace_back(1, 0, 0, 0);
    tsb.emplace_back(0, 0, 0);

    SystemBase SLAM(nullptr, map, viewer, nullptr, Qsb, tsb, nullptr, nullptr, string(argv[1]));

    /* // add landmarks
    bool isAsymmetric = true;
    fsSettings["Is_Pattern_Asymmetric"] >> isAsymmetric;
    int rows = fsSettings["BoardSize_Rows"];
    int cols = fsSettings["BoardSize_Cols"];
    double squareSize = fsSettings["Square_Size"];
    if (isAsymmetric)
    {
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
            {
                auto lm = std::make_shared<LandmarkBase>(j + i * cols, Eigen::Vector3d((2 * j + i % 2) * squareSize, i * squareSize, 0));
                SLAM.map->addLandmark(lm);
            }
    }
    else
    {
        for (int i = 0; i < rows; ++i)
            for (int j = 0; j < cols; ++j)
            {
                auto lm = std::make_shared<LandmarkBase>(j + i * cols, Eigen::Vector3d(j * squareSize, i * squareSize, 0));
                SLAM.map->addLandmark(lm);
            }
    } */

    //Get base DIR
    string base_dir = fsSettings["Input"];

    // load GT, cvTrajectory
    std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> Twbs, Tcps;
    for (size_t i = 2; i < argc; i = i + 2)
    {
        std::map<double, Eigen::Matrix<double, 7, 1>> Twb, Tcp;
        opengv2::SystemBase::loadTUM(base_dir + string(argv[i]), Twb);
        opengv2::SystemBase::loadTUM(base_dir + string(argv[i + 1]), Tcp);
        Twbs.push_back(Twb);
        Tcps.push_back(Tcp);
        std::cout << "GT " << i << " has " << Twb.size() << " frames." << std::endl;
        std::cout << "Camera " << i << " has " << Tcp.size() << " frames." << std::endl;
    }

    Eigen::Quaterniond unitQpb;
    Eigen::Vector3d tpb;
    std::vector<Eigen::Quaterniond> unitQcws;
    std::vector<Eigen::Vector3d> tcws;
    unitQcws.resize(Twbs.size());
    tcws.resize(Twbs.size());

    //Do Camera Calibration
    double R_error = 0;
    double t_error = 0;
    calibrateHandEye(Tcps, Twbs, unitQpb, tpb, unitQcws, tcws, argv);
    HandEyeErrors(Tcps, Twbs, unitQpb, tpb, unitQcws, tcws, argv, R_error, t_error);

    std::cout.precision(6);
    std::ofstream f(base_dir + "result/result.txt", std::ofstream::app);
    f << std::fixed << std::setprecision(10);
    //timestamp tx ty tz qx qy qz qw
    f << R_error << " " << t_error << std::endl;
    f.close();

    for (size_t i = 0; i < Tcps.size(); i++)
    {
        //size_t i = 0;
        Eigen::Quaterniond unitQwc = unitQcws[i].conjugate();
        Eigen::Vector3d twc = -(unitQwc * tcws[i]);
        //std::map<double, Eigen::Matrix<double, 7, 1>> Twp;
        for (const auto &it : Tcps[i])
        {
            Eigen::Quaterniond unitQcp(it.second.data());
            Eigen::Vector3d tcp(it.second.data() + 4);
            unitQcp.normalize();

            Eigen::Quaterniond unitQwp = unitQwc * unitQcp;
            Eigen::Vector3d twp = unitQwc * tcp + twc;

            vector<FrameBase::Ptr> frames(1);
            auto bf = std::make_shared<Bodyframe>(frames, it.first + i * 1000, twp, unitQwp);
            SLAM.map->addFrame(bf);
        }
    }

    //SLAM.loadGT(0, Twbs[0]);
    SLAM.viewer->updateViewer();

    std::cout << "press any key to exit..." << std::endl;
    std::cin.ignore();
    SLAM.shutdown();
    return 0;
}

void calibrateHandEye(const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tws,
                      const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tw1b,
                      Eigen::Quaterniond &unitQsb, Eigen::Vector3d &tsb,
                      std::vector<Eigen::Quaterniond> &multi_unitQww1, std::vector<Eigen::Vector3d> &multi_tww1,
                      char **argv)
{
    cv::FileStorage fsSettings(string(argv[1]), cv::FileStorage::READ);
    std::vector<std::vector<cv::Mat>> multi_Rbw1Set, multi_tbw1Set, multi_RswSet, multi_tswSet;
    int useMultiCam = fsSettings["UseMultiCam"];
    int useTsb = fsSettings["UseTsb"];
    unitQsb.w() = 0;

    for (size_t i = 0; i < multi_Tws.size(); i++)
    {
        auto &Tws = multi_Tws[i];
        auto &Tw1b = multi_Tw1b[i];
        auto &unitQww1 = multi_unitQww1[i];
        auto &tww1 = multi_tww1[i];

        std::vector<cv::Mat> Rbw1Set, tbw1Set, RswSet, tswSet;
        for (const auto &itr : Tws)
        {
            // Tbw1Set
            auto n1 = Tw1b.lower_bound(itr.first);
            if (n1 == Tw1b.end())
            {
                n1--;
            }
            else if (n1 != Tw1b.begin())
            {
                auto n2 = n1--;
                n1 = std::abs(itr.first - n1->first) > std::abs(itr.first - n2->first) ? n2 : n1;
            }

            auto itr1 = Tws.lower_bound(n1->first);
            if (itr1 == Tws.end())
            {
                itr1--;
            }
            else if (itr1 != Tws.begin())
            {
                auto itr2 = itr1--;
                itr1 = std::abs(n1->first - itr1->first) > std::abs(n1->first - itr2->first) ? itr2 : itr1;
            }

            if (itr1->first != itr.first || std::abs(n1->first - itr.first) > 3e-2)
                continue;

            Eigen::Quaterniond unitQw1b(n1->second.data());
            Eigen::Vector3d tw1b(n1->second.data() + 4);

            Eigen::Quaterniond unitQbw1 = unitQw1b.conjugate();
            Eigen::Vector3d tbw1 = -(unitQbw1 * tw1b);
            cv::Mat cv_Rbw1, cv_tbw1;
            cv::eigen2cv(unitQbw1.toRotationMatrix(), cv_Rbw1);
            cv::eigen2cv(tbw1, cv_tbw1);
            Rbw1Set.push_back(cv_Rbw1);
            tbw1Set.push_back(cv_tbw1);

            // TswSet
            Eigen::Quaterniond unitQws(itr.second.data());
            Eigen::Vector3d tws(itr.second.data() + 4);

            cv::Mat cv_Rsw, cv_tsw;
            cv::eigen2cv(unitQws.conjugate().toRotationMatrix(), cv_Rsw);
            cv::eigen2cv(Eigen::Vector3d(unitQws.conjugate() * (-tws)), cv_tsw);
            RswSet.push_back(cv_Rsw);
            tswSet.push_back(cv_tsw);

            //std::cout << "timestamps " << n1->first << " and " << itr.first << " are pairs." << std::endl;
            int NumofMeas = fsSettings["NumOfMeasures"];
            if (RswSet.size() > NumofMeas) //MUST BE REMOVED AFTER EXPERIMENT!!!!!!!!!!!!
            {
                cout << "Num of Measurements:" << RswSet.size() << std::endl;
                break;
            }
        }
        //std::cout << "Camera " << i << " has " << Rbw1Set.size() << " pairs." << std::endl;
        std::cout << "Calibration for " << string(argv[i * 2 + 2]) << " has " << Rbw1Set.size() << " pairs." << std::endl;

        multi_Rbw1Set.push_back(Rbw1Set);
        multi_tbw1Set.push_back(tbw1Set);
        multi_RswSet.push_back(RswSet);
        multi_tswSet.push_back(tswSet);

        //Use standard Handeye Calibration
        if (!useMultiCam)
        {
            cv::Mat cv_Rww1, cv_tww1;
            cv::Mat cv_Rsb, cv_tsb;

            switch (fsSettings["HandeyeSolver"])
            {
            case 0:
                calibrateRobotWorldHandEye(RswSet, tswSet, Rbw1Set, tbw1Set, cv_Rww1, cv_tww1, cv_Rsb, cv_tsb,
                                           cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH);
                break;

            case 1:
                calibrateRobotWorldHandEye(RswSet, tswSet, Rbw1Set, tbw1Set, cv_Rww1, cv_tww1, cv_Rsb, cv_tsb,
                                           cv::CALIB_ROBOT_WORLD_HAND_EYE_LI);
                break;

            default:
                break;
            }
            Eigen::Matrix3d Rsb_mono;
            Eigen::Vector3d tsb_mono;
            Eigen::Quaterniond unitQsb_mono;

            cv::cv2eigen(cv_Rsb, Rsb_mono);
            cv::cv2eigen(cv_tsb, tsb_mono);
            unitQsb_mono = Rsb_mono;
            tsb = tsb + tsb_mono;

            if (unitQsb_mono.w() > 0)
            {
                unitQsb.w() += unitQsb_mono.w();
                unitQsb.x() += unitQsb_mono.x();
                unitQsb.y() += unitQsb_mono.y();
                unitQsb.z() += unitQsb_mono.z();
            }
            else
            {
                unitQsb.w() -= unitQsb_mono.w();
                unitQsb.x() -= unitQsb_mono.x();
                unitQsb.y() -= unitQsb_mono.y();
                unitQsb.z() -= unitQsb_mono.z();
            }

            //unitQsb.normalize();
            std::cout << "Target extrinsic parameter for camera " << string(argv[i * 2 + 2]) << ": " << std::endl
                      << unitQsb_mono.coeffs().transpose() << std::endl
                      << tsb_mono.transpose() << std::endl;

            Eigen::Matrix3d Rww1;
            cv::cv2eigen(cv_Rww1, Rww1);
            cv::cv2eigen(cv_tww1, tww1);

            multi_unitQww1[i] = Rww1;
            multi_unitQww1[i].normalize();
            multi_tww1[i] = tww1;

            //Get Tw1w, which is the pose of camera inside the world cordinate
            Eigen::Vector3d tw1w;
            Eigen::Quaterniond unitQw1w;
            unitQw1w = multi_unitQww1[i].conjugate();
            tw1w = -(unitQw1w * tww1);

            /*         std::cout.precision(6);
                 std::cout << "Estimated extrinsic parameter: " << std::endl
                  << unitQsb.coeffs().transpose() << std::endl
                  << tsb.transpose() << std::endl; */

            std::cout.precision(6);
            std::cout << "Camera extrinsic parameter for camera " << string(argv[i * 2 + 2]) << ": " << std::endl
                      << (multi_unitQww1[0].conjugate() * unitQw1w.conjugate()).coeffs().transpose() << std::endl
                      << (multi_unitQww1[0] * (tw1w + (multi_unitQww1[0].conjugate() * multi_tww1[0]))).transpose() << std::endl;
        }
    }
    // Normalize Tsb
    if (!useMultiCam)
    {
        tsb = tsb / multi_Tws.size();
        unitQsb.normalize();
        std::cout << "Target extrinsic parameter for all camera " << std::endl
                  << unitQsb.coeffs().transpose() << std::endl
                  << tsb.transpose() << std::endl;
    }

    // Use Known Tsb
    if (!useTsb){
        //ToDo
    }
    
    //Use New Handeye Calibration
    if (useMultiCam)
    {
        std::cout << "------------New Approach------------" << std::endl;
        std::vector<cv::Mat> cv_multi_Rww1, cv_multi_tww1;
        cv::Mat cv_Rsb, cv_tsb;
        cv_multi_Rww1.resize(multi_RswSet.size());
        cv_multi_tww1.resize(multi_RswSet.size());
        calibrateRobotWorldHandEyeMulti(multi_RswSet, multi_tswSet, multi_Rbw1Set, multi_tbw1Set, cv_multi_Rww1, cv_multi_tww1, cv_Rsb, cv_tsb);

        Eigen::Matrix3d Rsb;
        cv::cv2eigen(cv_Rsb, Rsb);
        cv::cv2eigen(cv_tsb, tsb);
        unitQsb = Rsb;
        unitQsb.normalize();

        std::cout.precision(10);
        std::cout << "Estimated extrinsic parameter: " << std::endl
                  << unitQsb.coeffs().transpose() << std::endl
                  << tsb.transpose() << std::endl;

        for (size_t i = 0; i < multi_RswSet.size(); i++)
        {
            Eigen::Matrix3d Rww1;
            cv::cv2eigen(cv_multi_Rww1[i], Rww1);
            cv::cv2eigen(cv_multi_tww1[i], multi_tww1[i]);
            multi_unitQww1[i] = Rww1;
            multi_unitQww1[i].normalize();

            //Get Tw1w, which is the pose of camera inside the world cordinate
            Eigen::Vector3d tw1w;
            Eigen::Quaterniond unitQw1w;
            unitQw1w = multi_unitQww1[i].conjugate();
            tw1w = -(unitQw1w * multi_tww1[i]);

            std::cout.precision(6);
            std::cout << "Camera extrinsic parameter for camera " << string(argv[i * 2 + 2]) << ": " << std::endl
                      << (multi_unitQww1[0].conjugate() * unitQw1w.conjugate()).coeffs().transpose() << std::endl
                      << (multi_unitQww1[0] * (tw1w + (multi_unitQww1[0].conjugate() * multi_tww1[0]))).transpose() << std::endl;
/*                      std::cout.precision(6);
                       std::cout << "Camera extrinsic parameter for camera " << i << ": " << std::endl
                                 << unitQw1w.coeffs().transpose() << std::endl
                                 << tw1w.transpose() << std::endl;  */
        }
    }
}

void HandEyeErrors(const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tws,
                   const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tw1b,
                   Eigen::Quaterniond &unitQsb, Eigen::Vector3d &tsb,
                   std::vector<Eigen::Quaterniond> &multi_unitQww1, std::vector<Eigen::Vector3d> &multi_tww1,
                   char **argv, double &R_error, double &t_error)
{
    cv::FileStorage fsSettings(string(argv[1]), cv::FileStorage::READ);

    double total_terr = 0;
    double total_Rerr = 0;
    double count = 0;

    for (size_t i = 0; i < multi_Tws.size(); i++)
    {
        auto &Tws = multi_Tws[i];
        auto &Tw1b = multi_Tw1b[i];
        auto &unitQww1 = multi_unitQww1[i];
        auto &tww1 = multi_tww1[i];

        std::vector<cv::Mat> Rbw1Set, tbw1Set, RswSet, tswSet;
        for (const auto &itr : Tws)
        {
            // Tbw1Set
            auto n1 = Tw1b.lower_bound(itr.first);
            if (n1 == Tw1b.end())
            {
                n1--;
            }
            else if (n1 != Tw1b.begin())
            {
                auto n2 = n1--;
                n1 = std::abs(itr.first - n1->first) > std::abs(itr.first - n2->first) ? n2 : n1;
            }

            auto itr1 = Tws.lower_bound(n1->first);
            if (itr1 == Tws.end())
            {
                itr1--;
            }
            else if (itr1 != Tws.begin())
            {
                auto itr2 = itr1--;
                itr1 = std::abs(n1->first - itr1->first) > std::abs(n1->first - itr2->first) ? itr2 : itr1;
            }

            if (itr1->first != itr.first || std::abs(n1->first - itr.first) > 3e-2)
                continue;

            // Tw1bSet
            Eigen::Quaterniond unitQw1b(n1->second.data());
            Eigen::Vector3d tw1b(n1->second.data() + 4);

            Eigen::Quaterniond unitQbw1 = unitQw1b.conjugate();
            Eigen::Vector3d tbw1 = -(unitQbw1 * tw1b);

            // TswSet
            Eigen::Quaterniond unitQws(itr.second.data());
            Eigen::Vector3d tws(itr.second.data() + 4);

            Eigen::Quaterniond unitQsw = unitQws.conjugate();
            Eigen::Vector3d tsw = -(unitQsw * tws);

            // Tw1w
            Eigen::Quaterniond unitQw1w = unitQww1.conjugate();
            Eigen::Vector3d tw1w = -(unitQw1w * tww1);

            // Tbs
            Eigen::Quaterniond unitQbs = unitQsb.conjugate();
            Eigen::Vector3d tbs = -(unitQbs * tsb);

            //Error Term
            //(RY*RB)^T * (RA*RX) => (Qbs*Qsw)^T * (Qbw1 * Qw1w)
            Eigen::Quaterniond unitQerr = unitQws * unitQsb * unitQbw1 * unitQw1w;
            //(RA*tX + tA) - (RY*tB + tY) => (Qbw1*tw1w + tbw1) - (Qbs*tsw + tbs)
            Eigen::Vector3d terr = (unitQbw1 * tw1w + tbw1) - (unitQbs * tsw + tbs);

            Eigen::AngleAxisd axisangle(unitQerr);
            total_Rerr = total_Rerr + axisangle.angle();
            total_terr = total_terr + terr.norm();
            count = count + 1;
        }
    }
    t_error = total_terr / count;
    R_error = total_Rerr / count * 180 / M_PI;
    /*     std::cout.precision(6);
    std::cout << "Extrinsic error for camera " << std::endl
              << (total_Rerr / count * 180 / M_PI) << std::endl
              << (total_terr / count) << std::endl; */
}
