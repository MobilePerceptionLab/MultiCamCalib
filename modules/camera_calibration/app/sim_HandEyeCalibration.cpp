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
                      char **argv, double noise, int numofmeas);

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

    //SystemBase SLAM(nullptr, map, viewer, nullptr, Qsb, tsb, nullptr, nullptr, string(argv[1]));

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

    srand((int)time(0));
    double operation_time = 0;
    for (size_t numofmeas = 3; numofmeas < 41; numofmeas++)
    {
        for (size_t i = 10; i < 11; i++)
        {
            double Sum_Rerror = 0;
            double Sum_terror = 0;
            size_t iteration = 50;
            struct timeval tic;
            struct timeval toc;
            gettimeofday(&tic, 0);
            for (size_t j = 0; j < iteration; j++) //iteration times
            {
                double R_error = 0;
                double t_error = 0;
                double noise = 0.005 * i;
                calibrateHandEye(Tcps, Twbs, unitQpb, tpb, unitQcws, tcws, argv, noise, numofmeas);
                HandEyeErrors(Tcps, Twbs, unitQpb, tpb, unitQcws, tcws, argv, R_error, t_error);
                Sum_Rerror = Sum_Rerror + R_error;
                Sum_terror = Sum_terror + t_error;
            }
            gettimeofday(&toc, 0);
            operation_time = operation_time + TIMETODOUBLE(timeval_minus(toc, tic));

            std::cout.precision(6);
            string base_dir = fsSettings["Input"];
            std::ofstream f(base_dir + "result/result.txt", std::ofstream::app);
            f << std::fixed << std::setprecision(10);
            //timestamp tx ty tz qx qy qz qw
            f << Sum_Rerror / iteration << " " << Sum_terror / iteration << std::endl;
            f.close();
        }
        std::cout.precision(6);
        std::cout << "operation time:" << operation_time / 3000 << std::endl;
    }

    /* for (size_t i = 0; i < Tcps.size(); i++)
    {
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
            auto bf = std::make_shared<Bodyframe>(frames, it.first, twp, unitQwp);
            SLAM.map->addFrame(bf);
        }
    }

    //SLAM.loadGT(0, Twb);
    SLAM.viewer->updateViewer();

    std::cout << "press any key to exit..." << std::endl;
    std::cin.ignore();
    SLAM.shutdown(); */
    return 0;
}

void calibrateHandEye(const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tws,
                      const std::vector<std::map<double, Eigen::Matrix<double, 7, 1>>> &multi_Tw1b,
                      Eigen::Quaterniond &unitQsb, Eigen::Vector3d &tsb,
                      std::vector<Eigen::Quaterniond> &multi_unitQww1, std::vector<Eigen::Vector3d> &multi_tww1,
                      char **argv, double noise, int numofmeas)
{
    cv::FileStorage fsSettings(string(argv[1]), cv::FileStorage::READ);
    std::vector<std::vector<cv::Mat>> multi_Rbw1Set, multi_tbw1Set, multi_RswSet, multi_tswSet;
    int useMultiCam = fsSettings["UseMultiCam"];
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

            //double factor = fsSettings["Noise_Rate"]; // Used for adding noise here
            double factor = noise; // Used for adding noise here

            Eigen::Quaterniond unitQw1b(n1->second.data());
            Eigen::Vector3d tw1b(n1->second.data() + 4);

            /*Add noise*/
            unitQw1b.x() = unitQw1b.x() * generateRandomNum(factor);
            unitQw1b.y() = unitQw1b.y() * generateRandomNum(factor);
            unitQw1b.z() = unitQw1b.z() * generateRandomNum(factor);
            unitQw1b.w() = unitQw1b.w() * generateRandomNum(factor);
            unitQw1b.normalize();

            tw1b[0] = tw1b[0] * generateRandomNum(factor);
            tw1b[1] = tw1b[1] * generateRandomNum(factor);
            tw1b[2] = tw1b[2] * generateRandomNum(factor);
            /*Add noise*/

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

            /*Add noise*/
            unitQws.x() = unitQws.x() * generateRandomNum(factor);
            unitQws.y() = unitQws.y() * generateRandomNum(factor);
            unitQws.z() = unitQws.z() * generateRandomNum(factor);
            unitQws.w() = unitQws.w() * generateRandomNum(factor);
            unitQws.normalize();

            tws[0] = tws[0] * generateRandomNum(factor);
            tws[1] = tws[1] * generateRandomNum(factor);
            tws[2] = tws[2] * generateRandomNum(factor);
            /*Add noise*/

            cv::Mat cv_Rsw, cv_tsw;
            cv::eigen2cv(unitQws.conjugate().toRotationMatrix(), cv_Rsw);
            cv::eigen2cv(Eigen::Vector3d(unitQws.conjugate() * (-tws)), cv_tsw);
            RswSet.push_back(cv_Rsw);
            tswSet.push_back(cv_tsw);

            /* // save GT
            std::ofstream f("/home/ifwang/software/MultiSensorsCalib/parameter/results/simulation/sim_gt_17082012.txt", std::ofstream::app);
            f << std::fixed << std::setprecision(10);
            //qw qx qy qz tx ty tz

            //Tww1 Tbs
            Eigen::Quaterniond unitQw1w(0, -0.7, 0, 0.7);
            Eigen::Vector3d tw1w(1.1, 0.9, -2.1);
            Eigen::Quaterniond unitQsb(-0.2, 0.55, -0.26, 0.75);
            Eigen::Vector3d tsb(0.1, 0.05, 0);
            unitQw1w.normalize();
            unitQsb.normalize();

            unitQw1b = unitQw1w * unitQws * unitQsb;
            tw1b = unitQw1w * (unitQws * tsb + tws) + tw1w;

            //timestamp tx ty tz qx qy qz qw
            f << itr.first << " " << tw1b[0] << " " << tw1b[1] << " " << tw1b[2] << " "
              << unitQw1b.x() << " " << unitQw1b.y() << " " << unitQw1b.z() << " " << unitQw1b.w() << std::endl;
            f.close(); */

            if (RswSet.size() > numofmeas) //MUST BE REMOVED AFTER EXPERIMENT!!!!!!!!!!!!
            {
                cout << "wtf:" << RswSet.size() << std::endl;
                break;
            }
        }
        cout << "count:" << RswSet.size() << std::endl;
        //std::cout << "Camera " << i << " has " << Rbw1Set.size() << " pairs." << std::endl;
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
            Eigen::Matrix3d Rsb;
            cv::cv2eigen(cv_Rsb, Rsb);
            cv::cv2eigen(cv_tsb, tsb);
            unitQsb = Rsb;
            unitQsb.normalize();

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
            /*         std::cout.precision(6);
                       std::cout << "Camera extrinsic parameter for camera " << i << ": " << std::endl
                                 << unitQw1w.coeffs().transpose() << std::endl
                                 << tw1w.transpose() << std::endl; */
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
