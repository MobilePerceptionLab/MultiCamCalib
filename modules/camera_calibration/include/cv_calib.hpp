//
// Created by huangkun on 2020/9/28.
//

#ifndef OPENGV2_CV_CALIB_HPP
#define OPENGV2_CV_CALIB_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

namespace cv
{
    /*
     * The function attempts to determine whether the input image contains a grid of circles.
     * If it is, the function locates centers of the circles. The function returns a non-zero value
     * if all of the centers have been found and they have been placed in a certain order
     * (row by row, left to right in every row). Otherwise,
     * if the function fails to find all the corners or reorder them, it returns 0.
     */
    bool findCirclesGrid(const std::vector<Point2f> &points_, Size patternSize,
                         OutputArray _centers, int flags,
                         const CirclesGridFinderParameters &parameters_ = CirclesGridFinderParameters());

    void calibrateRobotWorldHandEyeMulti(std::vector<std::vector<cv::Mat>> &multi_R_world2cam, std::vector<std::vector<cv::Mat>> &multi_t_world2cam,
                                         std::vector<std::vector<cv::Mat>> &multi_R_base2gripper, std::vector<std::vector<cv::Mat>> &multi_t_base2gripper,
                                         std::vector<cv::Mat> &multi_R_base2world, std::vector<cv::Mat> &multi_t_base2world,
                                         cv::Mat &R_gripper2cam, cv::Mat &t_gripper2cam);

    void RecoverBase2World(std::vector<std::vector<cv::Mat>> &multi_R_world2cam, std::vector<std::vector<cv::Mat>> &multi_t_world2cam,
                           std::vector<std::vector<cv::Mat>> &multi_R_base2gripper, std::vector<std::vector<cv::Mat>> &multi_t_base2gripper,
                           std::vector<cv::Mat> &multi_R_base2world, std::vector<cv::Mat> &multi_t_base2world,
                           cv::Mat &R_gripper2cam, cv::Mat &t_gripper2cam);
}

#endif //OPENGV2_CV_CALIB_HPP
