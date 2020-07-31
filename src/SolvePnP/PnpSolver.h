#ifndef PNP_SOLVER_H
#define PNP_SOLVER_H

// 用于从charuco标定求解pnp进而完成定位
#include <bits/stdc++.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
using namespace std;
using namespace cv;


class PnpSolver {
public:
    PnpSolver();
    PnpSolver(int sq_x, int sq_y, double sq_len, double mk_len, int dict_flag, string);
    // 输入图片，若返回true，则解出外参R、t。否则没有
    bool Solve(cv::Mat& image_in, cv::Mat& R, cv::Mat& t);
    bool DetectCharuco(cv::Mat& frame, cv::Mat& charuco_corners, cv::Mat& charuco_ids);
    bool ReadInParams(cv::Mat& camera_matrix, cv::Mat& dist_coeffs, std::string in_file);
    void GetObjectCoor(cv::Mat& charuco_ids, std::vector<cv::Point3f>& object_coors);
public:
    int sq_x_;
    int sq_y_;
    double sq_len_;
    double mk_len_;
    int dict_flag_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

#endif