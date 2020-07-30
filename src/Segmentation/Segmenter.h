#ifndef SEGMENTER_H
#define SEGMENTER_H

// 使用的通道
enum { H,S,V,L,A,B };

#include <bits/stdc++.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
using namespace std;
using namespace cv;

// 注意输出图 背景为255 物体为0
class Segmenter {
public:
    Segmenter(int channel_flag, int channel_min, int channel_max, int cw);
    void ImageProcesser(cv::Mat& image_in, cv::Mat& binary_image, cv::Mat& image_out);
    cv::Mat GetUsedChannel(cv::Mat& src_image, int flag);
public:
    int cnl_flag_;
    int thre_min_;
    int thre_max_;
    int cw_; // 通道阈值化方向(针对Hue)
};

#endif