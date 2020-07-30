#include "Segmenter.h"

Segmenter::Segmenter(int channel_flag=H, int channel_min=0, int channel_max=255, int cw=1) {
    cnl_flag_ = channel_flag;
    thre_min_ = channel_min;
    thre_max_ = channel_max;
    cw_ = cw;
}

void Segmenter::ImageProcesser(cv::Mat& image_in, cv::Mat& binary_image, cv::Mat& image_out) {
    cv::Mat used_channel = GetUsedChannel(image_in, cnl_flag_);
    // cv::GaussianBlur(used_channel, used_channel, cv::Size(3, 3), 0.0, 0.0);
    
    if (cw_) {
        binary_image = used_channel>thre_min_ & used_channel<thre_max_;
    }
    else {
        binary_image = used_channel<thre_min_ | used_channel>thre_max_;
    }

    // 分割出色卡里最大物体轮廓
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> heirachy;
    cv::Mat image_for_contours = binary_image.clone();

    cv::findContours(image_for_contours, contours, heirachy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    
    double max_area = -1;
    int max_cont_idx = -1;
    int object_cont_idx = -1;

    if (contours.size() == 0) {
        goto L1;
    }
    for (int i=0; i<contours.size(); i++) {
        double t_area = cv::contourArea(contours[i]);
        if (t_area > max_area) {
            max_area = t_area;
            max_cont_idx = i;
        }
    }
    // 最大颜色轮廓有儿子
    if (heirachy[max_cont_idx][2] != -1) {
        // 直接floodfill莽
        image_out = cv::Mat(image_in.rows+2, image_in.cols+2, CV_8UC1, cv::Scalar(0));
        binary_image.copyTo(image_out(cv::Rect(1, 1, image_in.cols, image_in.rows)));
        // image_out(cv::Rect(1, 1, image_in.cols, image_in.rows)) = binary_image.clone();
        // cv::imshow("ya", image_out);
        cv::Rect cc;
        cv::floodFill(image_out, cv::Point(0, 0), cv::Scalar(255), &cc, cv::Scalar(128), cv::Scalar(128), 8);
        image_out = image_out(cv::Rect(1, 1, image_in.cols, image_in.rows)).clone();



        // // 轮廓里面面积最大的儿子轮廓
        // max_area = -1;
        // object_cont_idx = -1;
        // for (int i=0; i<contours.size(); i++) {
        //     // 是这个轮廓儿子
        //     if (heirachy[i][3] == max_cont_idx) {
        //         double t_area = cv::contourArea(contours[i]);
        //         if (t_area > max_area) {
        //             max_area = t_area;
        //             object_cont_idx = i;
        //         }
        //     }
        // }

        // image_out = cv::Mat(image_in.rows, image_in.cols, CV_8UC1, cv::Scalar(0));
        // std::vector<cv::Point> objent_cont = contours[object_cont_idx];
        // for (int i=0; i<objent_cont.size(); i++) {
        //     image_out.at<uchar>(objent_cont[i]) = 255;
        // }
        
    }
    else {
L1:     image_out = cv::Mat(image_in.rows, image_in.cols, CV_8UC1, cv::Scalar(255));
        return ;
    }
}

cv::Mat Segmenter::GetUsedChannel(cv::Mat& src_image, int flag) {
    cv::Mat t;
    cv::Mat t_cs[3];
    switch (flag) {
    case 0:
    case 1:
    case 2:
        cv::cvtColor(src_image, t, CV_BGR2HSV_FULL);
        cv::split(t, t_cs);
        return t_cs[flag];
    case 3:
    case 4:
    case 5:
        cv::cvtColor(src_image, t, CV_BGR2Lab);
        cv::split(t, t_cs);
        return t_cs[flag - 3];
    }
}


