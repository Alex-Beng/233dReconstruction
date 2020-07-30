#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int counter = 0;
cv::Size board_size(7, 7);  
cv::Size square_size(26, 26); // 标定板方格实际大小 mm
std::vector<cv::Point2f> img_corners;
std::vector<std::vector<cv::Point2f>> all_img_corners;

// 相机参数部分
cv::Mat K = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0)); // 内参
cv::Mat Dis = cv::Mat(1, 5, CV_32FC1, cv::Scalar(0)); // 畸变参数
std::vector<cv::Mat> Rs; // 每幅图外参的R
std::vector<cv::Mat> ts; // 每幅图外参的t



bool FindCorners(cv::Mat& frame, std::vector<cv::Point2f>& img_corners, cv::Size board_size) {
    bool found;
    found = cv::findChessboardCorners(frame, board_size, img_corners);
    return found;
}

// 用于初始化标定时用到空间点 & 角点数量
void InitCaliThings(std::vector<std::vector<cv::Point3f>>& object_points,
                    std::vector<int>& point_count) {
    for (int i=0; i<counter; i++) {
        std::vector<cv::Point3f> t_pnt_set;
        for (int h=0; h<board_size.height; h++) {
            for (int w=0; w<board_size.width; w++) {
                cv::Point3f t_real_point(
                    h*square_size.width,
                    w*square_size.width,
                    0
                );
                t_pnt_set.push_back(t_real_point);
            }
        }
        object_points.push_back(t_pnt_set);
        point_count.push_back(board_size.width*board_size.height);
    }
}

int main(int argc, char const *argv[]) {
    int open_cam_idx = 0;
    if (argc < 2) {
        ;
    }    
    else if (argc == 2){
        open_cam_idx = argv[1][0]-'0';
    }
    cout<<"using web cam : "<<open_cam_idx<<endl;

    cv::VideoCapture cp(open_cam_idx);
    cv::Mat frame;
    string save_path;
    bool corners_found;


    while (true) {
        cp >> frame;
        if (frame.empty()) {
            cout<<"empty frame !"<<endl;
            cp.open(open_cam_idx);
            continue;
        }
        // 验证找角点
        corners_found = FindCorners(frame, img_corners, board_size);

        cv::Mat frame_copy;
        frame.copyTo(frame_copy);

        cv::imshow("233", frame);

        // 对于粗提取的角点进行绘制
        cv::drawChessboardCorners(frame, board_size, img_corners, corners_found);
        cv::imshow("corners", frame);

        
        char key = cv::waitKey(1);
        if (key == 'q') {
            break;
        }
        // 向队列中增加角点
        else if (key == 'a') {
            // 获得亚像素级角点
            cv::Mat t_image;
            cv::cvtColor(frame_copy, t_image, CV_BGR2GRAY);
            cv::find4QuadCornerSubpix(t_image, img_corners, board_size);
            all_img_corners.push_back(img_corners);
            counter++;
            cout<<"get "<<counter<<" image"<<endl;
        }
        // 进行标定
        else if (key == 'c') {
            cout<<"begin calibration..."<<endl;
            std::vector<std::vector<cv::Point3f>> object_points;
            std::vector<int> point_count;
            InitCaliThings(object_points, point_count);
            
            calibrateCamera(object_points, 
                            all_img_corners, 
                            frame_copy.size(),
                            K,
                            Dis,
                            Rs,ts
                            );
            cout<<"K:"<<endl<<K<<endl
                <<"Dis:"<<endl<<Dis<<endl;

            cout<<"calibration end"<<endl<<endl;
        }

    }
    return 0;
}
