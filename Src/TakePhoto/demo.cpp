#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

#define SAVE_PATH "../../Pic/"
#define COUNTER_INIT_NUM 0

int counter = COUNTER_INIT_NUM;


string GetNextPath() {
    stringstream t_ss;
    string t_s;

    t_ss << counter++;
    t_ss >> t_s;
    t_s = SAVE_PATH + t_s;
    t_s += ".jpg";
    cout<<t_s<<endl;
    return t_s;
}

bool FindCorners(cv::Mat& frame, std::vector<cv::Point2f>& img_corners, cv::Size board_size) {
    bool found;
    found = cv::findChessboardCorners(frame, board_size, img_corners);
    return found;
}

cv::Size board_size(7, 7);
std::vector<cv::Point2f> img_corners;


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

        if (corners_found) {
            cv::drawChessboardCorners(frame, board_size, img_corners, corners_found);
        }
        else {
            cv::drawChessboardCorners(frame, board_size, img_corners, corners_found);
        }
        cv::imshow("corners", frame);

        
        char key = cv::waitKey(1);
        if (key == 'q') {
            break;
        }
        else if (key == 's') {
            save_path = GetNextPath();
            cv::imwrite(save_path, frame_copy);
            cout<<"save to :"<<save_path<<endl;
        }
        else if (key == 'd') {        
            int t = system(("rm "+save_path).c_str());
            if (t) {
                cout<<"delete fail"<<endl;
            }
            else {
                cout<<"delete "+save_path<<endl;
                counter --;
            }

        }
    }
    return 0;
}
