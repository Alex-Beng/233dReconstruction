// 用于从charuco标定求解pnp进而完成定位
#include <bits/stdc++.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
using namespace std;
using namespace cv;

int sq_x = 5;
int sq_y = 7;
double sq_len = 26; // 单位是mm
double mk_len = 18;
int dict_flag = cv::aruco::DICT_4X4_50;


bool DetectCharuco(cv::Mat& frame,
        cv::Mat& charuco_corners,
        cv::Mat& charuco_ids) {
    
    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dict_flag));
    Ptr<aruco::CharucoBoard> charucoboard =
            aruco::CharucoBoard::create(sq_x, sq_y, sq_len, mk_len, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();


    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;

    // 寻找aruco的markers
    aruco::detectMarkers(frame, dictionary, corners, ids, detectorParams, rejected);

    // rf 得到更多aruco markers
    aruco::refineDetectedMarkers(frame, board, corners, ids, rejected);

    // 寻找charuco的角点
    if(ids.size() > 0){
        aruco::interpolateCornersCharuco(corners, ids, frame, charucoboard, 
                                            charuco_corners,
                                            charuco_ids);
        return true;
    }
    else {
        return false;
    }

}

void ReadInParams(cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
    ifstream inp_in("../Calib/InParams/in.param");
    
    camera_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
    dist_coeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar(0));

    string t_line;
    for (int i=0; i<3; i++) {
        getline(inp_in, t_line);
        stringstream t_line_ss;
        t_line_ss << t_line;        
        for (int j=0; j<3; j++) {
            t_line_ss >> camera_matrix.at<float>(i, j);
        }
    }
    
    getline(inp_in, t_line);
    stringstream t_line_ss;
    t_line_ss << t_line;    
    for (int i=0; i<5; i++) {
        t_line_ss >> dist_coeffs.at<float>(0, i);
    }
}

// 获得棋盘坐标系下各个角点的坐标
void GetObjectCoor(
        cv::Mat& charuco_ids,
        std::vector<cv::Point3f>& object_coors) {
    // cout<<charuco_ids.size()<<endl;
    for (int i=0; i<charuco_ids.rows; i++) {
        int t_id = charuco_ids.at<int>(i, 0);
        int t_x = t_id%4;
        int t_y = t_id/4;
        cv::Point3f t_pnt(
            t_x*sq_len,
            t_y*sq_len,
            0
        );
        object_coors.push_back(t_pnt);
    }
}

const char* keys  =
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }";

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    int cam_id = parser.get<int>("ci");

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    ReadInParams(camera_matrix, dist_coeffs);
    cout<<camera_matrix<<dist_coeffs<<endl;

    VideoCapture cp;
    cp.open(cam_id);
    // viz part
    viz::Viz3d window("window");
    // 世界坐标系
    viz::WCoordinateSystem world_coor(40.0);
    viz::WPlane plane(cv::Size(100, 100));
    // viz::WCloud

    window.showWidget("World",world_coor);
    window.showWidget("plane", plane);
    
    viz::WCoordinateSystem camer_coor(20.5);

    while (cp.grab()) {
        cv::Mat frame;
        cp.retrieve(frame);

        cv::Mat charuco_corners;
        cv::Mat charuco_ids;
        std::vector<cv::Point3f> object_coors;

        bool valid = DetectCharuco(frame, charuco_corners, charuco_ids);
        if (valid) {
            GetObjectCoor(charuco_ids, object_coors);
            // cout<<charuco_corners.size()<<' '<<object_coors.size()<<'y'<<endl;
            std::vector<cv::Point2f> t_corners;
            for (int i=0; i<object_coors.size(); i++) {
                cv::Point2f t_pnt = cv::Point2f(charuco_corners.at<float>(i, 0), 
                    charuco_corners.at<float>(i, 1));
                t_corners.push_back(t_pnt);
                if (i == 0) {
                    cv::circle(frame, t_pnt, 5, cv::Scalar(0, 0, 255), 3);
                } 
                
            }
            cv::Mat R, t;
            if (charuco_corners.total() > 4) {
                // cout<<t_corners.size()<<' '<<object_coors.size()<<endl;
                // cv::solvePnPRansac()
                // cv::solvePnPRansac(object_coors, t_corners, camera_matrix, dist_coeffs, R, t);
                cv::solvePnP(object_coors, t_corners, camera_matrix, dist_coeffs, R, t, false, CV_ITERATIVE);
                // cout<<R<<' '<<t<<endl;
                
                // 重投影检验一下
                std::vector<cv::Point3f> ori_pnt;
                std::vector<cv::Point2f> reproj_point;
                ori_pnt.push_back(cv::Point3f(0, 0, 0));
                ori_pnt.push_back(cv::Point3f(0, 100, 0));
                ori_pnt.push_back(cv::Point3f(100, 0, 0));
                ori_pnt.push_back(cv::Point3f(0, 0, 100));
                cv::projectPoints(ori_pnt, R, t, camera_matrix, dist_coeffs, reproj_point);
                // cout<<reproj_point[0]<<endl;

                cv::circle(frame, reproj_point[0], 5, cv::Scalar(0,255,255), 3);
                cv::circle(frame, reproj_point[1], 5, cv::Scalar(0,0,0), 3);
                cv::circle(frame, reproj_point[2], 5, cv::Scalar(0,255,0), 3);
                cv::circle(frame, reproj_point[3], 5, cv::Scalar(255,255,0), 3);
                cv::line(frame, reproj_point[0], reproj_point[1], cv::Scalar(0,0,255), 3);
                cv::line(frame, reproj_point[0], reproj_point[2], cv::Scalar(0,255,0), 3);
                cv::line(frame, reproj_point[0], reproj_point[3], cv::Scalar(255,0,0), 3);
                cv::imshow("reproj", frame);

                Affine3f pose(R, t);
                window.showWidget("Camera",camer_coor);
                window.setWidgetPose("Camera", pose);
                // window.setViewerPose(pose);
                window.spinOnce(10, false);
            }
        }
        cv::Mat frame_copy;
        frame.copyTo(frame_copy);
        aruco::drawDetectedCornersCharuco(frame_copy, charuco_corners, charuco_ids);

        cv::imshow("result", frame_copy);
        cv::waitKey(10);        
    }

    return 0;
}