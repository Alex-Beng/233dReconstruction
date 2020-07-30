#include "PnpSolver.h"

PnpSolver::PnpSolver() {
    ;
}

PnpSolver::PnpSolver(int sq_x, int sq_y, double sq_len, double mk_len, int dict_flag, string inp_file_path) {
    sq_x_ = sq_x;
    sq_y_ = sq_y;
    sq_len_ = sq_len;
    mk_len_ = mk_len;
    dict_flag_ = dict_flag;
    ReadInParams(inp_file_path, camera_matrix_, dist_coeffs_);
    cout<<camera_matrix_<<endl<<dist_coeffs_<<endl;
}

bool PnpSolver::Solve(cv::Mat& frame, cv::Mat& R, cv::Mat& t) {
    cv::Mat charuco_corners;
    cv::Mat charuco_ids;
    std::vector<cv::Point3f> object_coors;

    bool valid = DetectCharuco(frame, charuco_corners, charuco_ids);

    if (valid) {
        GetObjectCoor(charuco_ids, object_coors);
        std::vector<cv::Point2f> t_corners;
        for (int i=0; i<object_coors.size(); i++) {
            cv::Point2f t_pnt = cv::Point2f(charuco_corners.at<float>(i, 0), 
                charuco_corners.at<float>(i, 1));
            t_corners.push_back(t_pnt);
            if (i == 0) {
                cv::circle(frame, t_pnt, 5, cv::Scalar(0, 0, 255), 3);
            } 
            
        }
        // cv::Mat R, t;
        if (charuco_corners.total() > 4) {
            cv::solvePnP(object_coors, t_corners, camera_matrix_, dist_coeffs_, R, t, false, CV_ITERATIVE);
            return true;
            // 重投影检验一下
            // std::vector<cv::Point3f> ori_pnt;
            // std::vector<cv::Point2f> reproj_point;
            // ori_pnt.push_back(cv::Point3f(0, 0, 0));
            // ori_pnt.push_back(cv::Point3f(0, 100, 0));
            // ori_pnt.push_back(cv::Point3f(100, 0, 0));
            // ori_pnt.push_back(cv::Point3f(0, 0, 100));
            // cv::projectPoints(ori_pnt, R, t, camera_matrix_, dist_coeffs_, reproj_point);
            // // cout<<reproj_point[0]<<endl;

            // cv::circle(frame, reproj_point[0], 5, cv::Scalar(0,255,255), 3);
            // cv::circle(frame, reproj_point[1], 5, cv::Scalar(0,0,0), 3);
            // cv::circle(frame, reproj_point[2], 5, cv::Scalar(0,255,0), 3);
            // cv::circle(frame, reproj_point[3], 5, cv::Scalar(255,255,0), 3);
            // cv::line(frame, reproj_point[0], reproj_point[1], cv::Scalar(0,0,255), 3);
            // cv::line(frame, reproj_point[0], reproj_point[2], cv::Scalar(0,255,0), 3);
            // cv::line(frame, reproj_point[0], reproj_point[3], cv::Scalar(255,0,0), 3);
            // cv::imshow("reproj", frame);
            // cv::waitKey();
        }
    }
    return false;
}
bool PnpSolver::DetectCharuco(cv::Mat& frame, cv::Mat& charuco_corners, cv::Mat& charuco_ids) {
    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dict_flag_));
    Ptr<aruco::CharucoBoard> charucoboard =
            aruco::CharucoBoard::create(sq_x_, sq_y_, sq_len_, mk_len_, dictionary);
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
void PnpSolver::ReadInParams(string file_path, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
    ifstream inp_in(file_path);
    
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
void PnpSolver::GetObjectCoor(cv::Mat& charuco_ids, std::vector<cv::Point3f>& object_coors) {
    for (int i=0; i<charuco_ids.rows; i++) {
        int t_id = charuco_ids.at<int>(i, 0);
        int t_x = t_id%4;
        int t_y = t_id/4;
        cv::Point3f t_pnt(
            t_x*sq_len_,
            t_y*sq_len_,
            0
        );
        object_coors.push_back(t_pnt);
    }

}