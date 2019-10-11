// 用于从charuco标定求解pnp进而完成定位
#include <bits/stdc++.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
using namespace std;
using namespace cv;

int sq_x = 5;
int sq_y = 7;
double sq_len = 29; // 单位是mm
double mk_len = 19.5;
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

void CreatPointCloud(std::vector<cv::Vec3f>& cloud, std::vector<cv::Vec3b>& color) {
    for (int x=-100; x<0; x++) {
        for (int y=0; y<100; y++) {
            for (int z=0; z<100; z++) {
                cloud.push_back(
                cv::Vec3f(x, y, z)
                );
                color.push_back(
                    cv::Vec3b(0, 255, 0)
                    // cv::Vec3b(NAN, NAN, NAN)
                );
            }
        }
    }
}

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
    // viz::WPlane plane(cv::Size(100, 100));
    std::vector<cv::Vec3f> cloud;
    std::vector<cv::Vec3b> color;
    CreatPointCloud(cloud, color);
    viz::WCloud cloud_widget(cloud, color);

    window.showWidget("World",world_coor);
    // window.showWidget("plane", plane);
    window.showWidget("point_cloud", cloud_widget);
    
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

                cv::projectPoints(cloud, R, t, camera_matrix, dist_coeffs, reproj_point);
                for (int i=0; i<reproj_point.size(); i++) {
                    cv::circle(frame, reproj_point[i], 1, cv::Scalar(0, 255, 0), 1);
                }

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


// #include <iostream>
// #include <opencv2/core.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/calib3d.hpp>

// using namespace std;
// using namespace cv;

// namespace
// {
// enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

// void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
// {
//     corners.resize(0);

//     switch (patternType)
//     {
//     case CHESSBOARD:
//     case CIRCLES_GRID:
//         for( int i = 0; i < boardSize.height; i++ )
//             for( int j = 0; j < boardSize.width; j++ )
//                 corners.push_back(Point3f(float(j*squareSize),
//                                           float(i*squareSize), 0));
//         break;

//     case ASYMMETRIC_CIRCLES_GRID:
//         for( int i = 0; i < boardSize.height; i++ )
//             for( int j = 0; j < boardSize.width; j++ )
//                 corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
//                                           float(i*squareSize), 0));
//         break;

//     default:
//         CV_Error(Error::StsBadArg, "Unknown pattern type\n");
//     }
// }

// //! [compute-homography]
// Mat computeHomography(const Mat &R_1to2, const Mat &tvec_1to2, const double d_inv, const Mat &normal)
// {
//     Mat homography = R_1to2 + d_inv * tvec_1to2*normal.t();
//     return homography;
// }
// //! [compute-homography]

// Mat computeHomography(const Mat &R1, const Mat &tvec1, const Mat &R2, const Mat &tvec2,
//                       const double d_inv, const Mat &normal)
// {
//     Mat homography = R2 * R1.t() + d_inv * (-R2 * R1.t() * tvec1 + tvec2) * normal.t();
//     return homography;
// }

// //! [compute-c2Mc1]
// void computeC2MC1(const Mat &R1, const Mat &tvec1, const Mat &R2, const Mat &tvec2,
//                   Mat &R_1to2, Mat &tvec_1to2)
// {
//     //c2Mc1 = c2Mo * oMc1 = c2Mo * c1Mo.inv()
//     R_1to2 = R2 * R1.t();
//     tvec_1to2 = R2 * (-R1.t()*tvec1) + tvec2;
// }
// //! [compute-c2Mc1]

// void homographyFromCameraDisplacement(const string &img1Path, const string &img2Path, const Size &patternSize,
//                                       const float squareSize, const string &intrinsicsPath)
// {
//     Mat img1 = imread(img1Path);
//     Mat img2 = imread(img2Path);

//     //! [compute-poses]
//     vector<Point2f> corners1, corners2;
//     bool found1 = findChessboardCorners(img1, patternSize, corners1);
//     bool found2 = findChessboardCorners(img2, patternSize, corners2);

//     if (!found1 || !found2)
//     {
//         cout << "Error, cannot find the chessboard corners in both images." << endl;
//         return;
//     }

//     vector<Point3f> objectPoints;
//     calcChessboardCorners(patternSize, squareSize, objectPoints);

//     FileStorage fs(intrinsicsPath, FileStorage::READ);
//     Mat cameraMatrix, distCoeffs;
//     fs["camera_matrix"] >> cameraMatrix;
//     fs["distortion_coefficients"] >> distCoeffs;

//     Mat rvec1, tvec1;
//     solvePnP(objectPoints, corners1, cameraMatrix, distCoeffs, rvec1, tvec1);
//     Mat rvec2, tvec2;
//     solvePnP(objectPoints, corners2, cameraMatrix, distCoeffs, rvec2, tvec2);
//     //! [compute-poses]

//     Mat img1_copy_pose = img1.clone(), img2_copy_pose = img2.clone();
//     Mat img_draw_poses;
//     drawFrameAxes(img1_copy_pose, cameraMatrix, distCoeffs, rvec1, tvec1, 2*squareSize);
//     drawFrameAxes(img2_copy_pose, cameraMatrix, distCoeffs, rvec2, tvec2, 2*squareSize);
//     hconcat(img1_copy_pose, img2_copy_pose, img_draw_poses);
//     imshow("Chessboard poses", img_draw_poses);

//     //! [compute-camera-displacement]
//     Mat R1, R2;
//     Rodrigues(rvec1, R1);
//     Rodrigues(rvec2, R2);

//     Mat R_1to2, t_1to2;
//     computeC2MC1(R1, tvec1, R2, tvec2, R_1to2, t_1to2);
//     Mat rvec_1to2;
//     Rodrigues(R_1to2, rvec_1to2);
//     //! [compute-camera-displacement]

//     //! [compute-plane-normal-at-camera-pose-1]
//     Mat normal = (Mat_<double>(3,1) << 0, 0, 1);
//     Mat normal1 = R1*normal;
//     //! [compute-plane-normal-at-camera-pose-1]

//     //! [compute-plane-distance-to-the-camera-frame-1]
//     Mat origin(3, 1, CV_64F, Scalar(0));
//     Mat origin1 = R1*origin + tvec1;
//     double d_inv1 = 1.0 / normal1.dot(origin1);
//     //! [compute-plane-distance-to-the-camera-frame-1]

//     //! [compute-homography-from-camera-displacement]
//     Mat homography_euclidean = computeHomography(R_1to2, t_1to2, d_inv1, normal1);
//     Mat homography = cameraMatrix * homography_euclidean * cameraMatrix.inv();

//     homography /= homography.at<double>(2,2);
//     homography_euclidean /= homography_euclidean.at<double>(2,2);
//     //! [compute-homography-from-camera-displacement]

//     //Same but using absolute camera poses instead of camera displacement, just for check
//     Mat homography_euclidean2 = computeHomography(R1, tvec1, R2, tvec2, d_inv1, normal1);
//     Mat homography2 = cameraMatrix * homography_euclidean2 * cameraMatrix.inv();

//     homography_euclidean2 /= homography_euclidean2.at<double>(2,2);
//     homography2 /= homography2.at<double>(2,2);

//     cout << "\nEuclidean Homography:\n" << homography_euclidean << endl;
//     cout << "Euclidean Homography 2:\n" << homography_euclidean2 << endl << endl;

//     //! [estimate-homography]
//     Mat H = findHomography(corners1, corners2);
//     cout << "\nfindHomography H:\n" << H << endl;
//     //! [estimate-homography]

//     cout << "homography from camera displacement:\n" << homography << endl;
//     cout << "homography from absolute camera poses:\n" << homography2 << endl << endl;

//     //! [warp-chessboard]
//     Mat img1_warp;
//     warpPerspective(img1, img1_warp, H, img1.size());
//     //! [warp-chessboard]

//     Mat img1_warp_custom;
//     warpPerspective(img1, img1_warp_custom, homography, img1.size());
//     imshow("Warped image using homography computed from camera displacement", img1_warp_custom);

//     Mat img_draw_compare;
//     hconcat(img1_warp, img1_warp_custom, img_draw_compare);
//     imshow("Warped images comparison", img_draw_compare);

//     Mat img1_warp_custom2;
//     warpPerspective(img1, img1_warp_custom2, homography2, img1.size());
//     imshow("Warped image using homography computed from absolute camera poses", img1_warp_custom2);

//     waitKey();
// }

// const char* params
//     = "{ help h         |       | print usage }"
//       "{ image1         | ../data/left02.jpg | path to the source chessboard image }"
//       "{ image2         | ../data/left01.jpg | path to the desired chessboard image }"
//       "{ intrinsics     | ../data/left_intrinsics.yml | path to camera intrinsics }"
//       "{ width bw       | 9     | chessboard width }"
//       "{ height bh      | 6     | chessboard height }"
//       "{ square_size    | 0.025 | chessboard square size }";
// }

// int main(int argc, char *argv[])
// {
//     CommandLineParser parser(argc, argv, params);

//     if (parser.has("help"))
//     {
//         parser.about("Code for homography tutorial.\n"
//             "Example 3: homography from the camera displacement.\n");
//         parser.printMessage();
//         return 0;
//     }

//     Size patternSize(parser.get<int>("width"), parser.get<int>("height"));
//     float squareSize = (float) parser.get<double>("square_size");
//     homographyFromCameraDisplacement(parser.get<String>("image1"),
//                                      parser.get<String>("image2"),
//                                      patternSize, squareSize,
//                                      parser.get<String>("intrinsics"));

//     return 0;
// }



// // #include <opencv2/opencv.hpp>
// // //#include <opencv2/aruco.hpp>
// // #include <opencv2/aruco/charuco.hpp>
// // #include <vector>
// // #include <iostream>
 
// // using namespace std;
// // using namespace cv;
 
// // void createCharucoBoard(cv::Mat &boardImage)
// // {
// // 	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
// // 	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04, 0.02, dictionary);
// // 	board->draw(cv::Size(3600, 3600), boardImage);
// // }
 
// // int main()
// // {
 
// // 	cv::Mat charuco_boardImage;
// // 	createCharucoBoard(charuco_boardImage);
// //   cv::imshow("ya", charuco_boardImage);
// //   cv::waitKey();
// // 	return 0;
// // }

// // // #include <opencv2/highgui.hpp>
// // // #include <opencv2/aruco/charuco.hpp>

// // // using namespace cv;

// // // namespace {
// // // const char* about = "Create a ChArUco board image";
// // // const char* keys  =
// // //         "{@outfile |<none> | Output image }"
// // //         "{w        |       | Number of squares in X direction }"
// // //         "{h        |       | Number of squares in Y direction }"
// // //         "{sl       |       | Square side length (in pixels) }"
// // //         "{ml       |       | Marker side length (in pixels) }"
// // //         "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
// // //         "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
// // //         "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
// // //         "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
// // //         "{m        |       | Margins size (in pixels). Default is (squareLength-markerLength) }"
// // //         "{bb       | 1     | Number of bits in marker borders }"
// // //         "{si       | false | show generated image }";
// // // }

// // // int main(int argc, char *argv[]) {
// // //     CommandLineParser parser(argc, argv, keys);
// // //     parser.about(about);

// // //     if(argc < 7) {
// // //         parser.printMessage();
// // //         return 0;
// // //     }

// // //     int squaresX = parser.get<int>("w");
// // //     int squaresY = parser.get<int>("h");
// // //     int squareLength = parser.get<int>("sl");
// // //     int markerLength = parser.get<int>("ml");
// // //     int dictionaryId = parser.get<int>("d");
// // //     int margins = squareLength - markerLength;
// // //     if(parser.has("m")) {
// // //         margins = parser.get<int>("m");
// // //     }

// // //     int borderBits = parser.get<int>("bb");
// // //     bool showImage = parser.get<bool>("si");

// // //     String out = parser.get<String>(0);

// // //     if(!parser.check()) {
// // //         parser.printErrors();
// // //         return 0;
// // //     }

// // //     Ptr<aruco::Dictionary> dictionary =
// // //         aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

// // //     Size imageSize;
// // //     imageSize.width = squaresX * squareLength + 2 * margins;
// // //     imageSize.height = squaresY * squareLength + 2 * margins;

// // //     Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(squaresX, squaresY, (float)squareLength,
// // //                                                             (float)markerLength, dictionary);

// // //     // show created board
// // //     Mat boardImage;
// // //     board->draw(imageSize, boardImage, margins, borderBits);

// // //     if(showImage) {
// // //         imshow("board", boardImage);
// // //         waitKey(0);
// // //     }

// // //     imwrite(out, boardImage);

// // //     return 0;
// // // }



// // // // #include <iostream>
// // // // #include <opencv2/viz.hpp>
// // // // #include <opencv2/highgui.hpp>
// // // // #include <opencv2/calib3d.hpp>
 
// // // // using namespace std;
// // // // using namespace cv;
// // // // int main()
// // // // {
// // // //     //1.创建可视化窗口
// // // //     viz::Viz3d vis("VO");
// // // //     //2.构造一个坐标系，并显示到窗口中
// // // //     viz::WCoordinateSystem world_coor(1.0),camera_coor(2.0);
// // // //     //相机在世界坐标系下位姿视角
// // // //     Affine3d cam_pose=viz::makeCameraPose((0,-1,-1),(0,0,0),(0,1,0));
// // // //     vis.setViewerPose(cam_pose);
// // // //     vis.showWidget("World",world_coor);
// // // //     vis.showWidget("Camera",camera_coor);
// // // //     //先创建一旋转向量，罗德里格斯公式转为旋转矩阵，
// // // //     Mat rvec = Mat::zeros(1, 3, CV_32F);
// // // //   while(!vis.wasStopped())
// // // //   {
// // // //     rvec.at<float>(0,0) = 0.f;
// // // //     rvec.at<float>(0,1) += CV_PI*0.01f;
// // // //     rvec.at<float>(0,2) = 0.f;
// // // //     Mat rmat;
// // // //     Rodrigues(rvec, rmat);
// // // //     Affine3f pose(rmat, Vec3f(0,0,0));
// // // //     vis.setWidgetPose("Camera", pose);
// // // //     vis.spinOnce(1, true);
// // // //   }
    
// // // //     //3.开启循环暂留
// // // //        vis.spinOnce(1, true);
// // // // }
