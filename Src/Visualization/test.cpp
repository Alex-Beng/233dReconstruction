#include <opencv2/opencv.hpp>
//#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <iostream>
 
using namespace std;
using namespace cv;
 
void createCharucoBoard(cv::Mat &boardImage)
{
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04, 0.02, dictionary);
	board->draw(cv::Size(3600, 3600), boardImage);
}
 
int main()
{
 
	cv::Mat charuco_boardImage;
	createCharucoBoard(charuco_boardImage);
  cv::imshow("ya", charuco_boardImage);
  cv::waitKey();
	return 0;
}

// #include <opencv2/highgui.hpp>
// #include <opencv2/aruco/charuco.hpp>

// using namespace cv;

// namespace {
// const char* about = "Create a ChArUco board image";
// const char* keys  =
//         "{@outfile |<none> | Output image }"
//         "{w        |       | Number of squares in X direction }"
//         "{h        |       | Number of squares in Y direction }"
//         "{sl       |       | Square side length (in pixels) }"
//         "{ml       |       | Marker side length (in pixels) }"
//         "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
//         "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
//         "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
//         "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
//         "{m        |       | Margins size (in pixels). Default is (squareLength-markerLength) }"
//         "{bb       | 1     | Number of bits in marker borders }"
//         "{si       | false | show generated image }";
// }

// int main(int argc, char *argv[]) {
//     CommandLineParser parser(argc, argv, keys);
//     parser.about(about);

//     if(argc < 7) {
//         parser.printMessage();
//         return 0;
//     }

//     int squaresX = parser.get<int>("w");
//     int squaresY = parser.get<int>("h");
//     int squareLength = parser.get<int>("sl");
//     int markerLength = parser.get<int>("ml");
//     int dictionaryId = parser.get<int>("d");
//     int margins = squareLength - markerLength;
//     if(parser.has("m")) {
//         margins = parser.get<int>("m");
//     }

//     int borderBits = parser.get<int>("bb");
//     bool showImage = parser.get<bool>("si");

//     String out = parser.get<String>(0);

//     if(!parser.check()) {
//         parser.printErrors();
//         return 0;
//     }

//     Ptr<aruco::Dictionary> dictionary =
//         aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

//     Size imageSize;
//     imageSize.width = squaresX * squareLength + 2 * margins;
//     imageSize.height = squaresY * squareLength + 2 * margins;

//     Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(squaresX, squaresY, (float)squareLength,
//                                                             (float)markerLength, dictionary);

//     // show created board
//     Mat boardImage;
//     board->draw(imageSize, boardImage, margins, borderBits);

//     if(showImage) {
//         imshow("board", boardImage);
//         waitKey(0);
//     }

//     imwrite(out, boardImage);

//     return 0;
// }



// // #include <iostream>
// // #include <opencv2/viz.hpp>
// // #include <opencv2/highgui.hpp>
// // #include <opencv2/calib3d.hpp>
 
// // using namespace std;
// // using namespace cv;
// // int main()
// // {
// //     //1.创建可视化窗口
// //     viz::Viz3d vis("VO");
// //     //2.构造一个坐标系，并显示到窗口中
// //     viz::WCoordinateSystem world_coor(1.0),camera_coor(2.0);
// //     //相机在世界坐标系下位姿视角
// //     Affine3d cam_pose=viz::makeCameraPose((0,-1,-1),(0,0,0),(0,1,0));
// //     vis.setViewerPose(cam_pose);
// //     vis.showWidget("World",world_coor);
// //     vis.showWidget("Camera",camera_coor);
// //     //先创建一旋转向量，罗德里格斯公式转为旋转矩阵，
// //     Mat rvec = Mat::zeros(1, 3, CV_32F);
// //   while(!vis.wasStopped())
// //   {
// //     rvec.at<float>(0,0) = 0.f;
// //     rvec.at<float>(0,1) += CV_PI*0.01f;
// //     rvec.at<float>(0,2) = 0.f;
// //     Mat rmat;
// //     Rodrigues(rvec, rmat);
// //     Affine3f pose(rmat, Vec3f(0,0,0));
// //     vis.setWidgetPose("Camera", pose);
// //     vis.spinOnce(1, true);
// //   }
    
// //     //3.开启循环暂留
// //        vis.spinOnce(1, true);
// // }
