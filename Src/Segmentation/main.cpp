// 用于分割出物体
// 然后返回图像的二值图
#include "Segmenter.h"

cv::VideoCapture cp;
Segmenter test_vision(H, 0, 255, 1);

const char* keys  =
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }";

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    int cam_id = parser.get<int>("ci");
    cp.open(cam_id);

    test_vision.cnl_flag_ = 4;
    test_vision.thre_min_ = 0;

    cv::namedWindow("set_params", CV_WINDOW_NORMAL);
    cv::createTrackbar("cnl_idx", "set_params", &test_vision.cnl_flag_, 5);
    cv::createTrackbar("thre_min", "set_params", &test_vision.thre_min_, 255);
    cv::createTrackbar("thre_max", "set_params", &test_vision.thre_max_, 255);
    cv::createTrackbar("cw", "set_params", &test_vision.cw_, 1);

    while (cp.grab()) {
        cv::Mat frame, binary_frame, frame_out;
        cp.retrieve(frame);

        test_vision.ImageProcesser(frame, binary_frame, frame_out);

        cv::imshow("frame", frame);
        cv::imshow("binary", binary_frame);
        cv::imshow("result", frame_out);
        char key = cv::waitKey(1);
        if (key == 'q') {
            break;
        }
    }
    
    return 0;
}