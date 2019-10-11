// 创建三个线程，同时读取三个摄像头的图像
// 然后三个线程解算pnp，求出当前摄像头的外参
// 求当前摄像头分割图
// 然后将区域点云返回去三个摄像头图像

#include <Segmenter.h>



const char* keys  =
        "{cn       | 1     | camera num }";

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    int cam_num = parser.get<int>("cn");

    cv::VideoCapture cp[cam_num];
    cv::Mat frames[cam_num];

    for (int i=0; i<cam_num; i++) {
        cp[i].open(i+1);
    }

    while (1) {
        bool valid = true;
        for (int i=0; i<cam_num; i++) {
            if (cp[i].grab()) {
                ;
            }
            else {
                valid = false;
            }
        }
        if (!valid) {
            break;
        }

        // Get Image
        for (int i=0; i<cam_num; i++) {
            cp[i].retrieve(frames[i]);
            string t_s = "frame";
            stringstream t_ss;
            t_ss << t_s;
            t_ss << i;
            t_ss >> t_s;
            cv::imshow(t_s, frames[i]);
        }

        // Solve Pnp Part


        char t_key = cv::waitKey(1);
        if (t_key == 'q') {
            break;
        }
    }
}
