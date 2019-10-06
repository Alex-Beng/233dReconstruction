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
    while (true) {
        cp >> frame;
        if (frame.empty()) {
            cout<<"empty frame !"<<endl;
            cp.open(open_cam_idx);
            continue;
        }
        cv::imshow("233", frame);
        char key = cv::waitKey(1);
        if (key == 'q') {
            break;
        }
        else if (key == 's') {
            save_path = GetNextPath();
            cv::imwrite(save_path, frame);
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
