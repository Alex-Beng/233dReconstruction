// 用于可视化相机外参
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

void ReadRt(string file_path,
            std::vector<cv::Mat>& Rs,
            std::vector<cv::Vec3f>& ts) {
    ifstream Rs_in(file_path+"Rs.param");
    ifstream ts_in(file_path+"ts.param");

    string t_line;
    cv::Vec3f t_t;
    while (getline(ts_in, t_line)) {
        
        stringstream t_line_ss;
        t_line_ss << t_line;
        for (int i=0; i<3; i++) {
            t_line_ss >> t_t[i];
        }
        cout<<t_t<<endl;
        ts.push_back(t_t);
    }

    cv::Mat t_R(1, 3, CV_32FC1, cv::Scalar(0));
    while (getline(Rs_in, t_line)) {
        
        stringstream t_line_ss;
        t_line_ss << t_line;
        for (int i=0; i<3; i++) {
            t_line_ss >> t_R.at<float>(0, i);
        }
        cout<<t_R<<endl;
        Rs.push_back(t_R);
    }
}


// 对外参进行可视化
void VisualRt(std::vector<cv::Mat>& Rs,
            std::vector<cv::Vec3f>& ts) {
    viz::Viz3d window("window");
    // 世界坐标系
    viz::WCoordinateSystem world_coor(1.0);
    
    window.showWidget("World",world_coor);
    window.spinOnce(1, false);
    for (int i=0; i<ts.size(); i++) {
        viz::WCoordinateSystem camer_coor(0.5);
        // cv::Mat r_vec = cv::Mat::zeros(1, 3, CV_32F);
        // cv::Mat r_mat;
        // cv::Rodrigues(r_vec, r_mat);
        Affine3f pose(Rs[i], ts[i]);
        
        stringstream t_ss;
        string t_s;
        t_ss << i;
        t_ss >> t_s;

        window.showWidget("Camera"+t_s,camer_coor);
        window.setWidgetPose("Camera"+t_s, pose);
        window.spinOnce(1000, true);
    }

    window.spin();
    

    


    


    // window.showWidget("Coordinate", viz::WCoordinateSystem());
    // //创建平面
    // viz::WPlane plane;
    // //添加平面，并设置一个ID为plane
    // window.showWidget("plane", plane);
    // window.showWidget("Coordinate", viz::WCoordinateSystem());

    // // Rs 已经是旋转矩阵
    // // ts 已经是位姿
    // int t = 0;
    // while (! window.wasStopped()) {
    //     Affine3f pose(Rs[t], ts[t]);
    //     window.setWidgetPose("plane", pose);
    //     window.spin();
    //     t++;
    // }
    
}


int main() {
    std::vector<cv::Mat> Rs;
    std::vector<cv::Vec3f> ts;
    ReadRt("../Calib/ExtParams/", Rs, ts);

    VisualRt(Rs, ts);

    return 0;

    viz::Viz3d window("window");
    window.showWidget("Coordinate", viz::WCoordinateSystem());
    //创建平面
    viz::WPlane plane;
    //添加平面，并设置一个ID为plane
    window.showWidget("plane", plane);

    //创建一个1*3的rotation vector
    Mat rvec = Mat::zeros(1, 3, CV_32F);
    //动画的本质：调整部件位姿并循环显示，控制条件是窗口没被停止，也就是主动按下了q或者e键
    while(!window.wasStopped())
    {
        rvec.at<float>(0, 0) = 0.f;
        rvec.at<float>(0, 1) += CV_PI * 0.01f;
        rvec.at<float>(0, 2) = 0.f;
        Mat rmat;
        //罗德里格斯公式，将罗德里格斯向量转换成旋转矩阵
        Rodrigues(rvec, rmat);
        //构造仿射变换类型的pose，这个类型暂且看做OpenCV中的位姿类型，两个参数，一个旋转，一个平移
        Affine3f pose(rmat, Vec3f(0, 0, 0));
        //这一句就是整个可视化窗口能够动起来的核心语句了，
        //说白了就是利用循环来不断调整上面plane部件的位姿，达到动画的效果
        //另外这里就利用到了平面的ID，来表征调整的是平面的位姿
        window.setWidgetPose("plane", pose);
        //控制单帧暂留时间，调整time参数的效果就是平面转的快慢，本质上就是每一个位姿的停留显示时间。
        window.spinOnce(1, false);
    }

    return 0;
}