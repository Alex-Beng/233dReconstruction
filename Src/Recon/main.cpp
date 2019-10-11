// 创建三个线程，同时读取三个摄像头的图像
// 然后三个线程解算pnp，求出当前摄像头的外参
// 求当前摄像头分割图
// 然后将区域点云返回去三个摄像头图像

// ↑↑沙雕做法↑↑
// 
#include <PnpSolver.h>
#include <Segmenter.h>

void CreatPointCloud(std::vector<cv::Vec3f>& cloud, std::vector<cv::Vec3b>& color) {
    for (int x=-100; x<0; x+=1) {
        for (int y=0; y<100; y+=1) {
            for (int z=0; z<100; z+=1) {
                cloud.push_back(
                cv::Vec3f(x, y, z)
                );
                color.push_back(
                    cv::Vec3b(0, 255, 0)
                );
            }
        }
    }
}


const char* keys  =
        "{ci       | 0     | camera index }";

int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    int cam_idx = parser.get<int>("ci");

    // 重建时候用到的点云
    std::vector<cv::Vec3f> cloud;
    std::vector<cv::Vec3b> color;
    CreatPointCloud(cloud, color);
    std::vector<bool> cloud_point_valid;
    for (int i=0; i<cloud.size(); i++) {
        cloud_point_valid.push_back(true);
    }

    viz::Viz3d window("window");
    viz::WCoordinateSystem world_coor(40.0);
    viz::WPlane plane(cv::Size(200, 200));

    window.showWidget("World",world_coor);
    window.showWidget("plane", plane);

    cv::VideoCapture cp(cam_idx);
    cv::Mat frame;
    PnpSolver pnp_solvers(5, 7, 29, 19.5, 0, "../Calib/InParams/in.param");
    Segmenter segmenters(5, 118, 255, 1);
    std::vector<cv::Mat> Rs;
    std::vector<cv::Mat> ts;
    cv::Mat frame_out;

    while (cp.grab()) {

        // Get Image
        cp.retrieve(frame);
        
        // Solve Pnp Part
        cv::Mat t_R;
        cv::Mat t_t;
        bool t_pnp_valid = pnp_solvers.Solve(frame, t_R, t_t);
        if (!t_pnp_valid) {
            continue;
        }
        else {
            Rs.push_back(t_R);
            ts.push_back(t_t);
        }
        
        // 重投影康康
        std::vector<cv::Point3f> ori_pnt;
        std::vector<cv::Point2f> reproj_point;
        ori_pnt.push_back(cv::Point3f(0, 0, 0));
        ori_pnt.push_back(cv::Point3f(0, 100, 0));
        ori_pnt.push_back(cv::Point3f(100, 0, 0));
        ori_pnt.push_back(cv::Point3f(0, 0, 100));
        cv::projectPoints(ori_pnt, t_R, t_t, pnp_solvers.camera_matrix_, pnp_solvers.dist_coeffs_, reproj_point);

        cv::circle(frame, reproj_point[0], 5, cv::Scalar(0,255,255), 3);
        cv::circle(frame, reproj_point[1], 5, cv::Scalar(0,0,0), 3);
        cv::circle(frame, reproj_point[2], 5, cv::Scalar(0,255,0), 3);
        cv::circle(frame, reproj_point[3], 5, cv::Scalar(255,255,0), 3);
        cv::line(frame, reproj_point[0], reproj_point[1], cv::Scalar(0,0,255), 3);
        cv::line(frame, reproj_point[0], reproj_point[2], cv::Scalar(0,255,0), 3);
        cv::line(frame, reproj_point[0], reproj_point[3], cv::Scalar(255,0,0), 3);

        cv::imshow("reproj", frame);


        // Segment Part
        cv::Mat t_binary;
        segmenters.ImageProcesser(frame, t_binary, frame_out);
        cv::imshow("segment", frame_out);
        

        // Reconstruct
        // for (int i=0; i<cloud.size(); i++) {
        //     cloud_point_valid[i] = true;
        // }

        // std::vector<cv::Point2f> reproj_point;
        cv::projectPoints(cloud, t_R, t_t, pnp_solvers.camera_matrix_, pnp_solvers.dist_coeffs_, reproj_point);


        for (int j=0; j<reproj_point.size(); j++) {

            int t_y = (int)reproj_point[j].y;
            int t_x = (int)reproj_point[j].x;
            if (t_x<0 || t_x>=frame.cols || t_y<0 || t_y>=frame.rows) {
                continue;
            }
            if (frame_out.at<uchar>(t_y, t_x) == 255) {
                cloud_point_valid[j] = false; 
            }
        }

        // viz part
        std::vector<cv::Vec3f> object_cloud;
        std::vector<cv::Vec3b> object_color;
        for (int i=0; i<cloud.size(); i++) {
            if (cloud_point_valid[i]) {
                object_cloud.push_back(cloud[i]);
                object_color.push_back(cv::Vec3b(0,255,0));
            }

        }
        
        if (object_cloud.size() == 0) {
            object_cloud.push_back(cloud[0]);
            object_color.push_back(cv::Vec3b(0,255,0));
        }
        viz::WCloud cloud_widget(object_cloud, object_color);
        window.showWidget("point_cloud", cloud_widget);
        window.spinOnce(10, false);

        char t_key = cv::waitKey(1);
        if (t_key == 'q') {
            break;
        }
    }
}

