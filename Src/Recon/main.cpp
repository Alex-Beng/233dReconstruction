// 创建三个线程，同时读取三个摄像头的图像
// 然后三个线程解算pnp，求出当前摄像头的外参
// 求当前摄像头分割图
// 然后将区域点云返回去三个摄像头图像

// ↑↑沙雕做法↑↑
// 
#include <PnpSolver.h>
#include <Segmenter.h>

void CreatPointCloud(std::vector<cv::Vec3f>& cloud, std::vector<cv::Vec3b>& color) {
    for (int x=-200; x<0; x+=1) {
        for (int y=0; y<200; y+=1) {
            for (int z=0; z<100; z+=1) {
                cloud.push_back(
                cv::Vec3f(x, y, z)
                );
                color.push_back(
                    cv::Vec3b(0, 0, 0)
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
    viz::WCloud* cloud_widget = NULL;

    window.showWidget("World",world_coor);
    window.showWidget("plane", plane);

    cv::VideoCapture cp(cam_idx);
    cv::Mat frame;
    PnpSolver pnp_solvers(5, 7, 29, 19.5, 0, "../Calib/InParams/in.param");
    Segmenter segmenters(0, 76, 93, 1);
    std::vector<cv::Mat> Rs;
    std::vector<cv::Mat> ts;
    cv::Mat frame_out;

    cv::namedWindow("set_params", CV_WINDOW_NORMAL);
    cv::createTrackbar("cnl_idx", "set_params", &segmenters.cnl_flag_, 5);
    cv::createTrackbar("thre_min", "set_params", &segmenters.thre_min_, 255);
    cv::createTrackbar("thre_max", "set_params", &segmenters.thre_max_, 255);
    cv::createTrackbar("cw", "set_params", &segmenters.cw_, 1);
    

    while (cp.grab()) {

        // Get Image
        cp.retrieve(frame);
        cv::Mat frame_copy;
        frame.copyTo(frame_copy);

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
        cv::line(frame, reproj_point[0], reproj_point[1], cv::Scalar(0,255,0), 3);
        cv::line(frame, reproj_point[0], reproj_point[2], cv::Scalar(0,0,255), 3);
        cv::line(frame, reproj_point[0], reproj_point[3], cv::Scalar(255,0,0), 3);

        cv::imshow("reproj", frame);


        // Segment Part
        cv::Mat t_binary;
        segmenters.ImageProcesser(frame_copy, t_binary, frame_out);
        cv::imshow("binary", t_binary);
        cv::imshow("segment", frame_out);
        

        // Reconstruct
        // for (int i=0; i<cloud.size(); i++) {
        //     cloud_point_valid[i] = true;
        // }

        // std::vector<cv::Point2f> reproj_point;

        char t_key = cv::waitKey(1);
        if (t_key == 'q') {
            break;
        }
        else if (t_key == 'c') {
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
            // channel 0 是距离， 1 是对应点云中的idx
            cv::Mat camera_dists(frame_copy.rows, frame_copy.cols, CV_64FC2, cv::Scalar(99999999, -1));

            std::vector<cv::Vec3f> object_cloud;
            std::vector<cv::Vec3b> object_color;
            std::vector<cv::Vec2i> object_cloud_reproj;
            for (int i=0; i<cloud.size(); i++) {
                if (cloud_point_valid[i]) {
                    object_cloud.push_back(cloud[i]);
                    object_color.push_back(color[i]);

                    // 计算体素与相机的距离
                    double t_dist = sqrt(
                        (cloud[i][0] - t_t.at<float>(0, 0))*(cloud[i][0] - t_t.at<float>(0, 0)) +
                        (cloud[i][1] - t_t.at<float>(0, 1))*(cloud[i][1] - t_t.at<float>(0, 1)) +
                        (cloud[i][2] - t_t.at<float>(0, 2))*(cloud[i][2] - t_t.at<float>(0, 2))
                    );
                    cv::Point2i t_pnt = reproj_point[i];
                    
                    object_cloud_reproj.push_back(t_pnt);
                    if (t_pnt.x<0 || t_pnt.x>=camera_dists.cols || t_pnt.y<0 || t_pnt.y>=camera_dists.rows) {
                        continue;
                    }
                    if (t_dist < camera_dists.at<cv::Vec2d>(t_pnt)[0]) {
                        // cout<<t_pnt<<endl;
                        camera_dists.at<cv::Vec2d>(t_pnt)[0] = t_dist;
                        camera_dists.at<cv::Vec2d>(t_pnt)[1] = i;
                    }
                }

            }

            // cout<<1<<endl;
            for (int i=0; i<object_cloud.size(); i++) {
                cv::Point2i t_reproj = object_cloud_reproj[i];
                if (t_reproj.x<0 || t_reproj.x>=camera_dists.cols || t_reproj.y<0 || t_reproj.y>=camera_dists.rows) {
                    continue;
                }
                if (camera_dists.at<cv::Vec2d>(t_reproj)[1] != -1) {
                    object_color[i] = frame_copy.at<cv::Vec3b>(t_reproj);
                    color[i] = object_color[i];
                }
            }
            // cout<<2<<endl;
            if (object_cloud.size() == 0) {
                object_cloud.push_back(cloud[0]);
                object_color.push_back(color[0]);
            }
            if (cloud_widget == NULL) {
                cloud_widget = new viz::WCloud(object_cloud, object_color);
            }
            else {
                delete cloud_widget;
                cloud_widget = new viz::WCloud(object_cloud, object_color);
            }
            // cloud_widget = viz::WCloud(object_cloud, object_color);
        }
        if (cloud_widget != NULL)
            window.showWidget("point_cloud", *cloud_widget);
        window.spinOnce(10, false);

    }
}

