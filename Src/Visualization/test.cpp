#include <iostream>
#include <opencv2/viz.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
 
using namespace std;
using namespace cv;
int main()
{
    //1.创建可视化窗口
    viz::Viz3d vis("VO");
    //2.构造一个坐标系，并显示到窗口中
    viz::WCoordinateSystem world_coor(1.0),camera_coor(2.0);
    //相机在世界坐标系下位姿视角
    Affine3d cam_pose=viz::makeCameraPose((0,-1,-1),(0,0,0),(0,1,0));
    vis.setViewerPose(cam_pose);
    vis.showWidget("World",world_coor);
    vis.showWidget("Camera",camera_coor);
    //先创建一旋转向量，罗德里格斯公式转为旋转矩阵，
    Mat rvec = Mat::zeros(1, 3, CV_32F);
  while(!vis.wasStopped())
  {
    rvec.at<float>(0,0) = 0.f;
    rvec.at<float>(0,1) += CV_PI*0.01f;
    rvec.at<float>(0,2) = 0.f;
    Mat rmat;
    Rodrigues(rvec, rmat);
    Affine3f pose(rmat, Vec3f(0,0,0));
    vis.setWidgetPose("Camera", pose);
    vis.spinOnce(1, true);
  }
    
    //3.开启循环暂留
       vis.spinOnce(1, true);
}
