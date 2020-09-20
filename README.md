# DIP assignment 4 ---- 3d reconstruction

![demostration](/pic/out-5.gif))

运行环境为 Ubuntu 16.04 + opencv3.4.5 + opencv-contirb3.4.5

工作流程 
1. 相机标定 
   
   标定主要是标内参和畸变参数，外参通过求解PnP获得
2. 实时重建
   
   通过上述的求解PnP实时获得相机外参，通过标好的内参和求解的外参将空间中的点成像到相机平面，通过分割结果去除不是物体的点。
   
   其中，物体颜色是通过每次成像，在计算点云中每个点与相机距离，每张图只更新投影到物体区域中某一点离相机最近的点获得的。

PS: 所用的标定板在pic/cali_（也可自己生成，生成使用opencv-contrib的位于modules/aruco/samples/create_board_charuco例程）