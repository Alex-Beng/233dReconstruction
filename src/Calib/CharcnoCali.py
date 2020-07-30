# -*- coding: utf-8 -*-
###相机标定
import cv2
import numpy as np
import os
dirpath=os.getcwd()
print("当前路径是%s"%dirpath)
#创建一个标准板子
num = 9#9*9的板子
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(num, num, .025, .0125, dictionary)#0.025单位是米
img = board.draw((200 * num, 200 * num))
cv2.imwrite(dirpath+"\\"+"{}.png".format(num), img)
#打印板子，如贴在墙上，用相机不同角度拍摄若干张照片
#标定
demo_path=r'' #文件夹
if len(os.listdir(demo_path))>40:
    assert("相机拍摄照片少于40张")
allCorners = []
allIds = []
os.chdir(demo_path) #改变路径，变换到文件夹中
for i in range(len(os.listdir(demo_path))): #这里也可以用webcamera 测试，把标准板子在webcamera 前移动
    im=cv2.imread("%g.jpg"%i,0)
    corners, ids, rejected = cv2.aruco.detectMarkers(im, dictionary)
    if corners == None or len(corners) == 0:
        continue
    ret, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners, ids, im, board)#其中的参数依赖于detectMarkers检测的初始值
    if corners is not  None  and charucoIds is not None:
        allCorners.append(charucoCorners)
        allIds.append(charucoIds)
    cv2.aruco.drawDetectedMarkers(im,corners,ids)
    cv2.imshow("marsk",im)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
w,h=im.shape[1],im.shape[0]
ret, K, dist_coef, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(allCorners, allIds, board,(w,h),None,None,flag=cv2.CALIB_USE_INTRINSIC_GUESS)
#save results
cali_results=np.savez(demo_path+"\\"+"camera.npz",k= K,d=dist_coef)#cali_results['k']和cali_results['d']可以可视化结果
