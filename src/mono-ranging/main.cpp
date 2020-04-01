/**
原理如下：
世界坐标系选相机重力方向反方向为y方向，z x只要求与地面平行
相机坐标外参提前标好
水平的点[X, 0, Z, 1]^T到像素坐标[u, v, 1]^T成像模型有

s*[u, v, 1]^T = M*[X, 0, Z, 1]^T
其中M为3x4大小的矩阵, 只有X Z s三个未知数

简单推导得
+-          -+   +- -+   +-    -+
| m11 m13 -u |   | X |   | -m14 |
| m21 m23 -v | * | Z | = | -m24 |
| m31 m33 -1 |   | s |   | -m34 |
+-          -+   +- -+   +-    -+
然后瞎几把解就完事了

So
1. 读内参外参畸变参
2. 整出M
3. 用鼠标标出像素坐标(u, v, 1)
4. 解方程运算一下
*/

#include "io_cam_param.h"

#include <bits/stdc++.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
using namespace std;
using namespace cv;

cv::Point2f pix_coor;
const char* keys  =
        "{ci        | 0     | Camera id if input doesnt come from video (-v) }"
        "{in_file   |       | File in which stores the in-param }"
        "{out_file  |       | File in which stores the out-param }";


int main(int argc, char const *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    int cam_id = parser.get<int>("ci");
    string in_param_file = parser.get<string>("in_file");
    string out_param_file = parser.get<string>("out_file");

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    ReadInParams(in_param_file, camera_matrix, dist_coeffs);

    return 0;
}