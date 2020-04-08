/**
原理如下：
世界坐标系选相机重力方向反方向为z方向，x y只要求与地面平行
相机坐标外参提前标好
水平的点[X, Y, 0, 1]^T到像素坐标[u, v, 1]^T成像模型有

s*[u, v, 1]^T = M*[X, Y, 0, 1]^T
其中M为3x4大小的矩阵, 只有X Y s三个未知数

简单推导得
+-          -+   +- -+   +-    -+
| m11 m12 -u |   | X |   | -m14 |
| m21 m22 -v | * | Y | = | -m24 |
| m31 m32 -1 |   | s |   | -m34 |
+-          -+   +- -+   +-    -+
然后瞎几把解就完事了

So
1. 读内参外参畸变参
2. 整出M
3. 用鼠标标出像素坐标(u, v, 1)
4. 整出A B， 解Ax = B
*/

#include "io_cam_param.h"

#include <bits/stdc++.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
using namespace std;
using namespace cv;

cv::Point2f pix_coor;
cv::Mat camera_matrix;  // 内参
cv::Mat dist_coeffs;    // 畸变参
cv::Mat R_matrix;       // Pw -> Pc 旋转矩阵
cv::Mat t_matrix;       // ｔ
cv::Mat Rt_4x4(4, 4, CV_32F, cv::Scalar(0));    // 写成齐次形式的
cv::Mat A, B, x;        // 线性方程的 A B x

const char* keys  =
        "{ci        | 0     | Camera id if input doesnt come from video (-v) }"
        "{in_file   |       | File in which stores the in-param }"
        "{out_file  |       | File in which stores the out-param }";


void MouseHandler(int event, int x, int y, int flags, void * param);

int main(int argc, char const *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    int cam_id = parser.get<int>("ci");
    string in_param_file = parser.get<string>("in_file");
    string out_param_file = parser.get<string>("out_file");

    
    ReadInParams(in_param_file, camera_matrix, dist_coeffs);
    ReadOutParams(out_param_file, R_matrix, t_matrix);

    // 统一转成32float
    camera_matrix.convertTo(camera_matrix, CV_32F);
    dist_coeffs.convertTo(dist_coeffs, CV_32F);
    R_matrix.convertTo(R_matrix, CV_32F);
    t_matrix.convertTo(t_matrix, CV_32F);
    
    cv::Mat Rt_4x4(4, 4, CV_32F, cv::Scalar(0));

    for (int i=0; i<3; i++) {
        Rt_4x4.at<float>(i, 0) = R_matrix.at<float>(i, 0);
        Rt_4x4.at<float>(i, 1) = R_matrix.at<float>(i, 1);
        Rt_4x4.at<float>(i, 2) = R_matrix.at<float>(i, 2);
        Rt_4x4.at<float>(i, 3) = t_matrix.at<float>(i, 0);
    }
    Rt_4x4.at<float>(3, 3) = 1.0;

    cv::Mat M_matrix(3, 4, CV_32F, cv::Scalar(0));
    for (int i=0; i<3; i++) {
        M_matrix.at<float>(i, i) = 1;
    }

    M_matrix = camera_matrix*M_matrix*Rt_4x4;

    // M 铁定是没毛病的(包括类型)
    cout<<M_matrix.type()<<' '<<CV_32F<<endl;


    
    // get A and B
    A = cv::Mat(3, 3, CV_32F, cv::Scalar(-1));
    B = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
    for (int i=0; i<3; i++) {
        A.at<float>(i, 0) = M_matrix.at<float>(i, 0);
        A.at<float>(i, 1) = M_matrix.at<float>(i, 1);

        B.at<float>(i, 0) = -M_matrix.at<float>(i, 3);
    }

    // cout<<Rt_4x4<<endl<<R_matrix<<endl<<t_matrix<<endl;
    cout<<M_matrix<<endl;
    cout<<A<<endl<<B<<endl;

    VideoCapture cp;
    cv::Mat frame;
    cp.open(cam_id);

    cv::namedWindow("ya");
    cv::setMouseCallback("ya", MouseHandler, (void*)&frame);


    while (cp.grab()) {
        
        cp.retrieve(frame);

        cv::imshow("ya", frame);
        char t_key = cv::waitKey(1);
        if (t_key == 'q') {
            break;
        }
    }
    return 0;
}

void MouseHandler(int event, int x, int y, int flags, void * param) {
    cv::Mat& image = *(Mat*)param;
    cv::Mat X_solved;
    bool valid;
    char t_str[256];
    switch (event) {
    case EVENT_LBUTTONDOWN:// 左键点击
        A.at<float>(0, 2) = -x;
        A.at<float>(1, 2) = -y;

        valid = cv::solve(A, B, X_solved);
        
        if (valid) {
            sprintf(t_str, "%f %f %lf", 
                X_solved.at<float>(0, 0), X_solved.at<float>(1, 0),
                sqrt(
                    X_solved.at<float>(0, 0)*X_solved.at<float>(0, 0)
                   +X_solved.at<float>(1, 0)*X_solved.at<float>(1, 0)
                ));
        }
        cout<<X_solved<<endl;
        cv::putText(image, t_str, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,0,255));
        cv::circle(image, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), 5);
        cv::imshow("ya", image);
        break;
    default:
        break;
    }
}