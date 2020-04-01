#ifndef IO_CAM_PARAM_H
#define IO_CAM_PARAM_H

#include <bits/stdc++.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
using namespace std;
using namespace cv;


void ReadInParams(string& file_name, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
    FileStorage fs(file_name, FileStorage::READ);
    if (!fs.isOpened())
        return;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
}

void WriteOutParams(string& file_name, cv::Mat R_mat, cv::Mat t_mat) {
    FileStorage fs(file_name, FileStorage::WRITE);
    if (!fs.isOpened())
        return;
    fs << "R_mat" << R_mat;
    fs << "t_mat" << t_mat;
}

void ReadOutParams(string& file_name, cv::Mat& R_mat, cv::Mat& t_mat) {
    FileStorage fs(file_name, FileStorage::READ);
    if (!fs.isOpened())
        return;
    fs["R_mat"] >> R_mat;
    fs["t_mat"] >> t_mat;
}


#endif