#include "features.h"
#include <thread>
#include <chrono>

#define MIN_NUM_FEAT 2000
enum  {LOCATING, INITING};
int status = INITING; 

// 初始化和跟踪
int Init();
int Loca();

// 最终位姿 R t
cv::Mat R, t;

// 初始化时候的尺度
double scale = 1.00;

// cam
cv::VideoCapture cp(0);
double focal = 819.216; // fx = fy
cv::Point2d principal_point(291.973, 252.379); // 主点
cv::Mat prev_frame;
std::vector<cv::Point2f> prev_features;

int main(int argc, char const *argv[]) {
    while (1) {
        if (status == INITING) {
            status = Init();
        }
        else if (status == LOCATING) {
            status = Loca();
        }
    }
    return 0;
}

int Init() {
    try {
        cout<<"try initing"<<endl;       
        cv::Mat frame_1, frame_2;
        if (!cp.grab()) {
            exit(-1);
        }
        cp >> frame_1;
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(1 * 1e6)));
        cp >> frame_2;

        cv::cvtColor(frame_1, frame_1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(frame_2, frame_2, cv::COLOR_BGR2GRAY);
        
        std::vector<cv::Point2f> point_1, point_2;
        featureDetection(frame_1, point_1);
        std::vector<uchar> status;
        featureTracking(frame_1, frame_2, point_1, point_2, status);

        cv::Mat E, mask;
        E = cv::findEssentialMat(point_2, point_1, focal, principal_point, cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, point_2, point_1, R, t, focal, principal_point, mask);
        
        prev_frame = frame_2;
        prev_features = point_2;
        cout<<"finish"<<endl;
        return LOCATING;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        return INITING;
    }
    
}

int Loca() {
    try {
        cout<<"loca begin"<<endl;
        if (!cp.grab()) {
            exit(-1);
        }
        cv::Mat curr_frame;
        std::vector<cv::Point2f> curr_features;
        cp >> curr_frame;
        cv::cvtColor(curr_frame, curr_frame, cv::COLOR_BGR2GRAY);
        std::vector<uchar> status;
        featureTracking(prev_frame, curr_frame, prev_features, curr_features, status);

        cv::Mat E, mask;
        E = findEssentialMat(curr_features, prev_features, focal, principal_point, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, curr_features, prev_features, R, t, focal, principal_point, mask);

        scale = t.at<double>(2);

        if ((fabs(scale)>0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
            // t = t +scale;
            // t_f = t_f + scale*(R_f*t);
            // R_f = R*R_f;
        }


        return LOCATING;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        return INITING;
    }
    
}