#include <opencv2/opencv.hpp>
#include <depth_map_process/test_pic.hpp>
#include <iostream>

using namespace cv;
using namespace std;


int main() {
    cv::Mat X_, Y_;
    cv::Mat standardRBFMatrix;


    meshgrid(-30, 30, X_, Y_);
    standardRBFMatrix = RBF(X_, Y_, 1);
    

    standardRBFMatrix.convertTo(standardRBFMatrix, CV_32F);
    
    fstream file;
    file.open("/home/uav/catkin_ws/src/MPC_aware/depth_map_process/src/data.txt", ios::in);
    if (!file) {
        cout << "File not found" << endl;
    }
    Mat origin = Mat::zeros(360, 640, CV_32FC1);
    for (int i = 0; i < 360; ++i) {
        for (int j = 0; j < 640; ++j) {
            file >> origin.at<float>(i, j);
            // cout << origin.at<float>(i, j) << " ";
        }
    }
    file.close();
    cv::Mat depth_image;
    cv::filter2D(origin, depth_image, CV_32FC1, standardRBFMatrix,cv::Point(-1,-1),(0,0),cv::BORDER_CONSTANT);

    
    // cv::imshow("depth_image", depth_image);
    // // cv::imshow("origin", origin);
    // cv::waitKey(0);
    
    double maxVal = 0;
	double minVal = 0;
	minMaxLoc(depth_image, &minVal, &maxVal);

    // depth_image.forEach<float>([](float &value, const int *position) -> void {
    //     value = round(value);
    // });
    depth_image = (depth_image - minVal) / (maxVal - minVal);

    Mat dst = Mat::zeros(360, 640, CV_8UC1);
    depth_image.convertTo(dst, CV_8U, 255.0);
    // cout << depth_image << endl;
    // adaptiveThreshold(dst, depth_image, 125, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, -2);

    cv::threshold(dst, depth_image, 200, 255, cv::THRESH_BINARY);

    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(depth_image, depth_image, element);
    erode (depth_image, depth_image, element);
    // cout << depth_image << endl;
    // cv::Mat image_re = depth_image.reshape(1);
    // double minValue, maxValue;    // 最大值，最小值
    // cv::Point  minIdx, maxIdx;    // 最小值坐标，最大值坐标     
    // cv::minMaxLoc(image_re, &minValue, &maxValue, &minIdx, &maxIdx);
    // std::cout << "最大值：" << maxValue <<"最小值："<<minValue<<std::endl;
    // std::cout << "最大值位置：" << maxIdx << "最小值位置：" << minIdx;
  
    // Mat depth_image_re = Mat::zeros (360, 640, CV_8UC1);
    // depth_image_re.data = depth_image.data;

    // cout << depth_image_re.type() << endl;

    std::vector<std::vector<Point>> contours;
    findContours(depth_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int max_count = 0;
    double max_area = 0;
    for(int i = 0; i < contours.size(); ++i)
    {
        double area = contourArea(contours[i]);
        if (area > max_area)
        {
            max_count = i;
            max_area = area;
        }
    }

    Moments mu = moments(contours[max_count]);
    cv::Point maxLoc(int(mu.m10/mu.m00), int(mu.m01/mu.m00));
    cout << maxLoc << endl;

    double target_depth = origin.at<float>(maxLoc.y, maxLoc.x);
    cout << target_depth << endl;

    fstream file_("data1.txt",ios::out);
    for (int i = 0; i < 360; ++i) {
        for (int j = 0; j < 640; ++j) {
            file_ << int(depth_image.at<uchar>(i, j)) << " ";
        }
        file_ << "\n";
    }
    file_.close();
}