#pragma once
#ifndef TEST_PIC_HPP
#define TEST_PIC_HPP
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

Mat RBF(const Mat &X, const Mat &Y, double sigma)
{
    Mat rbfMatrix;
    // cout << "what can i say" << endl;
    exp(-(X.mul(X) + Y.mul(Y)) / (2 * sigma * sigma), rbfMatrix);
    return rbfMatrix;
}

void meshgrid(int start, int end, Mat &X, Mat &Y)
{
    vector<double> range;
    for (int i = start; i < end; ++i)
    {
        range.push_back(i);
    }

    repeat(Mat(range).t(), range.size(), 1, X);
    repeat(Mat(range), 1, range.size(), Y);
}


#endif