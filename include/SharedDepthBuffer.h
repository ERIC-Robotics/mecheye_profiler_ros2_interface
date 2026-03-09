#pragma once
#include <deque>
#include <mutex>
#include <condition_variable>
#include <opencv2/core.hpp>

struct SharedDepthFrame {
    double  timestamp_sec = 0.0;
    cv::Mat image;                // CV_32FC1 depth map in mm
};

struct SharedDepthBuffer {
    std::mutex              mtx;
    std::condition_variable cv;
    std::deque<SharedDepthFrame> frames;  // front = oldest, back = newest
};
