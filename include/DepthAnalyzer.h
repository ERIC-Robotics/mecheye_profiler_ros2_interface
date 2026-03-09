#pragma once
#include <memory>
#include <thread>
#include <atomic>
#include <limits>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/opencv.hpp>
#include <SharedDepthBuffer.h>

namespace depth_analyzer {

class DepthAnalyzerNode : public rclcpp::Node
{
public:
    explicit DepthAnalyzerNode(std::shared_ptr<SharedDepthBuffer> buffer,
                               const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~DepthAnalyzerNode();

private:
    void processLoop();
    void processFrame(double timestamp_sec, const cv::Mat& depth);
    cv::Mat removeSurface(const cv::Mat& depth);
    std::vector<cv::Rect> detectObjects(const cv::Mat& normalized);

    std::shared_ptr<SharedDepthBuffer> buffer_;
    std::thread process_thread_;
    std::atomic<bool> running_{true};

    // Detection Parameters
    double z_threshold_ = 36.0;   // Legacy threshold for backward compatibility
    double min_height_ = 2.0;     // MIN_HEIGHT
    int min_area_ = 6;            // MIN_AREA
    int min_width_ = 3;           // MIN_WIDTH
    int gaussian_blur_size_ = 31; // GaussianBlur kernel size (31,31)

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr z_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr object_pub_;
};

} // namespace depth_analyzer
