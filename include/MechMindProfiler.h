#pragma once
#include <profiler/Profiler.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <string>
#include <sys/stat.h>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/int32.hpp>
#include <SharedDepthBuffer.h>

struct BufferedFrame {
    double  timestamp_sec = 0.0;   // ROS time as seconds (avoids rclcpp::Time default-ctor overhead)
    cv::Mat image;
};
    
class MechMindProfiler
{
public:
    explicit MechMindProfiler(std::shared_ptr<SharedDepthBuffer> shared_buffer = nullptr);
    void handleCallbackBatch(const mmind::eye::ProfileBatch& batch);
    rclcpp::Node::SharedPtr node;

private:
    mmind::eye::Profiler profiler;

    std::string profiler_ip;
    bool save_file = false;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;

    void publishDepthMap(mmind::eye::ProfileBatch::DepthMap&& depthMap);

    // ── Shared buffer (consumed by DepthAnalyzer) ───────────────────────────
    std::shared_ptr<SharedDepthBuffer> shared_buffer_;

    std::vector<BufferedFrame> depth_frames_;
    size_t  depth_write_index_ = 0;
    size_t  depth_max_size_    = 1000;
    bool    depth_buf_full_    = false;
    std::string depth_output_dir_ = "mechmind_depth_buffer_output_5";
};
