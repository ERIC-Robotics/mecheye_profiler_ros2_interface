#include <DepthAnalyzer.h>

namespace depth_analyzer {

DepthAnalyzerNode::DepthAnalyzerNode(std::shared_ptr<SharedDepthBuffer> buffer,
                                     const rclcpp::NodeOptions& options)
    : Node("depth_analyzer", options), buffer_(buffer)
{
    z_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mechmind_profiler/max_z", 10);
    object_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mechmind_profiler/object_detected", 10);

    RCLCPP_INFO(this->get_logger(),
            "[DepthAnalyzer] Started.");

    // Spawn background thread — waits for frames and processes them one by one
    process_thread_ = std::thread(&DepthAnalyzerNode::processLoop, this);
}

DepthAnalyzerNode::~DepthAnalyzerNode()
{
    running_ = false;
    buffer_->cv.notify_all();  // wake up the thread so it can exit
    if (process_thread_.joinable())
        process_thread_.join();
}

void DepthAnalyzerNode::processLoop()
{
    while (running_) {
        SharedDepthFrame frame;

        {
            std::unique_lock<std::mutex> lock(buffer_->mtx);

            // Sleep here with zero CPU until a frame is available or node shuts down
            buffer_->cv.wait(lock, [&]{
                return !buffer_->frames.empty() || !running_;
            });

            if (!running_) break;

            // Take the oldest frame from the front and remove it
            frame = std::move(buffer_->frames.front());
            buffer_->frames.pop_front();
        }
        // Lock released — process without blocking the producer

        processFrame(frame.timestamp_sec, frame.image);
    }
}

void DepthAnalyzerNode::processFrame(double timestamp_sec, const cv::Mat& depth)
{
    float max_z     = std::numeric_limits<float>::lowest();
    bool valid_found = false;

    for (int i = 0; i < depth.rows; ++i)
    {
        for (int j = 0; j < depth.cols; ++j)
        {
            float value = depth.at<float>(i, j);
            if (value != 0.0f && !std::isnan(value))
            {
                if (!valid_found || value > max_z)
                {
                    max_z       = value;
                    valid_found = true;
                }
            }
        }
    }

    if (valid_found)
    {
        RCLCPP_INFO(this->get_logger(),
                "[DepthAnalyzer] ts=%.3f  Max Z: %.2f mm", timestamp_sec, max_z);

        std_msgs::msg::Int32 max_z_msg;
        max_z_msg.data = static_cast<int>(max_z);
        z_pub_->publish(max_z_msg);

        std_msgs::msg::Int32 object_msg;
        if (max_z > z_threshold_)
        {
            RCLCPP_INFO(this->get_logger(),
                    "[DepthAnalyzer] Max Z (%.2f) > threshold (%.1f) → object detected",
                    max_z, z_threshold_);
            object_msg.data = 1;
        }
        else
        {
            object_msg.data = 0;
        }
        object_pub_->publish(object_msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
                "[DepthAnalyzer] ts=%.3f  No valid depth data (all 0 or NaN).", timestamp_sec);
    }
}

} // namespace depth_analyzer
