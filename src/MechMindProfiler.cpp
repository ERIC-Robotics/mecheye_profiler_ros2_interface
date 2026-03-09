#include <sstream>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <profiler/api_util.h>
#include <MechMindProfiler.h>

namespace {

void callbackFunc(const mmind::eye::ProfileBatch& batch, void* pUser)
{
    auto* mechMindProfiler = static_cast<MechMindProfiler*>(pUser);
    if (mechMindProfiler)
        mechMindProfiler->handleCallbackBatch(batch);
}
} // namespace

MechMindProfiler::MechMindProfiler(std::shared_ptr<SharedDepthBuffer> shared_buffer)
    : shared_buffer_(shared_buffer)
{
    node = rclcpp::Node::make_shared("mechmind_profiler_publisher_service");

    node->declare_parameter<std::string>("profiler_ip", "192.168.0.37");
    node->declare_parameter<bool>("save_file", false);
    
    node->get_parameter("profiler_ip", profiler_ip);
    node->get_parameter("save_file", save_file);

    pub_depth = node->create_publisher<sensor_msgs::msg::Image>("/mechmind_profiler/depth_map_1", 1);

    // Depth map ring buffer
    depth_frames_.resize(depth_max_size_);
    RCLCPP_INFO(node->get_logger(),
                "[Profiler] Depth ring buffer: %zu frames.",
                depth_max_size_);

    mmind::eye::ErrorStatus status;
    mmind::eye::ProfilerInfo info;
    info.firmwareVersion = mmind::eye::Version("2.5.4");
    info.ipAddress = profiler_ip;
    info.port = 5577;
    status = profiler.connect(info);
    if (!status.isOK())
    {
        throw status;
    }
    std::cout << "Connected to the profiler successfully." << std::endl;

    mmind::eye::ProfilerInfo profilerInfo;
    showError(profiler.getProfilerInfo(profilerInfo));
    printProfilerInfo(profilerInfo);

    status = profiler.registerAcquisitionCallback(callbackFunc, this);
    if (!status.isOK()) {
        throw status;
    }

    // Auto-start acquisition and trigger software as requested
    RCLCPP_INFO(node->get_logger(), "[Profiler] Auto-starting acquisition...");
    status = profiler.startAcquisition();
    if (!status.isOK()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start acquisition: %s", status.errorDescription.c_str());
        throw status;
    }

    RCLCPP_INFO(node->get_logger(), "[Profiler] Triggering software...");
    status = profiler.triggerSoftware();
    if (!status.isOK()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to trigger software: %s", status.errorDescription.c_str());
        // We might not want to throw here if startAcquisition succeeded, but usually it's a fatal error
        throw status;
    }
}

void MechMindProfiler::handleCallbackBatch(const mmind::eye::ProfileBatch& batch)
{
    auto batchBackup = batch;
    if (!batchBackup.getErrorStatus().isOK()) {
        showError(batch.getErrorStatus());
        return;
    }
    
    // RCLCPP_INFO(node->get_logger(), "[Profiler] Received batch: %d valid profiles.", batchBackup.validHeight());

    if (batchBackup.checkFlag(mmind::eye::ProfileBatch::BatchFlag::Incomplete))
        std::cout << "Part of the batch's data is lost, the number of valid profiles is: "
                  << batchBackup.validHeight() << "." << std::endl;
    publishDepthMap(batchBackup.getDepthMap());
}

void MechMindProfiler::publishDepthMap(mmind::eye::ProfileBatch::DepthMap&& depthMap)
{
    cv::Mat depth = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());

    // Depth map ring buffer 
    depth_frames_[depth_write_index_] = {node->now().seconds(), depth.clone()};
    depth_write_index_ = (depth_write_index_ + 1) % depth_max_size_;

    if (depth_write_index_ == 0)
    {
        depth_buf_full_ = true;
        RCLCPP_INFO(node->get_logger(),
                    "[Profiler] Depth ring buffer full (%zu frames).",
                    depth_max_size_);
    }

    // Push to shared buffer for DepthAnalyzer
    if (shared_buffer_)
    {
        {
            std::lock_guard<std::mutex> lock(shared_buffer_->mtx);
            shared_buffer_->frames.push_back({node->now().seconds(), depth.clone()});
        }
        shared_buffer_->cv.notify_one();  // wake up DepthAnalyzer
    }

    cv_bridge::CvImage cv_depth;
    cv_depth.image = depth;
    cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sensor_msgs::msg::Image ros_depth;
    cv_depth.toImageMsg(ros_depth);
    ros_depth.header.frame_id = "mechmind_profiler/depth_map";
    ros_depth.header.stamp = node->now();
    pub_depth->publish(ros_depth);
}
