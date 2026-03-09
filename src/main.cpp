#include <csignal>
#include <MechMindProfiler.h>
#include <DepthAnalyzer.h>

void signalHandler(int signum) { rclcpp::shutdown(); }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Create the shared buffer — passed to both nodes
    auto shared_buffer = std::make_shared<SharedDepthBuffer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    try {
        MechMindProfiler mm_profiler(shared_buffer);
        auto depth_analyzer = std::make_shared<depth_analyzer::DepthAnalyzerNode>(shared_buffer);

        executor.add_node(mm_profiler.node);
        executor.add_node(depth_analyzer);
        executor.spin();
    } catch (mmind::eye::ErrorStatus error) {
        showError(error);
        return error.errorCode;
    }
    return 0;
}