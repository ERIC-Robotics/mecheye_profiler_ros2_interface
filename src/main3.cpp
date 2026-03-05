#include <csignal>
#include <MechMindProfiler3.h>

void signalHandler(int signum) { rclcpp::shutdown(); }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    rclcpp::executors::MultiThreadedExecutor executor;
    try {
        auto node = std::make_shared<mechmind3::MechMindProfiler3>(rclcpp::NodeOptions{});
        executor.add_node(node);
        executor.spin();
    } catch (mmind::eye::ErrorStatus error) {
        showError(error);
        return error.errorCode;
    }
    return 0;
}
