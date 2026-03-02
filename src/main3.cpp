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
        MechMindProfiler3 mm_profiler;
        executor.add_node(mm_profiler.node);
        executor.spin();
    } catch (mmind::eye::ErrorStatus error) {
        showError(error);
        return error.errorCode;
    }
    return 0;
}
