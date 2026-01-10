#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm> // std::min_element用

class LidarMonitor : public rclcpp::Node {
public:
    LidarMonitor() : Node("lidar_monitor") {
        // "/scan" トピックを購読する設定
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarMonitor::scan_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "LiDAR Monitor has started.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // ranges配列の中から有効な最小値（一番近い距離）を探す
        float min_dist = msg->range_max;

        for (auto range : msg->ranges) {
            // センサーの最小検知距離より大きく、かつ inf でない場合のみ比較
            if (range > msg->range_min && range < min_dist) {
                min_dist = range;
            }
        }

        // 0.8メートル以内に何かあれば警告を表示
        if (min_dist < 1.0) {
            RCLCPP_WARN(this->get_logger(), "警告！ 1.0m以内の接近物あり : %.2f メートル", min_dist);
        } else {
            RCLCPP_INFO(this->get_logger(), "安全です。 最接近物: %.2f メートル", min_dist);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMonitor>());
    rclcpp::shutdown();
    return 0;
}
