#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <set>

class LidarMonitor : public rclcpp::Node {
public:
    LidarMonitor() : Node("lidar_monitor"), last_msg_time_(0, 0, RCL_ROS_TIME), is_calibration_done_(false) {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarMonitor::scan_callback, this, std::placeholders::_1));
        
        // ノード起動時の時刻を記録
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "LiDARキャリブレーション開始（3秒間）...");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 重複チェック
        rclcpp::Time current_msg_time = msg->header.stamp;
        if (current_msg_time <= last_msg_time_) return;
        last_msg_time_ = current_msg_time;

        // 経過時間の計算
        auto elapsed_time = this->now() - start_time_;

        if (!is_calibration_done_) {
            // 【最初の3秒間：キャリブレーション期間】
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                if (msg->ranges[i] < 0.8) {
                    // 0.8m以内のインデックスを除外リストに追加
                    masked_indices_.insert(i);
                }
            }

            if (elapsed_time.seconds() >= 3.0) {
                is_calibration_done_ = true;
                RCLCPP_INFO(this->get_logger(), "キャリブレーション完了。除外方向数: %zu", masked_indices_.size());
            }
            return; // 測定期間中は検知処理をスキップ
        }

        // 【3秒以降：通常モニタリング】
        float min_dist = msg->range_max;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            // 除外リストにある方向なら、距離計算に使わない
            if (masked_indices_.count(i)) continue;

            float range = msg->ranges[i];
            if (range > msg->range_min && range < min_dist) {
                min_dist = range;
            }
        }

        if (min_dist < 1.0) {
            RCLCPP_WARN(this->get_logger(), "警告！接近物あり（除外エリア外）: %.2f メートル", min_dist);
        } else {
            RCLCPP_INFO(this->get_logger(), "安全。最接近物: %.5f メートル", min_dist);
        }
    }

    // メンバ変数の定義
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Time last_msg_time_;
    rclcpp::Time start_time_;
    bool is_calibration_done_;
    std::set<size_t> masked_indices_;
}; // クラス定義の終わり

// ここから main 関数
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
