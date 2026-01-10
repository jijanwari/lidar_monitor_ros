#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp" // 速度命令用

class AvoidanceNode : public rclcpp::Node {
public:
    AvoidanceNode() : Node("avoidance_node") {
        // LiDARデータを購読
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&AvoidanceNode::scan_callback, this, std::placeholders::_1));
        
        // 速度命令を送信（パブリッシャー）
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/model/model_with_lidar/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "避難ロボット、起動しました！");
    }

private:
    bool is_avoiding = false;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float min_dist = msg->range_max;
        for (auto range : msg->ranges) {
            if (range > msg->range_min && range < min_dist) {
                min_dist = range;
            }
        }

        auto drive_msg = geometry_msgs::msg::Twist();

        // callback関数の中
        if (min_dist < 1.0) {
            is_avoiding = true; // 0.8mを切ったら回避モードON
        } else if (min_dist > 1.5) {
            is_avoiding = false; // 1.1m以上離れたら前進に戻る
        }

        if (is_avoiding) {
            drive_msg.linear.x = -0.3;
            drive_msg.angular.z = 0.05; // 回転を少し強めに
            RCLCPP_WARN(this->get_logger(), "回避中！ （最寄り: %.2f m）, v: %.2f, ω: %.2f", min_dist, drive_msg.linear.x, drive_msg.angular.z);
        } else {
            drive_msg.linear.x = 0.5; // 前進も少しだけ速く
            drive_msg.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "前進中...（最寄り: %.2f m）", min_dist);
        }        

        cmd_vel_pub_->publish(drive_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}