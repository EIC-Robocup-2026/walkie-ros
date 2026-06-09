#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <chrono>
#include <string>

using std::placeholders::_1;

class CloudToMapNode : public rclcpp::Node {
public:
    CloudToMapNode() : Node("cloud_to_map") {
        this->declare_parameter("source_topic",
            "/zed_head/zed_node/point_cloud/cloud_registered/filtered");
        this->declare_parameter("target_topic",
            "/zed_head/zed_node/point_cloud/cloud_registered/filtered_map");
        this->declare_parameter("target_frame", "map");

        source_topic_ = this->get_parameter("source_topic").as_string();
        target_topic_ = this->get_parameter("target_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS qos(1);
        qos.best_effort();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            source_topic_, qos,
            std::bind(&CloudToMapNode::cloud_callback, this, _1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            target_topic_, qos);

        RCLCPP_INFO(this->get_logger(),
            "cloud_to_map ready: %s → %s [%s]",
            source_topic_.c_str(), target_topic_.c_str(), target_frame_.c_str());
    }

private:
    std::string source_topic_;
    std::string target_topic_;
    std::string target_frame_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                target_frame_, msg->header.frame_id,
                msg->header.stamp, std::chrono::milliseconds(50));
        } catch (const tf2::ExtrapolationException &) {
            // Stamp too new — retry with latest available transform
            try {
                tf = tf_buffer_->lookupTransform(
                    target_frame_, msg->header.frame_id,
                    rclcpp::Time(0));
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "TF fallback failed: %s", ex.what());
                return;
            }
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup failed: %s", ex.what());
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud_out;
        tf2::doTransform(*msg, cloud_out, tf);
        pub_->publish(cloud_out);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudToMapNode>());
    rclcpp::shutdown();
    return 0;
}
