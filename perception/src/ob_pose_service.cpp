#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <perception/srv/get_ob_pose.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;

class ObPoseServiceCpp : public rclcpp::Node {
public:
    ObPoseServiceCpp() : Node("ob_pose_service_cpp"), fx_(0.0), fy_(0.0), cx_(0.0), cy_(0.0) {
        this->declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered");
        this->declare_parameter("info_topic", "/zed/zed_node/depth/camera_info");
        this->declare_parameter("target_frame", "map");
        this->declare_parameter("search_radius", 3);

        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        std::string info_topic = this->get_parameter("info_topic").as_string();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS qos(5);
        qos.best_effort();

        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic, qos, std::bind(&ObPoseServiceCpp::info_callback, this, _1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, qos, std::bind(&ObPoseServiceCpp::depth_callback, this, _1));

        srv_ = this->create_service<perception::srv::GetObPose>(
            "get_3d_poses", std::bind(&ObPoseServiceCpp::handle_service, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Service Ready. Listening to: %s", depth_topic.c_str());
    }

private:
    double fx_, fy_, cx_, cy_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_depth_msg_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Service<perception::srv::GetObPose>::SharedPtr srv_;

    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (fx_ == 0.0) {
            fx_ = msg->k[0];
            fy_ = msg->k[4];
            cx_ = msg->k[2];
            cy_ = msg->k[5];
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        latest_depth_msg_ = msg;
    }

    void handle_service(const std::shared_ptr<perception::srv::GetObPose::Request> request,
                        std::shared_ptr<perception::srv::GetObPose::Response> response) {

        auto local_depth = latest_depth_msg_;
        std::string target_frame = this->get_parameter("target_frame").as_string();
        int radius = this->get_parameter("search_radius").as_int();

        if (!local_depth || fx_ == 0.0) {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "No depth data or intrinsics available yet.");
            return;
        }

        geometry_msgs::msg::PoseArray result_poses;
        result_poses.header.frame_id = target_frame;
        result_poses.header.stamp = this->now();

        for (const auto& det : request->detections.detections) {
            int u = static_cast<int>(det.bbox.center.position.x);
            int v = static_cast<int>(det.bbox.center.position.y);

            double z = get_robust_depth(u, v, local_depth, radius);

            if (z > 0.0) {
                double x_c = (u - cx_) * z / fx_;
                double y_c = (v - cy_) * z / fy_;

                geometry_msgs::msg::PointStamped pt_cam;
                pt_cam.header = local_depth->header;
                pt_cam.point.x = x_c;
                pt_cam.point.y = y_c;
                pt_cam.point.z = z;

                try {
                    auto pt_map = tf_buffer_->transform(pt_cam, target_frame, std::chrono::milliseconds(100));

                    geometry_msgs::msg::Pose p;
                    p.position.x = pt_map.point.x;
                    p.position.y = pt_map.point.y;
                    p.position.z = pt_map.point.z;
                    p.orientation.w = 1.0;

                    result_poses.poses.push_back(p);
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN(this->get_logger(), "TF Transform failed: %s", ex.what());
                }
            }
        }

        response->poses = result_poses;
        response->success = true;
    }

    double get_robust_depth(int u, int v, const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg, int radius) {
        int w = depth_msg->width;
        int h = depth_msg->height;

        if (u < 0 || u >= w || v < 0 || v >= h) return -1.0;

        int u_min = std::max(0, u - radius);
        int u_max = std::min(w - 1, u + radius);
        int v_min = std::max(0, v - radius);
        int v_max = std::min(h - 1, v + radius);

        std::vector<float> valid_depths;
        valid_depths.reserve((radius * 2 + 1) * (radius * 2 + 1));

        bool is_32f = (depth_msg->encoding.find("32F") != std::string::npos);
        bool is_16u = (depth_msg->encoding.find("16U") != std::string::npos);

        const uint8_t* raw_data = depth_msg->data.data();

        for (int row = v_min; row <= v_max; ++row) {
            for (int col = u_min; col <= u_max; ++col) {
                int index = row * w + col;
                float z = 0.0;

                if (is_32f) {
                    z = reinterpret_cast<const float*>(raw_data)[index];
                } else if (is_16u) {
                    z = reinterpret_cast<const uint16_t*>(raw_data)[index] / 1000.0f;
                }

                if (std::isfinite(z) && z > 0.1f) {
                    valid_depths.push_back(z);
                }
            }
        }

        if (valid_depths.empty()) return -1.0;

        size_t n = valid_depths.size() / 2;
        std::nth_element(valid_depths.begin(), valid_depths.begin() + n, valid_depths.end());
        return valid_depths[n];
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObPoseServiceCpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
