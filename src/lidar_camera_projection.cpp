#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp> 
#include <opencv2/opencv.hpp> 

using namespace std::chrono_literals;
 
class MinimalImagePublisher : public rclcpp::Node {
public:
    MinimalImagePublisher() : Node("lidar_camera_projection") {
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw",
        10,
        std::bind(&MinimalImagePublisher::image_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ouster/points",
        10,
        std::bind(&MinimalImagePublisher::lidar_callback, this, std::placeholders::_1));

    is_lidar = false;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    if (is_lidar) {
        std::cout << lidar_msg_->header.stamp.nanosec << std::endl;
    }

    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    lidar_msg_ = msg;

    }


    cv_bridge::CvImagePtr cv_ptr;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Image::SharedPtr img_msg_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

    sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

    bool is_lidar;
    size_t count_;

};
 
int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
