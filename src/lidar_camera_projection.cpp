#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp> 
#include <opencv2/opencv.hpp> 

#include <string>
#include <vector>
#include <typeinfo>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

/*
Author : @sweunwave
LiDAR - Camera Calibration based on ROS2
*/

using namespace std::chrono_literals;

struct ProjectionPoints{
    cv::Point inRange_points;
    cv::Scalar color;
};

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

        filtered_lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/calibration/filtered_points",
            10
            );

        std::string calibration_params_path;
        this->declare_parameter<std::string>("calibration_params_path");
        this->get_parameter("calibration_params_path", calibration_params_path);

        std::string camera_params_path;
        this->declare_parameter<std::string>("camera_params_path");
        this->get_parameter("camera_params_path", camera_params_path);
    
        std::cout << calibration_params_path << std::endl;
        std::cout << camera_params_path << std::endl;

        Json::Reader calib_reader, camera_reader;
        Json::Value calib_actual_json, camera_actual_json;

        std::ifstream calib_file(calibration_params_path);
        calib_reader.parse(calib_file, calib_actual_json);

        std::ifstream camera_intrinsic_file(camera_params_path);
        camera_reader.parse(camera_intrinsic_file, camera_actual_json);
        
        json_to_mat(calib_actual_json["rvec"], rvec);
        json_to_mat(calib_actual_json["tvec"], tvec);
        json_to_mat(camera_actual_json["intrinsic_matrix"], camera_intrinsic_matrix);
        json_to_mat(camera_actual_json["dist_coeffs"], camera_dist_coeff);

        std::cout << "rvec: \n" << rvec << std::endl;
        std::cout << "tvec: \n" << tvec << std::endl;
        std::cout << "intrinsic_matrix: \n" << camera_intrinsic_matrix << std::endl;
        std::cout << "dist_coeffs: \n" << camera_dist_coeff << std::endl;

        is_lidar = false;
        is_img = false;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        is_img = true;

        if (is_lidar) {
            for (const auto &point : projection_points){
                cv::circle(cv_ptr->image, point.inRange_points, 2, point.color, -1);
            }
        }
        cv::imshow("projection_image", cv_ptr->image);
        cv::waitKey(1);
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud_dst;
        pcl::fromROSMsg(*msg, cloud_dst);
        ptr_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        *ptr_cloud = cloud_dst;

        // std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> dst;
        
        // @todo : 
        pcl::PassThrough<pcl::PointXYZI> filter;
        filter.setInputCloud(ptr_cloud);
        filter.setFilterFieldName("y");
        filter.setFilterLimits(-2, 2);
        filter.filter(*output);

        filter.setInputCloud(output);
        filter.setFilterFieldName("x");
        filter.setFilterLimits(-7, 0);
        filter.filter(*output);

        filter.setInputCloud(output);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(-2, 1);
        filter.filter(*output);

        dst = *output;

        auto pcd2_msg = cloud2cloudmsg(dst);
        filtered_lidar_pub_->publish(pcd2_msg);

        std::vector<cv::Point3f> points_3d;
        std::vector<float> intensities;
        points_3d.reserve(dst.size());
        intensities.reserve(dst.size());

        // convert pcl to cv vertor
        std::transform(dst.begin(), dst.end(), std::back_inserter(points_3d),
                       [](const pcl::PointXYZI& point) {
                           return cv::Point3f(point.x, point.y, point.z);
                       });

        // get intensity vector
        std::transform(dst.begin(), dst.end(), std::back_inserter(intensities),
                       [](const pcl::PointXYZI& point) {
                           return point.intensity;
                       });
        auto minmax = std::minmax_element(intensities.begin(), intensities.end());

        // std::chrono::duration<double>sec = std::chrono::system_clock::now() - start;
        // std::cout << "실행 시간(초) : " << sec.count() <<" seconds"<< std::endl;        

        cv::Mat img_points, jacobian;
        cv::projectPoints(points_3d, rvec, tvec, camera_intrinsic_matrix, camera_dist_coeff, img_points, jacobian);

        if (is_img){
            projection_points.clear();
            std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
            for (int i = 0; i < img_points.rows; ++i) {
                for (int j = 0; j < img_points.cols; ++j) {
                    cv::Point img_point = img_points.at<cv::Point2f>(i, j);
                    if (img_point.x <= cv_ptr->image.size().width && img_point.x >= 0 && img_point.y <= cv_ptr->image.size().height && img_point.y >= 0) {
                        // auto value = static_cast<unsigned int>((ptr_cloud->points[i].intensity / *(minmax.second)) * 255.0);

                        // @todo : Convert intensity to RGB       
                        auto _intensity = dst.points[i].intensity;
                        unsigned int b = static_cast<unsigned int>(*(minmax.second) - _intensity);
                        unsigned int g = static_cast<unsigned int>(_intensity <= *(minmax.second)/2 ? _intensity + *(minmax.second)/2 : *(minmax.second) - _intensity);
                        unsigned int r = static_cast<unsigned int>(_intensity);

                        // std::cout << "intensity : " << _intensity << std::endl;
                        // std::cout << "b g r : " << b << " " << g << " " << r << std::endl;

                        // ProjectionPoints _points {img_point, cv::Scalar(g/2, r/2, b)};
                        ProjectionPoints _points {img_point, cv::Scalar(0, 0, 255)};
                        projection_points.push_back(_points);
                    }
                }
            }
            is_lidar = true;
        }
    }

    void json_to_mat(const Json::Value &jsonValue, cv::Mat &mat) {
        mat = cv::Mat(jsonValue.size(), jsonValue[0].size(), CV_64F);

        for (int i = 0; i < jsonValue.size(); ++i) {
            for (int j = 0; j < jsonValue[0].size(); ++j) {
                mat.at<double>(i, j) = jsonValue[i][j].asDouble();
            }
        }
    }

    sensor_msgs::msg::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZI> cloud_src){
        sensor_msgs::msg::PointCloud2 cloudmsg;
        pcl::toROSMsg(cloud_src, cloudmsg);
        cloudmsg.header.frame_id = "os_sensor";
        return cloudmsg;
    }

    pcl::PointCloud<pcl::PointXYZI> cloudmsg2cloud(sensor_msgs::msg::PointCloud2 cloudmsg){
        pcl::PointCloud<pcl::PointXYZI> cloud_dst;
        pcl::fromROSMsg(cloudmsg, cloud_dst);
        return cloud_dst;
    }

    cv_bridge::CvImagePtr cv_ptr;
    
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_lidar_pub_;

    cv::Mat rvec, tvec, camera_intrinsic_matrix, camera_dist_coeff;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud;
    std::vector<ProjectionPoints> projection_points;

    bool is_lidar, is_img;
    size_t count_;

};
 
int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
