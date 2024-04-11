#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp> 
#include <opencv2/opencv.hpp> 

using namespace std::chrono_literals;
 
class MinimalImagePublisher : public rclcpp::Node {
public:
    MinimalImagePublisher() : Node("opencv_image_sub") {
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw",
        10,
        std::bind(&MinimalImagePublisher::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

    using namespace cv;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    Mat img;
	cvtColor(cv_ptr->image, img, COLOR_BGR2GRAY);
    
    Mat harris;
	cornerHarris(img, harris, 3, 3, 0.02);

	Mat harris_norm;
	normalize(harris, harris_norm, 0, 255, NORM_MINMAX, CV_8U);
    
    for (int j = 1; j < harris.rows - 1; j++)
	{
		for (int i = 1; i < harris.cols - 1; i++)
		{
			if (harris_norm.at<uchar>(j, i) > 100) {
				if (harris.at<float>(j, i) > harris.at<float>(j - 1, i) &&
					harris.at<float>(j, i) > harris.at<float>(j + 1, i) &&
					harris.at<float>(j, i) > harris.at<float>(j, i - 1) &&
					harris.at<float>(j, i) > harris.at<float>(j, i + 1))
				{
					circle(cv_ptr->image, Point(i, j), 5, Scalar(0, 0, 255), 2);
				}
					
			}
		}
	}

    cv::imshow("image", cv_ptr->image);
    cv::waitKey(1);

    }

    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Image::SharedPtr msg_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    size_t count_;

    

};
 
int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    // create a ros2 node
    auto node = std::make_shared<MinimalImagePublisher>();

    // process ros2 callbacks until receiving a SIGINT (ctrl-c)
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
