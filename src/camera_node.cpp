#include <chrono>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node{
public:
    CameraNode() : Node("camera_node"){
        this->declare_parameter<std::string>("source","0");
        this->declare_parameter<double>("fps",30.0);
        this->declare_parameter<int>("width",640);
        this->declare_parameter<int>("height",480);

        std::string source = this->get_parameter("source").as_string();
        double fps = this->get_parameter("fps").as_double();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();

        if (source.length() == 1 && std::isdigit(source[0])) {
          cap_.open(std::stoi(source)); 
        } else {
          cap_.open(source);            
          is_video_file_ = true;
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuka sumber video: %s", source.c_str());
            return;
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH,width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT,height_);

        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw",10);

        auto interval = std::chrono::duration<double>(1.0 / fps);
        timer_ = this->create_wall_timer(interval, std::bind(&CameraNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Camera Node berhasil berjalan dengan sumber: %s", source.c_str());
    }
private:
    void timer_callback(){
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            if (is_video_file_) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                cap_ >> frame;
            }
        }
        if (frame.empty()) return;

        cv::flip(frame, frame, 1);

        if (frame.cols != width_ || frame.rows != height_) {
            cv::resize(frame, frame, cv::Size(width_, height_));
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(header,"bgr8",frame).toImageMsg();

        pub_image_->publish(*msg);
    }

    
    cv::VideoCapture cap_;
    int width_;
    int height_;
    bool is_video_file_{false};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
