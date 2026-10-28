#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>


struct Ball {
    cv::Point2f pos;
    cv::Point2f vel;
    int radius;
    cv::Scalar color;
    double hold_time{0.0};
};

class GameNode : public rclcpp::Node {
public:
       GameNode() : Node("game_node"){
        this->declare_parameter<std::vector<int64_t>>("hsv_lower",{35, 100, 100});
       this->declare_parameter<std::vector<int64_t>>("hsv_upper",{85, 255, 255});
        this->declare_parameter<int>("num_balls", 3);
        this->declare_parameter<int>("ball_radius", 40);
        this->declare_parameter<int>("min_area", 300);
        this->declare_parameter<double>("hold_duration", 2.0);
        this->declare_parameter<double>("ball_speed", 150.0);

      score_ = 0;
       last_time_ = this->now();

        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/camera/image_raw", 10, std::bind(&GameNode::image_callback,this, std::placeholders::_1)
       );
        pub_score_ = this->create_publisher<std_msgs::msg::Int32>("/game/score", 10);
        pub_debug_ = this->create_publisher<sensor_msgs::msg::Image>("/game/debug_image",10);

        init_balls();

        RCLCPP_INFO(this->get_logger(),"Game Node berhasil diinisialisasi");
    }
private:
    int score_;
    rclcpp::Time last_time_;
    std::vector<Ball> balls_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_score_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;

    void init_balls(){
        int num_balls = this->get_parameter("num_balls").as_int();
        int radius = this->get_parameter("ball_radius").as_int();
        double speed = this->get_parameter("ball_speed").as_double();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_angle(0.0, 2.0 * M_PI);
        std::uniform_real_distribution<> dis_x(100.0, 540.0);
        std::uniform_real_distribution<> dis_y(100.0, 380.0);

        balls_.clear();
        for (int i = 0;i<num_balls;i++){
            Ball ball;
            ball.pos = cv::Point2f(dis_x(gen),dis_y(gen));

            double angle = dis_angle(gen);
            ball.vel = cv::Point2f(std::cos(angle)* speed,std::sin(angle) *speed);

            ball.radius = radius;
            ball.color = cv::Scalar(rand() %255,rand()%255,rand()%255);
            ball.hold_time = 0.0;
            balls_.push_back(ball);
        }
    }

    void respawn_ball(Ball &ball, int frame_width, int frame_height){
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(ball.radius, frame_width - ball.radius);
        std::uniform_real_distribution<> dis_y(ball.radius, frame_height - ball.radius);
        std::uniform_real_distribution<> dis_angle(0.0, 2.0 * M_PI);

        double speed = this->get_parameter("ball_speed").as_double();
        ball.pos = cv::Point2f(dis_x(gen), dis_y(gen));

        double angle = dis_angle(gen);
        ball.vel = cv::Point2f(std::cos(angle) * speed, std::sin(angle) * speed);
        ball.hold_time = 0.0;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).seconds();
        if(dt <= 0.0|| dt >0.5) dt = 0.033;
        last_time_ = current_time;

        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        

        cv::Mat frame = cv_ptr->image;
        cv::Mat hsv,mask;
        cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);

        auto lower_param = this->get_parameter("hsv_lower").as_integer_array();
        auto upper_param = this->get_parameter("hsv_upper").as_integer_array();
        cv::Scalar lower(lower_param[0],lower_param[1], lower_param[2]);
        cv::Scalar upper(upper_param[0],upper_param[1], upper_param[2]);

        cv::inRange(hsv, lower, upper, mask);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool object_detected = false;
        cv::Point2f object_center;
        int min_area = this->get_parameter("min_area").as_int();
        double max_area = 0;

        for(const auto &contour : contours){
            double area = cv::contourArea(contour);
            if(area > min_area && area > max_area){
                max_area = area;
                cv::Moments m = cv::moments(contour);
                if(m.m00 != 0){
                    object_center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
                    object_detected = true;

                    cv::Rect bound_rect = cv::boundingRect(contour);
                    cv::rectangle(frame, bound_rect,cv::Scalar(0,255,0),2);
                    cv::circle (frame,object_center,5, cv::Scalar(0,0,255),-1);
                }
            }
        }
    

     for(auto &ball : balls_) {
         ball.pos += ball.vel * static_cast<float>(dt);

         if(ball.pos.x - ball.radius < 0 || ball.pos.x + ball.radius > frame.cols){
             ball.vel.x *= -1;
            }
            if(ball.pos.y - ball.radius < 0 || ball.pos.y + ball.radius > frame.rows){
            ball.vel.y *= -1;
            }
        }

        int touched_ball_idx = -1;
        double min_dist = 1e9;
        if(object_detected){
            for(size_t i = 0; i < balls_.size();++i){
                double dist = cv::norm(balls_[i].pos - object_center);
                if(dist <= balls_[i].radius && dist < min_dist){
                    min_dist = dist;
                    touched_ball_idx = static_cast<int>(i);
                }
           }
        }

        double hold_duration = this->get_parameter("hold_duration").as_double();

        for(size_t i = 0;i<balls_.size();++i){
            if(static_cast<int>(i) == touched_ball_idx){
                balls_[i].hold_time += dt;

                if(balls_[i].hold_time>=hold_duration){
                    score_++;

                    std_msgs::msg::Int32 score_msg;
                    score_msg.data = score_;
                    pub_score_->publish(score_msg);

                    respawn_ball(balls_[i],frame.cols,frame.rows);
                }
            }else balls_[i].hold_time = 0.0;

            cv::circle(frame,balls_[i].pos, balls_[i].radius, balls_[i].color,2);

            float progress = static_cast<float>(balls_[i].hold_time / hold_duration);
            if(progress > 0.0f){
                cv::ellipse(frame,balls_[i].pos, cv::Size(balls_[i].radius,balls_[i].radius),0,-90,-90 + (progress *360), cv::Scalar(0,255,255),4);

            }
        }

        cv::putText(frame,"score: " + std::to_string(score_),cv::Point(20,40),cv::FONT_HERSHEY_SIMPLEX,1.2,cv::Scalar(255,255,255),3);

        sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(msg->header,"bgr8",frame).toImageMsg();
        pub_debug_->publish(*debug_msg);
    }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<GameNode>());
    rclcpp::shutdown();
    return 0;
}