#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <std_msgs/msg/string.hpp>  
#include <sensor_msgs/msg/joy.hpp>
#include <cstdlib>
#include <ctime>
#include <chrono>

using namespace std::chrono_literals;  // クラスの外に移動
using std::placeholders::_1;           // クラスの外に移動

class InitialPosePublisher : public rclcpp::Node {
public:
    InitialPosePublisher() 
      : Node("initial_pose_publisher"), score_(0.0), min_offset_(-2.0), max_offset_(2.0), dmp_received_(false) {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        ekf_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ekf_pose", 10, std::bind(&InitialPosePublisher::ekf_pose_callback, this, _1));

        score_subscription_ = this->create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
            "/score_ndt", 10, std::bind(&InitialPosePublisher::score_callback, this, _1));

        authority_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/wof_controlhead", 10, std::bind(&InitialPosePublisher::authority_callback, this, _1));

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&InitialPosePublisher::joy_callback, this, _1));

        timer_ = this->create_wall_timer(5s, std::bind(&InitialPosePublisher::publish_initial_pose, this));

        last_ekf_pose_.pose.position.x = 0.0;
        last_ekf_pose_.pose.position.y = 0.0;
        last_ekf_pose_.pose.position.z = 0.0;
    }

private:
    void ekf_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        last_ekf_pose_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received ekf_pose: [%.2f, %.2f]", 
                    msg->pose.position.x, msg->pose.position.y);
    }

    void score_callback(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
        score_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received score: %.2f", score_);
    }

    void authority_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string data = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received authority message: %s", data.c_str());

        if (data == "DMP") {
            dmp_received_ = true;
            RCLCPP_INFO(this->get_logger(), "DMP mode activated. Waiting for joy input...");
        } else if (data == "TSUKUBA" && dmp_received_) {
            RCLCPP_INFO(this->get_logger(), "TSUKUBA mode activated. Publishing random initialpose...");
            publish_random_initial_pose();
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        size_t button_index = 1;

        if (button_index < msg->buttons.size() && msg->buttons[button_index] == 1) {
            RCLCPP_INFO(this->get_logger(), "Button %zu was pressed! Shutting down...", button_index);
            rclcpp::shutdown();
        }
    }

    void publish_initial_pose() {
        if (!dmp_received_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for DMP message...");
            return;
        }
    }

    void publish_random_initial_pose() {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";

        pose_msg.pose.pose.position.x = last_ekf_pose_.pose.position.x + generate_random_offset(min_offset_, max_offset_);
        pose_msg.pose.pose.position.y = last_ekf_pose_.pose.position.y + generate_random_offset(min_offset_, max_offset_);
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.pose.orientation = last_ekf_pose_.pose.orientation;

        pose_msg.pose.covariance[0] = 0.25;
        pose_msg.pose.covariance[7] = 0.25;
        pose_msg.pose.covariance[35] = 0.06853891909122467;

        publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published initial pose: [%.2f, %.2f]", 
                    pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y);
    }

    double generate_random_offset(double min, double max) {
        return min + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (max - min)));
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pose_subscription_;
    rclcpp::Subscription<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr score_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr authority_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped last_ekf_pose_;
    float score_;
    double min_offset_;
    double max_offset_;
    bool dmp_received_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}



// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <tier4_debug_msgs/msg/float32_stamped.hpp>
// #include <std_msgs/msg/string.hpp>  
// #include <sensor_msgs/msg/joy.hpp>
// #include <cstdlib>
// #include <ctime>
// #include <chrono>

// using namespace std::chrono_literals;
// using std::placeholders::_1;

// class InitialPosePublisher : public rclcpp::Node {
// public:
//     InitialPosePublisher() 
//       : Node("initial_pose_publisher"), score_(0.0), min_offset_(-2.0), max_offset_(2.0) {
//         std::srand(static_cast<unsigned int>(std::time(nullptr)));

//         publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

//         ekf_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/ekf_pose", 10, std::bind(&InitialPosePublisher::ekf_pose_callback, this, _1));

//         score_subscription_ = this->create_subscription<tier4_debug_msgs::msg::Float32Stamped>(
//             "/score_ndt", 10, std::bind(&InitialPosePublisher::score_callback, this, _1));

//         authority_subscription_ = this->create_subscription<std_msgs::msg::String>(
//             "/wof_controlhead",10,std::bind(&InitialPosePublisher::authority_callback, this, _1));

//         joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
//             "/joy",10,std::bind(&InitialPosePublisher::joy_callback, this, _1));

//         timer_ = this->create_wall_timer(5s, std::bind(&InitialPosePublisher::publish_initial_pose, this));

//         last_ekf_pose_.pose.position.x = 0.0;
//         last_ekf_pose_.pose.position.y = 0.0;
//         last_ekf_pose_.pose.position.z = 0.0;
//     }

// private:
//     void ekf_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//         last_ekf_pose_ = *msg;
//         RCLCPP_INFO(this->get_logger(), "Received ekf_pose: [%.2f, %.2f]", 
//                     msg->pose.position.x, msg->pose.position.y);
//     }

//     void score_callback(const tier4_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
//         score_ = msg->data;
//         RCLCPP_INFO(this->get_logger(), "Received score: %.2f", score_);
//     }

//     void authority_callback(const std_msgs::msg::String::SharedPtr msg){
//         authority_ = *msg;
//         RCLCPP_INFO(this->get_logger(), "Received authority message: %s", msg->data.c_str());
//     }
    
//     void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
//         // joy_ = *msg;
//         size_t button_index = 1;  // size_t に変更して型を一致させる

//         if (button_index < msg->buttons.size() && msg->buttons[button_index] == 1) {
//             RCLCPP_INFO(this->get_logger(), "Button %zu was pressed!", button_index);
//             // ボタンが押されたときの処理
//         } 
//         else {
//             RCLCPP_INFO(this->get_logger(), "Button %zu is not pressed", button_index);
//         }
//     }

//     void publish_initial_pose() {
//         geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

//         pose_msg.header.stamp = this->now();
//         pose_msg.header.frame_id = "map";
        
        
        
//         if(score_ < 2.3){
//             pose_msg.pose.pose.position.x = last_ekf_pose_.pose.position.x + generate_random_offset(min_offset_, max_offset_);
//             pose_msg.pose.pose.position.y = last_ekf_pose_.pose.position.y + generate_random_offset(min_offset_, max_offset_);
//             pose_msg.pose.pose.position.z = 0.0;
//             pose_msg.pose.pose.orientation = last_ekf_pose_.pose.orientation;

//             pose_msg.pose.covariance[0] = 0.25;
//             pose_msg.pose.covariance[7] = 0.25;
//             pose_msg.pose.covariance[35] = 0.06853891909122467;

//             publisher_->publish(pose_msg);
//             RCLCPP_INFO(this->get_logger(), "Published initial pose: [%.2f, %.2f]", 
//                         pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y);
//         }
//         else{


//         }

//             // pose_msg.pose.pose.position.x = last_ekf_pose_.pose.position.x + generate_random_offset(min_offset_, max_offset_);
//             // pose_msg.pose.pose.position.y = last_ekf_pose_.pose.position.y + generate_random_offset(min_offset_, max_offset_);
//             // pose_msg.pose.pose.position.z = 0.0;
//             // pose_msg.pose.pose.orientation = last_ekf_pose_.pose.orientation;

//             // pose_msg.pose.covariance[0] = 0.25;
//             // pose_msg.pose.covariance[7] = 0.25;
//             // pose_msg.pose.covariance[35] = 0.06853891909122467;

//             // publisher_->publish(pose_msg);
//             // RCLCPP_INFO(this->get_logger(), "Published initial pose: [%.2f, %.2f]", 
//             //             pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y);
//     }

//     double generate_random_offset(double min, double max) {
//         return min + static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX / (max - min)));
//     }

//     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pose_subscription_;
//     rclcpp::Subscription<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr score_subscription_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr authority_subscription_;
//     rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     geometry_msgs::msg::PoseStamped last_ekf_pose_;
//     std_msgs::msg::String authority_;
//     sensor_msgs::msg::Joy joy_;
//     float score_;
//     double min_offset_;  // クラスメンバー変数として宣言
//     double max_offset_;  // クラスメンバー変数として宣言
// };  // クラス定義の後にセミコロンが必要

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<InitialPosePublisher>());
//     rclcpp::shutdown();
//     return 0;
// }

