#ifndef velocity_remapper_FEEDBACK_CONTROLLER_HPP
#define velocity_remapper_FEEDBACK_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <utility>
#include <string>

#include "velocity_remapper/utils.hpp"
#include "velocity_remapper/constraints.hpp"

namespace velocity_remapper {

class FeedbackController {
public:
    FeedbackController(rclcpp::Node& node, 
                       tf2_ros::Buffer& tf_buffer, 
                       Constraints& constraints);

    // Main logic methods
    std::pair<tf2::Vector3, tf2::Vector3> master_slave_feedback_correction(
        Mode mode, rclcpp::Time stamp);

    std::pair<tf2::Vector3, tf2::Vector3> sensor_vel_feedback_correction(
        Mode mode, tf2::Vector3 &sensor_vel, tf2::Vector3 &v2_sphere, rclcpp::Time stamp);

    // Configuration / Parameters (public for easy access from node)
    std::string world_, ee1_frame_, ee2_frame_;
    std::chrono::milliseconds tf_timeout_{10};
    double fb_max_corr_ = 0.5;
    double deadband_ = 0.001;

    // PID Gains
    double fb_kp_ms_ = 0.2, fb_ki_ms_ = 1.2, fb_kd_ms_ = 0.00007;
    double fb_kp_sv_ = 0.14, fb_ki_sv_ = 0.014, fb_kd_sv_ = 0.0;
    double fb_kp_obj_fetched_ = 0.075, fb_ki_obj_fetched_ = 0.005, fb_kd_obj_fetched_ = 0.0;

    tf2::Vector3 prev_pos_e1_, prev_pos_e2_;
    tf2::Vector3 prev_ang_e1_, prev_ang_e2_;
    bool feedback_initialized_;
    // Helper to set the publisher used in logic
    void set_pos_err_publisher(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub) {
        pos_err_x_pub_ = pub;
    }

private:
    rclcpp::Node& node_;
    tf2_ros::Buffer& tf_buffer_;
    Constraints& constraints_;

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    // State variables
    rclcpp::Time last_feedback_time_{0, 0, RCL_STEADY_TIME};
    
    tf2::Vector3 prev_error_pos_, prev_error_ang_;
    tf2::Vector3 prev_error_1, prev_error_2;
    
    tf2::Vector3 pid_integral_pos_, pid_integral_ang_;
    tf2::Vector3 pid_integral_1, pid_integral_2;
    
    tf2::Vector3 init_pos_diff, init_ang_diff;
    
    // Internal working variables used in implementation
    tf2::Vector3 pos_diff, ang_diff, pos_err, ang_err;
    double fb_kp_, fb_ki_, fb_kd_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_err_x_pub_;
};

} // namespace velocity_remapper

#endif