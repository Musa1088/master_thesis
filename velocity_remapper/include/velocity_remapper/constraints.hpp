#ifndef velocity_remapper_CONSTRAINTS_HPP
#define velocity_remapper_CONSTRAINTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <string>
#include <chrono>

#include "velocity_remapper/utils.hpp" // For helper math functions

namespace velocity_remapper {

class Constraints {
public:
    Constraints(rclcpp::Node* node, tf2_ros::Buffer& tf_buffer);

    // Map ee1 -> ee2 using FIXED (rotate-only).
    // Returns true on success, false on TF error.
    bool map_fixed(const std::string &ee1, const std::string &ee2,
                   const rclcpp::Time &stamp,
                   const std::chrono::milliseconds &timeout,
                   const tf2::Vector3 &v1, const tf2::Vector3 &w1,
                   tf2::Vector3 &v2, tf2::Vector3 &w2);

    // Map ee1 -> ee2 using MIRROR semantics.
    // Returns true on success, false on TF error.
    bool map_mirror(const std::string &ee1, const std::string &ee2,
                    const rclcpp::Time &stamp,
                    const std::chrono::milliseconds &timeout,
                    const tf2::Vector3 &v1, const tf2::Vector3 &w1,
                    tf2::Vector3 &v2, tf2::Vector3 &w2);

    // Map ee1 -> ee2 using SPHERE semantics (object manipulation).
    // Calculates the required linear velocities for both EEs to rotate an object.
    // v1_sphere and v2_sphere are OUTPUTS. w1 and w2 are also updated/passed through.
    bool map_sphere(const std::string &ee1, const std::string &ee2,
                    const std::string &world_frame,
                    const rclcpp::Time &stamp,
                    const std::chrono::milliseconds &timeout,
                    const tf2::Vector3 &w1_in, 
                    tf2::Vector3 &w2_out,
                    tf2::Vector3 &v1_sphere_out, tf2::Vector3 &v2_sphere_out);

private:
    rclcpp::Node* node_;
    tf2_ros::Buffer& tf_buffer_;
};

} // namespace velocity_remapper

#endif // velocity_remapper_CONSTRAINTS_HPP