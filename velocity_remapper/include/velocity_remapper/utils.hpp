#ifndef velocity_remapper_UTILS_HPP
#define velocity_remapper_UTILS_HPP

#include <string>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <rclcpp/time.hpp>
#include <array>
#include <optional>

namespace velocity_remapper {

// Helper enum (needed in both utils and main node)
enum class Mode { FIXED, MIRROR, LINE, SPHERE };

// Convert mode string to enum
Mode parse_mode(const std::string &s);

// Convert enum to string for logging
const char* mode_to_cstr(Mode m);

// Extract R,t from TransformStamped (target <- source)
void Rt_from_tf(const geometry_msgs::msg::TransformStamped &T,
                tf2::Matrix3x3 &R, tf2::Vector3 &t);

// Build adjoint mapping: w' = R w,  v' = R v + t x (R w)
void adjoint_map(const tf2::Matrix3x3 &R, const tf2::Vector3 &t,
                 const tf2::Vector3 &v, const tf2::Vector3 &w,
                 tf2::Vector3 &v_out, tf2::Vector3 &w_out);

// Rotate only mapping: w' = R w, v' = R v (ignores translation lever arm)
void rotate_only_map(const tf2::Matrix3x3 &R,
                     const tf2::Vector3 &v, const tf2::Vector3 &w,
                     tf2::Vector3 &v_out, tf2::Vector3 &w_out);

// Construct reflection matrix across plane with unit normal n
tf2::Matrix3x3 reflection_R(const tf2::Vector3 &n_unit);

// Quaternion to RPY
void quatToRPY(const tf2::Quaternion &q, double &r, double &p, double &y);

// Unwrap angles for continuous rotation tracking
double unwrapScalar(double now, double prev);
tf2::Vector3 unwrapVec(const tf2::Vector3& now, const tf2::Vector3& prev);

// Quaternion rotating +Z to a given unit normal
tf2::Quaternion quatFromZTo(const tf2::Vector3& n_in);

// Generate markers for surface panels (returns empty if TF fails)
visualization_msgs::msg::MarkerArray create_surface_markers(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& world_frame,
    const std::string& ee1_frame,
    const std::string& ee2_frame,
    const tf2::Vector3& surface_normal_world,
    bool neura_1_enabled,
    bool neura_2_enabled,
    double panel_w, double panel_h, double panel_t,
    const rclcpp::Time& stamp,
    const std::chrono::milliseconds& timeout
);

// Generate marker for mirror plane (returns empty if TF fails)
visualization_msgs::msg::MarkerArray create_mirror_plane_marker(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& ee1_frame,
    const std::string& ee2_frame,
    const std::string& display_frame,
    const rclcpp::Time& stamp,
    const std::chrono::milliseconds& timeout
);

// Generate marker for a line between EEs
visualization_msgs::msg::MarkerArray create_line_marker(
     const tf2_ros::Buffer& tf_buffer,
     const std::string& ee1_frame,
     const std::string& ee2_frame,
     const std::string& display_frame,
     int id,
     double width,
     const std::array<float, 4>& color, // {r, g, b, a}
     const rclcpp::Time& stamp,
     const std::chrono::milliseconds& timeout
);

// Generate markers to delete specific IDs
visualization_msgs::msg::MarkerArray create_delete_markers(
    const std::vector<int>& ids,
    const std::vector<std::string>& frames,
    const std::string& ns,
    const rclcpp::Time& stamp
);

} // namespace velocity_remapper

#endif // velocity_remapper_UTILS_HPP