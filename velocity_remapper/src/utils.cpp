#include "velocity_remapper/utils.hpp"
#include <iostream>

namespace velocity_remapper {

Mode parse_mode(const std::string &s) {
    std::string low; 
    low.reserve(s.size());
    for (char c: s) low.push_back(::tolower(c));

    if (low.find("fix") != std::string::npos || low.find("fixed") != std::string::npos) return Mode::FIXED;
    if (low.find("mirr") != std::string::npos || low.find("mirror") != std::string::npos) return Mode::MIRROR;
    if (low.find("spher") != std::string::npos || low.find("sphere") != std::string::npos) return Mode::SPHERE;
    return Mode::FIXED;
}

const char* mode_to_cstr(Mode m) {
    switch (m) {
        case Mode::FIXED:  return "fixed";
        case Mode::MIRROR: return "mirror";
        case Mode::LINE:   return "line";
        case Mode::SPHERE: return "sphere";
        default:           return "fixed";
    }
}

void Rt_from_tf(const geometry_msgs::msg::TransformStamped &T,
                tf2::Matrix3x3 &R, tf2::Vector3 &t) {
    tf2::Quaternion q(T.transform.rotation.x, T.transform.rotation.y,
                      T.transform.rotation.z, T.transform.rotation.w);
    R = tf2::Matrix3x3(q);
    t = tf2::Vector3(T.transform.translation.x,
                     T.transform.translation.y,
                     T.transform.translation.z);
}

void adjoint_map(const tf2::Matrix3x3 &R, const tf2::Vector3 &t,
                 const tf2::Vector3 &v, const tf2::Vector3 &w,
                 tf2::Vector3 &v_out, tf2::Vector3 &w_out) {
    w_out = R * w;
    v_out = R * v + t.cross(w_out);
}

void rotate_only_map(const tf2::Matrix3x3 &R,
                     const tf2::Vector3 &v, const tf2::Vector3 &w,
                     tf2::Vector3 &v_out, tf2::Vector3 &w_out) {
    w_out = R * w;
    v_out = R * v;
}

tf2::Matrix3x3 reflection_R(const tf2::Vector3 &n_unit) {
    const double nx = n_unit.x(), ny = n_unit.y(), nz = n_unit.z();
    return tf2::Matrix3x3(
      1.0 - 2*nx*nx,   -2*nx*ny,       -2*nx*nz,
      -2*ny*nx,        1.0 - 2*ny*ny,  -2*ny*nz,
      -2*nz*nx,        -2*nz*ny,        1.0 - 2*nz*nz
    );
}

void quatToRPY(const tf2::Quaternion &q, double &r, double &p, double &y) {
    tf2::Matrix3x3(q).getRPY(r, p, y);
}

double unwrapScalar(double now, double prev) {
    double d = std::remainder(now - prev, 2.0*M_PI);
    return prev + d;
}

tf2::Vector3 unwrapVec(const tf2::Vector3& now, const tf2::Vector3& prev) {
    return tf2::Vector3(
      unwrapScalar(now.x(), prev.x()),
      unwrapScalar(now.y(), prev.y()),
      unwrapScalar(now.z(), prev.z())
    );
}

tf2::Quaternion quatFromZTo(const tf2::Vector3& n_in) {
    tf2::Vector3 z(0,0,1);
    tf2::Vector3 n = n_in.normalized();
    double d = z.dot(n);
    if (d > 0.999999) {
        return tf2::Quaternion(0,0,0,1);
    } else if (d < -0.999999) {
        tf2::Quaternion q; q.setRPY(M_PI, 0, 0);
        return q;
    } else {
        tf2::Vector3 axis = z.cross(n).normalized();
        double angle = std::acos(std::clamp(d, -1.0, 1.0));
        tf2::Quaternion q(axis, angle); q.normalize();
        return q;
    }
}

visualization_msgs::msg::MarkerArray create_surface_markers(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& world_frame,
    const std::string& ee1_frame,
    const std::string& ee2_frame,
    const tf2::Vector3& n_world,
    bool neura_1_enabled,
    bool neura_2_enabled,
    double panel_w, double panel_h, double panel_t,
    const rclcpp::Time& stamp,
    const std::chrono::milliseconds& timeout)
{
    visualization_msgs::msg::MarkerArray out;
    if (n_world.length2() < 1e-12) return out;

    try {
        auto T_W_E1 = tf_buffer.lookupTransform(world_frame, ee1_frame, rclcpp::Time(0), timeout);
        auto T_W_E2 = tf_buffer.lookupTransform(world_frame, ee2_frame, rclcpp::Time(0), timeout);

        std::vector<tf2::Vector3> p;
        p.emplace_back(T_W_E1.transform.translation.x, T_W_E1.transform.translation.y, T_W_E1.transform.translation.z);
        p.emplace_back(T_W_E2.transform.translation.x, T_W_E2.transform.translation.y, T_W_E2.transform.translation.z);

        auto make_marker = [&](int id) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = world_frame;            
            m.header.stamp = stamp;
            m.ns = "surface_panel";
            m.id = id;
            m.type = visualization_msgs::msg::Marker::CUBE;
            m.action = visualization_msgs::msg::Marker::ADD;

            m.pose.position.x = p[id-1].x();
            m.pose.position.y = p[id-1].y();
            m.pose.position.z = p[id-1].z();

            // Orientation: Align local +Z with n_world
            tf2::Quaternion q = quatFromZTo(n_world);
            m.pose.orientation.x = q.x();
            m.pose.orientation.y = q.y();
            m.pose.orientation.z = q.z();
            m.pose.orientation.w = q.w();

            m.scale.x = panel_w;
            m.scale.y = panel_h;
            m.scale.z = panel_t;

            m.color.r = 0.2f; m.color.g = 0.7f; m.color.b = 1.0f; m.color.a = 0.25f;
            m.lifetime = rclcpp::Duration(0,0);
            m.frame_locked = true;
            return m;
        };

        if (neura_1_enabled) out.markers.push_back(make_marker(1));
        if (neura_2_enabled) out.markers.push_back(make_marker(2));

    } catch (const tf2::TransformException&) {
        // Return empty array on failure
    }
    return out;
}

visualization_msgs::msg::MarkerArray create_mirror_plane_marker(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& ee1,
    const std::string& ee2,
    const std::string& frame,
    const rclcpp::Time& stamp,
    const std::chrono::milliseconds& timeout)
{
    visualization_msgs::msg::MarkerArray out;
    try {
        auto T_F_E1 = tf_buffer.lookupTransform(frame, ee1, stamp, timeout);
        auto T_F_E2 = tf_buffer.lookupTransform(frame, ee2, stamp, timeout);

        tf2::Vector3 p1(T_F_E1.transform.translation.x, T_F_E1.transform.translation.y, T_F_E1.transform.translation.z);
        tf2::Vector3 p2(T_F_E2.transform.translation.x, T_F_E2.transform.translation.y, T_F_E2.transform.translation.z);

        tf2::Vector3 p_mid = 0.5 * (p1 + p2);
        tf2::Vector3 z = p2 - p1;
        double L = z.length();
        if (L < 1e-9) return out;

        z /= L;
        tf2::Quaternion q = quatFromZTo(z);

        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = stamp;
        m.ns = "mirror_plane";
        m.id = 3;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x = p_mid.x();
        m.pose.position.y = p_mid.y();
        m.pose.position.z = p_mid.z();
        m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y(); m.pose.orientation.z = q.z(); m.pose.orientation.w = q.w();

        m.scale.x = 2.0; m.scale.y = 2.0; m.scale.z = 0.002; // Fixed sizes from original
        m.color.r = 0.2f; m.color.g = 0.6f; m.color.b = 1.0f; m.color.a = 0.25f;
        m.lifetime = rclcpp::Duration(0,0);

        out.markers.push_back(m);
    } catch (const tf2::TransformException&) {
        // Return empty on error
    }
    return out;
}

visualization_msgs::msg::MarkerArray create_line_marker(
     const tf2_ros::Buffer& tf_buffer,
     const std::string& ee1,
     const std::string& ee2,
     const std::string& out_frame,
     int id,
     double width,
     const std::array<float, 4>& color,
     const rclcpp::Time& stamp,
     const std::chrono::milliseconds& timeout)
{
    visualization_msgs::msg::MarkerArray out;
    try {
        auto T_out_e1 = tf_buffer.lookupTransform(out_frame, ee1, stamp, timeout);
        auto T_out_e2 = tf_buffer.lookupTransform(out_frame, ee2, stamp, timeout);

        visualization_msgs::msg::Marker m;
        m.header.frame_id = out_frame;
        m.header.stamp = stamp;
        m.ns = "line_constraint";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point p1, p2;
        p1.x = T_out_e1.transform.translation.x; p1.y = T_out_e1.transform.translation.y; p1.z = T_out_e1.transform.translation.z;
        p2.x = T_out_e2.transform.translation.x; p2.y = T_out_e2.transform.translation.y; p2.z = T_out_e2.transform.translation.z;
        
        m.points.push_back(p1);
        m.points.push_back(p2);

        m.scale.x = width;
        m.color.r = color[0]; m.color.g = color[1]; m.color.b = color[2]; m.color.a = color[3];
        m.lifetime = rclcpp::Duration(0,0);
        m.pose.orientation.w = 1.0;

        out.markers.push_back(m);
    } catch (const tf2::TransformException&) {}
    return out;
}

visualization_msgs::msg::MarkerArray create_delete_markers(
    const std::vector<int>& ids,
    const std::vector<std::string>& frames,
    const std::string& ns,
    const rclcpp::Time& stamp)
{
    visualization_msgs::msg::MarkerArray arr;
    if (frames.size() != ids.size()) return arr;

    for (size_t i = 0; i < ids.size(); ++i) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frames[i];
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = ids[i];
        m.action = visualization_msgs::msg::Marker::DELETE;
        arr.markers.push_back(std::move(m));
    }
    return arr;
}

} // namespace velocity_remapper