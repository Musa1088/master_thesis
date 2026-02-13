#include "velocity_remapper/constraints.hpp"

namespace velocity_remapper {

Constraints::Constraints(rclcpp::Node* node, tf2_ros::Buffer& tf_buffer)
    : node_(node), tf_buffer_(tf_buffer) {}

bool Constraints::map_fixed(const std::string &ee1, const std::string &ee2,
                            const rclcpp::Time &stamp,
                            const std::chrono::milliseconds &timeout,
                            const tf2::Vector3 &v1, const tf2::Vector3 &w1,
                            tf2::Vector3 &v2, tf2::Vector3 &w2) 
{
    try {
        tf2::Matrix3x3 R_21; tf2::Vector3 t_21;
        auto T_ee2_ee1 = tf_buffer_.lookupTransform(ee2, ee1, stamp, timeout);
        Rt_from_tf(T_ee2_ee1, R_21, t_21);
        rotate_only_map(R_21, v1, w1, v2, w2);
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "TF %s <- %s failed (fixed): %s", ee2.c_str(), ee1.c_str(), ex.what());
        return false;
    }
}

bool Constraints::map_mirror(const std::string &ee1, const std::string &ee2,
                             const rclcpp::Time &stamp,
                             const std::chrono::milliseconds &timeout,
                             const tf2::Vector3 &v1, const tf2::Vector3 &w1,
                             tf2::Vector3 &v2, tf2::Vector3 &w2) 
{
    try {
        // compute ee1 <- ee2 to obtain translation from ee1 to ee2 (t12 in ee1)
        auto T_e1_e2 = tf_buffer_.lookupTransform(ee1, ee2, stamp, timeout);
        tf2::Vector3 t12(T_e1_e2.transform.translation.x,
                         T_e1_e2.transform.translation.y,
                         T_e1_e2.transform.translation.z);
        double L = t12.length();
        
        if (L < 1e-9) {
            // Coincident EEs → just rotate-only ee1->ee2 (no mirror)
            auto T_ee2_ee1 = tf_buffer_.lookupTransform(ee2, ee1, stamp, timeout);
            tf2::Matrix3x3 R_21; tf2::Vector3 _;
            Rt_from_tf(T_ee2_ee1, R_21, _);
            rotate_only_map(R_21, v1, w1, v2, w2);
            return true;
        } else {
            tf2::Vector3 n = t12 / L;                   // unit normal in ee1
            tf2::Matrix3x3 Rref = reflection_R(n);      // reflect in ee1
            tf2::Vector3 v1m = Rref * v1;               // polar
            tf2::Vector3 w1m = -(Rref * w1);            // axial

            // express in ee2 (rotate-only to preserve SpaceMouse axis semantics)
            auto T_ee2_ee1 = tf_buffer_.lookupTransform(ee2, ee1, stamp, timeout);
            tf2::Matrix3x3 R_21; tf2::Vector3 _;
            Rt_from_tf(T_ee2_ee1, R_21, _);
            rotate_only_map(R_21, v1m, w1m, v2, w2);
            return true;
        }
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Mirror TF failed: %s", ex.what());
        return false;
    }
}

bool Constraints::map_sphere(const std::string &ee1, const std::string &ee2,
                             const std::string &world_frame,
                             const rclcpp::Time &stamp,
                             const std::chrono::milliseconds &timeout,
                             const tf2::Vector3 &w1_in, 
                             tf2::Vector3 &w2_out,
                             tf2::Vector3 &v1_sphere_out, tf2::Vector3 &v2_sphere_out) 
{
    try {
        // --- 1) Convert object twist from ee1 -> WORLD
        // Need world <- ee1 rotation
        auto T_W_E1 = tf_buffer_.lookupTransform(world_frame, ee1, stamp, timeout);
        tf2::Matrix3x3 R_W_E1; tf2::Vector3 dummy;
        Rt_from_tf(T_W_E1, R_W_E1, dummy);
        tf2::Matrix3x3 R_E1_W = R_W_E1.transpose(); // world -> ee1

        tf2::Vector3 w_obj_W = R_W_E1 * w1_in;   // world

        // --- 2) Get EE positions in WORLD, compute box center O and lever arms r1,r2
        auto T_W_E2 = tf_buffer_.lookupTransform(world_frame, ee2, stamp, timeout);
        tf2::Matrix3x3 R_W_E2;
        Rt_from_tf(T_W_E2, R_W_E2, dummy);
        tf2::Matrix3x3 R_E2_W = R_W_E2.transpose(); // world -> ee2

        // Map angular velocity w1 -> w2 (simple rotation mapping)
        // We use a dummy linear output for map_fixed logic inside here, or manual lookup
        tf2::Matrix3x3 R_21; tf2::Vector3 t_21;
        auto T_ee2_ee1 = tf_buffer_.lookupTransform(ee2, ee1, stamp, timeout);
        Rt_from_tf(T_ee2_ee1, R_21, t_21);
        
        // Calculate w2 (rotate only)
        w2_out = R_21 * w1_in; 

        tf2::Vector3 p1(T_W_E1.transform.translation.x,
                        T_W_E1.transform.translation.y,
                        T_W_E1.transform.translation.z);
        tf2::Vector3 p2(T_W_E2.transform.translation.x,
                        T_W_E2.transform.translation.y,
                        T_W_E2.transform.translation.z);

        tf2::Vector3 O  = (p1 + p2) * 0.5;
        tf2::Vector3 r1 = p1 - O;
        tf2::Vector3 r2 = p2 - O;

        // Guard degenerate grasp (coincident EEs) -> no rotational lever
        if ((p2 - p1).length2() < 1e-10) {
            r1 = tf2::Vector3(0,0,0);
            r2 = tf2::Vector3(0,0,0);
        }

        // --- 3) World twists each EE must follow to realize the rigid object motion
        tf2::Vector3 v1_sphere_world = w_obj_W.cross(r1);
        tf2::Vector3 v2_sphere_world = w_obj_W.cross(r2);
        
        v1_sphere_out = R_E1_W * v1_sphere_world; // express in ee1
        v2_sphere_out = R_E2_W * v2_sphere_world; // express in ee2

        return true;
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "SPHERE TF failed: %s", ex.what());
        return false;
    }
}

} // namespace velocity_remapper