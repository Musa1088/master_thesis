#include "velocity_remapper/feedback_controller.hpp"

namespace velocity_remapper {

FeedbackController::FeedbackController(rclcpp::Node& node, 
                                       tf2_ros::Buffer& tf_buffer, 
                                       Constraints& constraints)
    : node_(node), tf_buffer_(tf_buffer), constraints_(constraints) {}

std::pair<tf2::Vector3, tf2::Vector3> FeedbackController::master_slave_feedback_correction(Mode mode, rclcpp::Time stamp) {
    try {
        auto T_W_E1 = tf_buffer_.lookupTransform(world_, ee1_frame_, stamp, tf_timeout_);
        auto T_W_E2 = tf_buffer_.lookupTransform(world_, ee2_frame_, stamp, tf_timeout_);

        tf2::Vector3 p1(T_W_E1.transform.translation.x, T_W_E1.transform.translation.y, T_W_E1.transform.translation.z);
        tf2::Vector3 p2(T_W_E2.transform.translation.x, T_W_E2.transform.translation.y, T_W_E2.transform.translation.z);
        tf2::Quaternion q1(T_W_E1.transform.rotation.x, T_W_E1.transform.rotation.y, T_W_E1.transform.rotation.z, T_W_E1.transform.rotation.w);
        tf2::Quaternion q2(T_W_E2.transform.rotation.x, T_W_E2.transform.rotation.y, T_W_E2.transform.rotation.z, T_W_E2.transform.rotation.w);

        q1.normalize(); q2.normalize();

        double r1, p1_rpy, y1, r2, p2_rpy, y2;
        quatToRPY(q1, r1, p1_rpy, y1);
        quatToRPY(q2, r2, p2_rpy, y2);

        tf2::Vector3 rot1_wrapped(r1, p1_rpy, y1);
        tf2::Vector3 rot2_wrapped(r2, p2_rpy, y2);

        tf2::Vector3 rot1 = unwrapVec(rot1_wrapped, prev_ang_e1_);
        tf2::Vector3 rot2 = unwrapVec(rot2_wrapped, prev_ang_e2_);

        tf2::Matrix3x3 R_W_E1; tf2::Vector3 tmp_t;
        Rt_from_tf(T_W_E1, R_W_E1, tmp_t);
        tf2::Matrix3x3 R_E1_W = R_W_E1.transpose();

        tf2::Matrix3x3 R_W_E2; tf2::Vector3 _;
        Rt_from_tf(T_W_E2, R_W_E2, _);
        tf2::Matrix3x3 R_E2_W = R_W_E2.transpose(); 

        rclcpp::Time now_steady = steady_clock_.now();
        double dt = (now_steady - last_feedback_time_).seconds();

        if (!feedback_initialized_ || dt <= 1e-6) {
            prev_pos_e1_ = p1; prev_pos_e2_ = p2;
            prev_ang_e1_ = rot1; prev_ang_e2_ = rot2;
            if (mode == Mode::MIRROR) {
                tf2::Vector3 p1_ee1 = R_E1_W * p1;
                tf2::Vector3 rot1_ee1 = R_E1_W * rot1;
                tf2::Vector3 p2_mirrored_ee2, rot2_mirrored_ee2;
                constraints_.map_mirror(ee1_frame_, ee2_frame_, stamp, tf_timeout_, p1_ee1, rot1_ee1, p2_mirrored_ee2, rot2_mirrored_ee2);
                init_pos_diff = (R_W_E2 * p2_mirrored_ee2) - p2;
                init_ang_diff = (R_W_E2 * rot2_mirrored_ee2) - rot2;
            } else {
                init_pos_diff = p1 - p2;
                init_ang_diff = rot1 - rot2;
            }
            prev_error_pos_ = prev_error_ang_ = pid_integral_pos_ = pid_integral_ang_ = tf2::Vector3(0,0,0);
            last_feedback_time_ = now_steady;
            feedback_initialized_ = true;
            return { tf2::Vector3(0,0,0), tf2::Vector3(0,0,0) };
        }

        tf2::Vector3 v1_world = (p1 - prev_pos_e1_) / dt;
        tf2::Vector3 v2_world = (p2 - prev_pos_e2_) / dt;
        tf2::Vector3 w1_world = (rot1 - prev_ang_e1_) / dt;
        tf2::Vector3 w2_world = (rot2 - prev_ang_e2_) / dt;

        tf2::Vector3 pos_error_world(0,0,0), ang_error_world(0,0,0);
        if (mode == Mode::MIRROR) {
            tf2::Vector3 v1_ee1 = R_E1_W * v1_world;
            tf2::Vector3 w1_ee1 = R_E1_W * w1_world;
            tf2::Vector3 p1_ee1 = R_E1_W * p1;
            tf2::Vector3 rot1_ee1 = R_E1_W * rot1;

            // call map_mirror to compute the mirrored v2 (expressed in ee2)
            tf2::Vector3 v2_mirrored_ee2, w2_mirrored_ee2, p2_mirrored_ee2, rot2_mirrored_ee2;
            bool mirror_ok = constraints_.map_mirror(ee1_frame_, ee2_frame_, stamp, tf_timeout_, v1_ee1, w1_ee1, v2_mirrored_ee2, w2_mirrored_ee2);
            constraints_.map_mirror(ee1_frame_, ee2_frame_, stamp, tf_timeout_, p1_ee1, rot1_ee1, p2_mirrored_ee2, rot2_mirrored_ee2);
            
            // tf2::Vector3 v2_m_ee2, w2_m_ee2, p2_m_ee2, rot2_m_ee2;
            // bool ok = constraints_.map_mirror(ee1_frame_, ee2_frame_, stamp, tf_timeout_, R_E1_W * v1_world, R_E1_W * w1_world, v2_m_ee2, w2_m_ee2);
            // constraints_.map_mirror(ee1_frame_, ee2_frame_, stamp, tf_timeout_, R_E1_W * p1, R_E1_W * rot1, p2_m_ee2, rot2_m_ee2);
            if (!mirror_ok) {
                pos_error_world = v1_world - v2_world; ang_error_world = w1_world - w2_world;
            } else {
                pos_error_world = (R_W_E2 * v2_mirrored_ee2) - v2_world;
                ang_error_world = (R_W_E2 * w2_mirrored_ee2) - w2_world;
                pos_err = ((R_W_E2 * p2_mirrored_ee2) - p2) - init_pos_diff;
                ang_err = ((R_W_E2 * rot2_mirrored_ee2) - rot2) - init_ang_diff;
            }
        } else if (mode == Mode::SPHERE) {
            pos_error_world = (-(R_E2_W * v1_world)) - (R_E2_W * v2_world);
            ang_error_world = (-(R_E2_W * w1_world)) - (R_E2_W * w2_world);
            pos_err = (-(R_E2_W * p1)) - (R_E2_W * p2);
            ang_err = (-(R_E2_W * rot1)) - (R_E2_W * rot2);
        } else {
            pos_error_world = v1_world - v2_world; ang_error_world = w1_world - w2_world;
            pos_err = (p1 - p2) - init_pos_diff; ang_err = (rot1 - rot2) - init_ang_diff;
        }

        pid_integral_pos_ += pos_error_world * dt;
        auto clamp_fn = [this](double x) { return std::max(-fb_max_corr_, std::min(fb_max_corr_, x)); };
        pid_integral_pos_.setX(clamp_fn(pid_integral_pos_.x()));
        pid_integral_pos_.setY(clamp_fn(pid_integral_pos_.y()));
        pid_integral_pos_.setZ(clamp_fn(pid_integral_pos_.z()));

        fb_kp_ = (mode == Mode::SPHERE) ? fb_kp_obj_fetched_ : fb_kp_ms_;
        fb_ki_ = (mode == Mode::SPHERE) ? fb_ki_obj_fetched_ : fb_ki_ms_;
        fb_kd_ = (mode == Mode::SPHERE) ? fb_kd_obj_fetched_ : fb_kd_ms_;

        tf2::Vector3 corr_world_pos = pos_error_world * fb_kp_ + pid_integral_pos_ * fb_ki_ + ((pos_error_world - prev_error_pos_) / dt) * fb_kd_;
        
        if (pos_err_x_pub_) {
            std_msgs::msg::Float64 msg; msg.data = pos_err.x();
            pos_err_x_pub_->publish(msg);
        }

        // Log diagnostics
        RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 50,
            "FB pos err (world) [%.4f, %.4f, %.4f]   ang err[%.4f, %.4f, %.4f]   pos2[%.4f, %.4f, %.4f]   dt=%.3f",
            pos_err.x(), pos_err.y(), pos_err.z(), ang_err.x(), ang_err.y(), ang_err.z(), p2.x(), p2.y(), p2.z(), dt);

        RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 50,
            "FB corr (world) [%.4f, %.4f, %.4f]   e[%.4f, %.4f, %.4f]   v1[%.4f, %.4f, %.4f]   v2[%.4f, %.4f, %.4f]   dt=%.3f fb_kp=%.3f ki=%.3f kd=%.3f",
            corr_world_pos.x(), corr_world_pos.y(), corr_world_pos.z(), pos_error_world.x(), pos_error_world.y(), pos_error_world.z(), v1_world.x(), v1_world.y(), v1_world.z(), v2_world.x(), v2_world.y(), v2_world.z(), dt, fb_kp_, fb_ki_, fb_kd_);

        auto dead_fn = [this](double x) { return (std::abs(x) < deadband_) ? 0.0 : x; };
        corr_world_pos.setX(dead_fn(clamp_fn(corr_world_pos.x())));
        corr_world_pos.setY(dead_fn(clamp_fn(corr_world_pos.y())));
        corr_world_pos.setZ(dead_fn(clamp_fn(corr_world_pos.z())));

        auto T_W2_E = tf_buffer_.lookupTransform(ee2_frame_, world_, stamp, tf_timeout_);
        tf2::Matrix3x3 R_W2_E; Rt_from_tf(T_W2_E, R_W2_E, _);

        prev_pos_e1_ = p1; prev_pos_e2_ = p2; prev_ang_e1_ = rot1; prev_ang_e2_ = rot2;
        prev_error_pos_ = pos_error_world; last_feedback_time_ = now_steady;

        return {R_W2_E * corr_world_pos, tf2::Vector3(0,0,0)};
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 10, "master_slave_feedback_correction: TF failed: %s", ex.what());
        return { tf2::Vector3(0,0,0), tf2::Vector3(0,0,0) };
    }
}

std::pair<tf2::Vector3, tf2::Vector3> FeedbackController::sensor_vel_feedback_correction(Mode mode, tf2::Vector3 &sensor_vel, tf2::Vector3 &v2_sphere, rclcpp::Time stamp) {
    try {
        auto T_E1_2W = tf_buffer_.lookupTransform(world_, ee1_frame_, stamp, tf_timeout_);
        auto T_E2_2W = tf_buffer_.lookupTransform(world_, ee2_frame_, stamp, tf_timeout_);
        auto T_E1_2E2 = tf_buffer_.lookupTransform(ee2_frame_, ee1_frame_, stamp, tf_timeout_);

        tf2::Vector3 p1(T_E1_2W.transform.translation.x, T_E1_2W.transform.translation.y, T_E1_2W.transform.translation.z);
        tf2::Vector3 p2(T_E2_2W.transform.translation.x, T_E2_2W.transform.translation.y, T_E2_2W.transform.translation.z);

        tf2::Matrix3x3 R_E1_2W, R_E2_2W, tmp_R; tf2::Vector3 tmp_t;
        Rt_from_tf(T_E1_2W, R_E1_2W, tmp_t);
        Rt_from_tf(T_E2_2W, R_E2_2W, tmp_t);
        tf2::Matrix3x3 R_W_E1 = R_E1_2W.transpose();

        rclcpp::Time now_steady = steady_clock_.now();
        double dt = (now_steady - last_feedback_time_).seconds();

        if (!feedback_initialized_ || dt <= 1e-6) {
            prev_pos_e1_ = p1; prev_pos_e2_ = p2;
            if (mode == Mode::MIRROR){
                tf2::Vector3 p1_ee1 = R_W_E1 * p1;
                tf2::Vector3 p2_mirrored_ee2, wtmp;
                constraints_.map_mirror(ee1_frame_, ee2_frame_, stamp, tf_timeout_, p1_ee1, tf2::Vector3(0,0,0), p2_mirrored_ee2, wtmp);
                tf2::Vector3 p2_mirrored_world = R_E2_2W * p2_mirrored_ee2;
                init_pos_diff = p2_mirrored_world - p2;
                } 
                else {
                init_pos_diff = p1 - p2;
                }
            prev_error_1 = prev_error_2 = pid_integral_1 = pid_integral_2 = tf2::Vector3(0,0,0);
            last_feedback_time_ = now_steady; feedback_initialized_ = true;
            return {tf2::Vector3(0,0,0), tf2::Vector3(0,0,0)};
        }

        tf2::Vector3 v1_world = (p1 - prev_pos_e1_) / dt;
        tf2::Vector3 v2_world = (p2 - prev_pos_e2_) / dt;
        tf2::Vector3 err1(0,0,0), err2(0,0,0);

        if (mode == Mode::MIRROR) {
            tf2::Vector3 v2_mirrored_ee2, wtmp, p2_mirrored_ee2;
            tf2::Vector3 p1_ee1 = R_W_E1 * p1;
            bool mirror_ok = constraints_.map_mirror(ee1_frame_, ee2_frame_, stamp, tf_timeout_, sensor_vel, tf2::Vector3(0,0,0), v2_mirrored_ee2, wtmp);
            constraints_.map_mirror(ee1_frame_, ee2_frame_, stamp, tf_timeout_, p1_ee1, tf2::Vector3(0,0,0), p2_mirrored_ee2, wtmp);
            if (!mirror_ok) {
            // fallback to nominal error in world (no mirror available)
            tf2::Vector3 sensor_vel_world = R_E1_2W * sensor_vel;

            err1 = sensor_vel_world - v1_world;
            err2 = sensor_vel_world - v2_world;
            } else {
            // Rt_from_tf(T_W_E2, R_E2_2W, _) produced R_E2_2W (ee2->world). To convert ee2->world multiply R_E2_2W * v_ee2.
            tf2::Vector3 v2_mirrored_world = R_E2_2W * v2_mirrored_ee2;
            tf2::Vector3 p2_mirrored_world = R_E2_2W * p2_mirrored_ee2;
            // error = v2_mirrored_world - v2_world (we want actual robot2 velocity to match mirrored target)
            err1 = sensor_vel - v1_world;
            err2 = v2_mirrored_world - v2_world;

            pos_err = (p2_mirrored_world - p2) - init_pos_diff;
            }
        } else {
            tf2::Vector3 sv_world = R_E1_2W * sensor_vel;
            err1 = sv_world - v1_world;
            err2 = (mode == Mode::SPHERE) ? ((R_E2_2W * v2_sphere) - v2_world) : (sv_world - v2_world);
        
            pos_err = (p1 - p2) - init_pos_diff;
        }

        fb_kp_ = (mode == Mode::SPHERE) ? fb_kp_obj_fetched_ : fb_kp_sv_;
        fb_ki_ = (mode == Mode::SPHERE) ? fb_ki_obj_fetched_ : fb_ki_sv_;
        fb_kd_ = (mode == Mode::SPHERE) ? fb_kd_obj_fetched_ : fb_kd_sv_;

        auto clamp_fn = [this](double x) { return std::max(-fb_max_corr_, std::min(fb_max_corr_, x)); };
        
        pid_integral_1 += err1 * dt; pid_integral_2 += err2 * dt;
        pid_integral_1.setX( clamp_fn(pid_integral_1.x()) );
        pid_integral_1.setY( clamp_fn(pid_integral_1.y()) );
        pid_integral_1.setZ( clamp_fn(pid_integral_1.z()) );

        pid_integral_2.setX( clamp_fn(pid_integral_2.x()) );
        pid_integral_2.setY( clamp_fn(pid_integral_2.y()) );
        pid_integral_2.setZ( clamp_fn(pid_integral_2.z()) );

        tf2::Vector3 c1_world = err1 * fb_kp_ + pid_integral_1 * fb_ki_ + ((err1 - prev_error_1)/dt) * fb_kd_;
        tf2::Vector3 c2_world = err2 * fb_kp_ + pid_integral_2 * fb_ki_ + ((err2 - prev_error_2)/dt) * fb_kd_;

        // Per-axis clamp & deadband
        auto dead = [this](double x){
            return (std::abs(x) < deadband_) ? 0.0 : x;
        };
        c1_world.setX( dead(clamp_fn(c1_world.x())) );
        c1_world.setY( dead(clamp_fn(c1_world.y())) );
        c1_world.setZ( dead(clamp_fn(c1_world.z())) );

        c2_world.setX( dead(clamp_fn(c2_world.x())) );
        c2_world.setY( dead(clamp_fn(c2_world.y())) );
        c2_world.setZ( dead(clamp_fn(c2_world.z())) );

        if (mode == Mode::SPHERE){
            RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 50,
            "FB vel err (world) [%.4f, %.4f, %.4f]  pos1[%.4f, %.4f, %.4f]  pos2[%.4f, %.4f, %.4f]  dt=%.3f",
            err1.x(), err1.y(), err1.z(),
            p1.x(), p1.y(), p1.z(),
            p2.x(), p2.y(), p2.z(),
            dt
            );

        }
        else{      
            RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 50,
            "FB pos err (world) [%.4f, %.4f, %.4f]  pos1[%.4f, %.4f, %.4f]  pos2[%.4f, %.4f, %.4f]  dt=%.3f",
            pos_err.x(), pos_err.y(), pos_err.z(),
            p1.x(), p1.y(), p1.z(),
            p2.x(), p2.y(), p2.z(),
            dt
        );
        } 

        RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 50,
            "FB corr (world) [%.4f, %.4f, %.4f]  v1[%.4f, %.4f, %.4f]  v2[%.4f, %.4f, %.4f]  dt=%.3f fb_kp=%.3f ki=%.3f kd=%.3f",
            c1_world.x(), c1_world.y(), c1_world.z(),
            v1_world.x(), v1_world.y(), v1_world.z(),
            v2_world.x(), v2_world.y(), v2_world.z(),
            dt, fb_kp_, fb_ki_, fb_kd_
        );

        auto T_W2_E2 = tf_buffer_.lookupTransform(ee2_frame_, world_, stamp, tf_timeout_);
        tf2::Matrix3x3 R_W2_E2; Rt_from_tf(T_W2_E2, R_W2_E2, tmp_t);

        prev_pos_e1_ = p1; prev_pos_e2_ = p2; prev_error_1 = err1; prev_error_2 = err2;
        last_feedback_time_ = now_steady;
        return {R_W_E1 * c1_world, R_W2_E2 * c2_world};
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 10,
                          "sensor_vel_feedback_correction: TF failed: %s", ex.what());
      return { tf2::Vector3(0,0,0), tf2::Vector3(0,0,0) };
    }
}

} // namespace velocity_remapper