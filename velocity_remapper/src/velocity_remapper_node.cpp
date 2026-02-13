#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <string>
#include <chrono>
#include <cmath>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sstream>
#include <algorithm>
#include <moveit_msgs/msg/servo_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "velocity_remapper/utils.hpp"              // <--- NEW
#include "velocity_remapper/feedback_controller.hpp"
#include "velocity_remapper/constraints.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using namespace velocity_remapper; // To use Mode, utils, FeedbackController


class PoseRouter : public rclcpp::Node {
public:
  PoseRouter()
  : Node("velocity_remapper"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {
    
    constraints_ = std::make_unique<Constraints>(this, tf_buffer_);
    feedback_controller_ = std::make_unique<FeedbackController>(*this, tf_buffer_, *constraints_);
    // --- Parameters ---
    declare_parameter<std::string>("source_topic", "neura/cmd_vel");
    declare_parameter<std::string>("out1_topic", "neura_1/cmd_vel");
    declare_parameter<std::string>("out2_topic", "neura_2/cmd_vel");
    declare_parameter<std::string>("base1_frame", "neura_1/link0");
    declare_parameter<std::string>("base2_frame", "neura_2/link0");
    declare_parameter<std::string>("ee1_frame", "neura_1/ee_tcp");
    declare_parameter<std::string>("ee2_frame", "neura_2/ee_tcp");
    declare_parameter<std::string>("world_frame", "world_link");   // set to "map"/"odom" if that's your root
    declare_parameter<std::string>("mode", "fixed");          // "fixed" or "mirror"
    declare_parameter<int>("tf_timeout_ms", 550);

    declare_parameter<bool>("neura_1_enabled", true); 
    declare_parameter<bool>("neura_2_enabled", true); 
    declare_parameter<double>("surface_normal_x", 0.0);
    declare_parameter<double>("surface_normal_y", 0.0);
    declare_parameter<double>("surface_normal_z", 0.0); 
    declare_parameter<bool>("master_slave_control", true); // true: master-slave, false: sensor-velocity

    // --- PID feedback parameters (new) --- ms: master-slave, sv: sensor-velocity
    declare_parameter<double>("fb_pid_kp_ms", 0.2); // was 0.15, 0.3050, 0.205, 0.2
    declare_parameter<double>("fb_pid_ki_ms", 1.2); // was 0.2, 0.1, 1,75
    declare_parameter<double>("fb_pid_kd_ms", 0.00007); //0.00007
    declare_parameter<double>("fb_pid_kp_sv", 0.14); // was 0.15, 0.3050, 0.205
    declare_parameter<double>("fb_pid_ki_sv", 0.014); // was 0.2, 0.1, 1,75
    declare_parameter<double>("fb_pid_kd_sv", 0.0); //0.00007
    declare_parameter<double>("fb_pid_kp_obj_fetched", 0.075); 
    declare_parameter<double>("fb_pid_ki_obj_fetched", 0.005); 
    declare_parameter<double>("fb_pid_kd_obj_fetched", 0.0); 
    declare_parameter<double>("fb_max_correction", 0.05); // m/s absolute clamp
    declare_parameter<double>("fb_speed_unit", 0.001);    // base speed step (m/s) that PID scales

    // Load params
    source_topic_ = get_parameter("source_topic").as_string();
    out1_topic_   = get_parameter("out1_topic").as_string();
    out2_topic_   = get_parameter("out2_topic").as_string();
    base1_        = get_parameter("base1_frame").as_string();
    base2_        = get_parameter("base2_frame").as_string();
    ee2_frame_    = get_parameter("ee2_frame").as_string();
    ee1_frame_    = get_parameter("ee1_frame").as_string();
    world_        = get_parameter("world_frame").as_string();
    mode_         = parse_mode(get_parameter("mode").as_string());
    tf_timeout_   = std::chrono::milliseconds(get_parameter("tf_timeout_ms").as_int());

    neura_1_enabled_ = get_parameter("neura_1_enabled").as_bool();
    neura_2_enabled_ = get_parameter("neura_2_enabled").as_bool();
    surface_normal_x_ = get_parameter("surface_normal_x").as_double();
    surface_normal_y_ = get_parameter("surface_normal_y").as_double();
    surface_normal_z_ = get_parameter("surface_normal_z").as_double();
    master_slave_control_ = get_parameter("master_slave_control").as_bool();

    // Read PID params
    fb_kp_ms_ = get_parameter("fb_pid_kp_ms").as_double();
    fb_ki_ms_ = get_parameter("fb_pid_ki_ms").as_double();
    fb_kd_ms_ = get_parameter("fb_pid_kd_ms").as_double();
    fb_kp_sv_ = get_parameter("fb_pid_kp_sv").as_double();
    fb_ki_sv_ = get_parameter("fb_pid_ki_sv").as_double();
    fb_kd_sv_ = get_parameter("fb_pid_kd_sv").as_double();
    fb_kp_obj_fetched_ = get_parameter("fb_pid_kp_obj_fetched").as_double();
    fb_ki_obj_fetched_ = get_parameter("fb_pid_ki_obj_fetched").as_double();
    fb_kd_obj_fetched_ = get_parameter("fb_pid_kd_obj_fetched").as_double();
    fb_max_corr_ = get_parameter("fb_max_correction").as_double();
    fb_speed_unit_ = get_parameter("fb_speed_unit").as_double();

    // initialize feedback state
    feedback_initialized_ = false;
    pid_integral_pos_ = tf2::Vector3(0.0,0.0,0.0);
    pid_integral_ang_ = tf2::Vector3(0.0,0.0,0.0);
    prev_error_pos_ = tf2::Vector3(0.0,0.0,0.0);
    prev_error_ang_ = tf2::Vector3(0.0,0.0,0.0);
    prev_pos_e1_ = tf2::Vector3(0.0,0.0,0.0);
    prev_pos_e2_ = tf2::Vector3(0.0,0.0,0.0);
    prev_ang_e1_ = tf2::Vector3(0.0,0.0,0.0);
    prev_ang_e2_ = tf2::Vector3(0.0,0.0,0.0);
    last_feedback_time_ = steady_clock_.now();

    // Dynamic params
    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &ps){
        for (auto &p: ps) {
          if (p.get_name()=="mode") mode_ = parse_mode(p.as_string());
          else if (p.get_name()=="world_frame") world_ = p.as_string();
          else if (p.get_name()=="base1_frame") base1_ = p.as_string();
          else if (p.get_name()=="base2_frame") base2_ = p.as_string();
          else if (p.get_name()=="ee2_frame") ee2_frame_ = p.as_string();
          else if (p.get_name()=="ee1_frame") ee1_frame_ = p.as_string();
          else if (p.get_name()=="tf_timeout_ms") tf_timeout_ = std::chrono::milliseconds(p.as_int());
          else if (p.get_name()=="fb_pid_kp_ms") fb_kp_ms_ = p.as_double();
          else if (p.get_name()=="fb_pid_ki_ms") fb_ki_ms_ = p.as_double();
          else if (p.get_name()=="fb_pid_kd_ms") fb_kd_ms_ = p.as_double();
          else if (p.get_name()=="fb_pid_kp_sv") fb_kp_sv_ = p.as_double();
          else if (p.get_name()=="fb_pid_ki_sv") fb_ki_sv_ = p.as_double();
          else if (p.get_name()=="fb_pid_kd_sv") fb_kd_sv_ = p.as_double();
          else if (p.get_name()=="fb_pid_kp_obj_fetched") fb_kp_obj_fetched_ = p.as_double();
          else if (p.get_name()=="fb_pid_ki_obj_fetched") fb_ki_obj_fetched_ = p.as_double();
          else if (p.get_name()=="fb_pid_kd_obj_fetched") fb_kd_obj_fetched_ = p.as_double();
          else if (p.get_name()=="fb_max_correction") fb_max_corr_ = p.as_double();
          else if (p.get_name()=="fb_speed_unit") fb_speed_unit_ = p.as_double();
          else if (p.get_name()=="neura_1_enabled") neura_1_enabled_ = p.as_bool();
          else if (p.get_name()=="neura_2_enabled") neura_2_enabled_ = p.as_bool();
          else if (p.get_name()=="surface_normal_x") surface_normal_x_ = p.as_double();
          else if (p.get_name()=="surface_normal_y") surface_normal_y_ = p.as_double();
          else if (p.get_name()=="surface_normal_z") surface_normal_z_ = p.as_double();
          else if (p.get_name()=="master_slave_control") master_slave_control_ = p.as_bool();
        }
        rcl_interfaces::msg::SetParametersResult ok; ok.successful=true; return ok;
      });

    feedback_controller_ -> world_ = world_;
    feedback_controller_ -> ee1_frame_ = ee1_frame_;
    feedback_controller_ -> ee2_frame_ = ee2_frame_;
    feedback_controller_ -> tf_timeout_ = tf_timeout_;
    feedback_controller_ -> prev_pos_e1_ = prev_pos_e1_;
    feedback_controller_ -> prev_pos_e2_ = prev_pos_e2_;
    feedback_controller_ -> prev_ang_e1_ = prev_ang_e1_;
    feedback_controller_ -> prev_ang_e2_ = prev_ang_e2_;
    feedback_controller_ -> feedback_initialized_ = feedback_initialized_;

    // IO
    sub_  = create_subscription<geometry_msgs::msg::TwistStamped>(
              source_topic_, rclcpp::SensorDataQoS(),
              std::bind(&PoseRouter::cb, this, _1));
    pub1_ = create_publisher<geometry_msgs::msg::TwistStamped>(out1_topic_, rclcpp::SystemDefaultsQoS());
    pub2_ = create_publisher<geometry_msgs::msg::TwistStamped>(out2_topic_, rclcpp::SystemDefaultsQoS());
    pos_err_x_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/velocity_remapper/pos_err_x", rclcpp::QoS(10));

    // Subscribe to motion type published by geo_constraints_tab (std_msgs/String)
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/motion_type", 10,
      std::bind(&PoseRouter::mode_cb, this, _1)
    );

    // Subscribe to surface normal published by geo_constraints_tab (std_msgs/String: "x,y,z")
    surface_normal_sub_ = create_subscription<std_msgs::msg::String>(
      "/surface_normal", 10,
      std::bind(&PoseRouter::surface_normal_cb, this, _1)
    );

    status1_sub_ = this->create_subscription<moveit_msgs::msg::ServoStatus>(
      "/neura_1/servo_node/status", rclcpp::QoS(10),
      [this](moveit_msgs::msg::ServoStatus::ConstSharedPtr msg){
        this->servo_status_cb(*msg, "neura_1");
      });

    status2_sub_ = this->create_subscription<moveit_msgs::msg::ServoStatus>(
      "/neura_2/servo_node/status", rclcpp::QoS(10),
      [this](moveit_msgs::msg::ServoStatus::ConstSharedPtr msg){
        this->servo_status_cb(*msg, "neura_2" );
      });

    control_type_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/control_type", rclcpp::QoS(10),
      std::bind(&PoseRouter::control_type_cb, this, _1)
     );

    // Subscribe to rescue_robots boolean to allow temporary override when servos are bad
    rescue_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/rescue_robots", rclcpp::QoS(10),
      std::bind(&PoseRouter::rescue_robots_cb, this, _1)
    );


    // Subscribe to selected robots topic
    selected_robot_sub_ = create_subscription<std_msgs::msg::String>(
      "/selected_robot", 10,
      std::bind(&PoseRouter::selected_robot_cb, this, _1)
    );

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/surface_markers", 1);


    RCLCPP_INFO(get_logger(),
      "velocity_remapper: in='%s'  out1='%s'  out2='%s'  base1='%s'  base2='%s'  world='%s'  mode=%s",
      source_topic_.c_str(), out1_topic_.c_str(), out2_topic_.c_str(),
      base1_.c_str(), base2_.c_str(), world_.c_str(),
      (mode_==Mode::FIXED ? "fixed" : "mirror"));
  }

private:
    // Helper that can take your tag
  void onStatus(const std::string& who, const moveit_msgs::msg::ServoStatus & msg)
  {
    RCLCPP_INFO(get_logger(), "[%s] ServoStatus code=%d msg=%s",
                who.c_str(), msg.code, msg.message.c_str());
  }
  // callback for external motion type control
  void mode_cb(const std_msgs::msg::String::SharedPtr msg) {
    try {
      const std::string s = msg->data;
      // lowercase copy for simple checks
      std::string low; low.reserve(s.size());
      for (char c: s) low.push_back(::tolower(c));

      // line modifier may be included as e.g. "fixed_rt_with_line" or "mirroring_with_line"
      line_motion_ = (low.find("line") != std::string::npos);
      if (line_motion_) {
        auto markers = create_line_marker(
            tf_buffer_, ee1_frame_, ee2_frame_, world_, 
            4, 0.01, {1.0f, 0.0f, 0.0f, 0.8f}, 
            this->now(), tf_timeout_
        );
        marker_pub_->publish(markers);
      } 
      else {
        auto markers = create_delete_markers({4}, {world_}, "line_constraint", this->now());
        marker_pub_->publish(markers);
      }

      // Determine base mode (ignore any 'line' token)
      // parse_mode is robust to extra tokens (we look for 'fix' / 'mirr')
      Mode new_mode = parse_mode(low);
      mode_ = new_mode;

      RCLCPP_INFO(get_logger(),
                  "motion_type received: '%s' -> mode=%s  line_motion=%s",
                  s.c_str(),
                  mode_to_cstr(mode_),
                  (line_motion_ ? "true" : "false"));

      feedback_controller_ -> feedback_initialized_ = false; // reset feedback state on mode change
      // 2. Handle Mirror Plane Visualization
      if (mode_ != Mode::MIRROR) {
        auto markers = create_delete_markers({3}, {world_}, "mirror_plane", this->now());
        marker_pub_->publish(markers);
      } 
      else {
        auto markers = create_mirror_plane_marker(
             tf_buffer_, ee1_frame_, ee2_frame_, world_, 
             this->now(), tf_timeout_
        );
        marker_pub_->publish(markers);
      } 
    }
    catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed to parse motion_type msg: %s", e.what());
    }
  } 

  // callback to receive surface normal as comma-separated string (assumed in ee1 frame)
  void surface_normal_cb(const std_msgs::msg::String::SharedPtr msg) {
    try {
      std::string s = msg->data;
      // replace commas with spaces and parse
      std::replace(s.begin(), s.end(), ',', ' ');
      std::istringstream iss(s);
      if (!(iss >> surface_normal_x_ >> surface_normal_y_ >> surface_normal_z_)) {
        RCLCPP_WARN(get_logger(), "Invalid surface_normal payload: '%s'", msg->data.c_str());
        surface_normal_set_ = false;
        auto markers = create_delete_markers({1,2}, {ee1_frame_, ee2_frame_}, "surface_panel", this->now());
        marker_pub_->publish(markers);        
        return;
      }
      n_world = tf2::Vector3 (surface_normal_x_, surface_normal_y_, surface_normal_z_);
      double L = n_world.length();
      if (L < 1e-6) {
        RCLCPP_WARN(get_logger(), "Received zero-length surface_normal, ignoring");
        surface_normal_set_ = false;
        auto markers = create_delete_markers({1,2}, {ee1_frame_, ee2_frame_}, "surface_panel", this->now());
        marker_pub_->publish(markers);
        return;
      }
      // normalize in world frame
      //n_world /= L;
      auto markers = create_surface_markers(
        tf_buffer_, world_, ee1_frame_, ee2_frame_, n_world,
        neura_1_enabled_, neura_2_enabled_,
        panel_w_, panel_h_, panel_t_,
        this->now(), tf_timeout_);
      marker_pub_->publish(markers);

      

    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed parsing surface_normal: %s", e.what());
      surface_normal_set_ = false;
      create_delete_markers({1,2}, {ee1_frame_, ee2_frame_}, "surface_panel", this->now());
    }
  }

  // servo status callbacks and flags
  void servo_status_cb(const moveit_msgs::msg::ServoStatus & msg, const std::string &robot) {
    int code = 0;
    try { code = msg.code; } catch(...) { }
    if (code != 0) {
      // latch the bad state and pause the subscription for this robot so it won't flip repeatedly
      if (robot == "neura_1") {
        servo1_bad_ = true;
        if (status1_sub_) { status1_sub_.reset(); RCLCPP_INFO(get_logger(), "Paused /neura_1/servo_node/status subscription after non-zero code"); }
      } else {
        servo2_bad_ = true;
        if (status2_sub_) { status2_sub_.reset(); RCLCPP_INFO(get_logger(), "Paused /neura_2/servo_node/status subscription after non-zero code"); }
      }
      RCLCPP_WARN(get_logger(), "%s servo status code %d -> outputs will be zeroed (subscription paused)", robot.c_str(), code);
    } else {
      // on zero we simply note OK; subscription may have been paused earlier and will be re-created when rescue is requested
      if (robot == "neura_1") servo1_bad_ = false; else servo2_bad_ = false;
      RCLCPP_DEBUG(get_logger(), "%s servo status OK", robot.c_str());
    }
  }

  void control_type_cb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg) return;
    master_slave_control_ = msg->data;
    RCLCPP_INFO(get_logger(), "control_type received: %s", master_slave_control_ ? "master_slave" : "sensor_velocity");
    feedback_controller_ -> feedback_initialized_ = false; // reset feedback state on control type change
  }

  // rescue_robots handler: msg->data == true requests allowing motion despite servo faults
  void rescue_robots_cb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg) return;
    if (msg->data) {
      rescue_requested_ = true;
      // Ensure we are subscribed to status topics so we can observe recovery while rescue is active
      if (!status1_sub_) {
        status1_sub_ = this->create_subscription<moveit_msgs::msg::ServoStatus>(
          "/neura_1/servo_node/status", rclcpp::QoS(10),
          [this](moveit_msgs::msg::ServoStatus::ConstSharedPtr m){ this->servo_status_cb(*m, "neura_1"); }
        );
        RCLCPP_INFO(get_logger(), "Resubscribed to /neura_1/servo_node/status for rescue checking");
      }
      if (!status2_sub_) {
        status2_sub_ = this->create_subscription<moveit_msgs::msg::ServoStatus>(
          "/neura_2/servo_node/status", rclcpp::QoS(10),
          [this](moveit_msgs::msg::ServoStatus::ConstSharedPtr m){ this->servo_status_cb(*m, "neura_2"); }
        );
        RCLCPP_INFO(get_logger(), "Resubscribed to /neura_2/servo_node/status for rescue checking");
      }
      // If servos are currently bad, enable override for 2s and then re-check
      if (servo1_bad_ || servo2_bad_) {
        rescue_override_ = true;
        // cancel any existing timer
        if (rescue_check_timer_) { rescue_check_timer_->cancel(); rescue_check_timer_.reset(); }
        rescue_check_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(2000),
          [this]() {
            // one-shot: stop timer first
            if (rescue_check_timer_) { rescue_check_timer_->cancel(); rescue_check_timer_.reset(); }
            // if servos still bad, remove override
            if (servo1_bad_ || servo2_bad_) {
              rescue_override_ = false;
              RCLCPP_WARN(get_logger(), "rescue_robots override expired: servos still report error -> freezing outputs");
            } else {
              // servos recovered within 2s; keep override active until rescue_requested_ cleared
              RCLCPP_INFO(get_logger(), "rescue_robots: servos recovered within 2s; override remains active");
            }
          }
        );
      } else {
        // servos already OK: enable override until save is cleared
        rescue_override_ = true;
      }
    } else {
      // clear any override/request
      rescue_requested_ = false;
      rescue_override_ = false;
      if (rescue_check_timer_) { rescue_check_timer_->cancel(); rescue_check_timer_.reset(); }
      // Optionally pause status subscriptions again if they had been running but servos were latched bad previously
      // (leave them running if you prefer continuous monitoring)
      // Here we do not automatically pause; the subscriptions will be paused only when a non-zero status arrives.
      RCLCPP_INFO(get_logger(), "rescue_robots cleared; normal servo gating restored");
    }
  }

  // Subscribe to selected robots topic
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selected_robot_sub_;

  // Track enabled robots
  std::string selected_robot_str_ = "none";

  void selected_robot_cb(const std_msgs::msg::String::SharedPtr msg) {
    selected_robot_str_ = msg->data;
    // Parse string: "neura_1", "neura_2", "neura_1-neura_2", or "none"
    neura_1_enabled_ = (selected_robot_str_.find("neura_1") != std::string::npos);
    neura_2_enabled_ = (selected_robot_str_.find("neura_2") != std::string::npos);
    RCLCPP_INFO(get_logger(), "Selected robots: %s (neura_1=%d, neura_2=%d)",
                selected_robot_str_.c_str(), neura_1_enabled_, neura_2_enabled_);
  }


  // -------- main callback --------
  void cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    // Timestamps
    const rclcpp::Time t_recv_steady = steady_clock_.now();
    const rclcpp::Time t_recv_ros    = this->now();

    // Frames
    const std::string F   = (msg->header.frame_id.empty() ? world_ : msg->header.frame_id); // incoming frame
    const std::string ee1 = ee1_frame_;   // e.g. "neura_1/flange"
    const std::string ee2 = ee2_frame_;   // e.g. "neura_2/flange"
    
    // If any servo reports a non-zero status code, publish zero twists and return
    if ((servo1_bad_ || servo2_bad_) && !rescue_override_) {
      geometry_msgs::msg::TwistStamped out1, out2;
      out1.header.stamp = this->now(); out1.header.frame_id = ee1;
      out2.header.stamp = this->now(); out2.header.frame_id = ee2;
      // twists default to zero, but set explicitly
      out1.twist.linear.x = out1.twist.linear.y = out1.twist.linear.z = 0.0;
      out1.twist.angular.x = out1.twist.angular.y = out1.twist.angular.z = 0.0;
      out2.twist.linear.x = out2.twist.linear.y = out2.twist.linear.z = 0.0;
      out2.twist.angular.x = out2.twist.angular.y = out2.twist.angular.z = 0.0;
      pub1_->publish(out1);
      pub2_->publish(out2);
      return;
    }

    const rclcpp::Time stamp = (msg->header.stamp.sec || msg->header.stamp.nanosec)
                              ? rclcpp::Time(msg->header.stamp)
                              : rclcpp::Time(0,0,RCL_ROS_TIME);


    // 0) Parse incoming vectors in its own frame F
    tf2::Vector3 vF(msg->twist.linear.x,  msg->twist.linear.y,  msg->twist.linear.z);
    tf2::Vector3 wF(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);

    // 1) Normalize: F -> ee1 (rotate-only) so downstream math assumes EE1
    tf2::Matrix3x3 R_e1_F; tf2::Vector3 t_dump;
    try {
      auto T_e1_F = tf_buffer_.lookupTransform(ee1, F, stamp, tf_timeout_);
      Rt_from_tf(T_e1_F, R_e1_F, t_dump);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                          "TF %s <- %s failed: %s", ee1.c_str(), F.c_str(), ex.what());
      return;
    }
    tf2::Vector3 v1, w1;
    rotate_only_map(R_e1_F, vF, wF, v1, w1);   // ^ee1 V1

    // If a surface normal is set (expressed in ee1), project v1 onto the surface plane
    if (surface_normal_set_) {
      tf2::Vector3 n = surface_normal_e1_;
      double dot = n.dot(v1);
      v1 = v1 - n * dot; // remove normal component
    }

    // 2) Map ee1 -> ee2 according to mode
    tf2::Vector3 v2, w2, v1_sphere, v2_sphere;

    // Use extracted mapping helpers
    bool ok = false;
    if (mode_ == Mode::FIXED || surface_normal_set_) {
      ok = constraints_->map_fixed(ee1, ee2, stamp, tf_timeout_, v1, w1, v2, w2);
    } 
    else if (mode_ == Mode::MIRROR) {
      ok = constraints_->map_mirror(ee1, ee2, stamp, tf_timeout_, v1, w1, v2, w2);
    }
    else if (mode_ == Mode::SPHERE) {
      ok = constraints_->map_sphere(ee1, ee2, world_, stamp, tf_timeout_, w1, w2, v1_sphere, v2_sphere);
      v1 = v1_sphere; // use spherical linear velocity for ee1
      v2 = v2_sphere; // use spherical linear velocity for ee2
    }
    else{
      ok = constraints_->map_fixed(ee1, ee2, stamp, tf_timeout_, v1, w1, v2, w2); // default to fixed
    } 
    if (!ok) return;

    // If surface normal is set, express it in ee2 and project v2 onto that plane
    if (surface_normal_set_ || surface_normal_x_ !=0.0 || surface_normal_y_ !=0.0 || surface_normal_z_ !=0.0) {
      try {
        n_world = tf2::Vector3(surface_normal_x_, surface_normal_y_, surface_normal_z_);
          // Transform normal from world frame into ee1 (flange) frame using TF
        auto T_e1_world = tf_buffer_.lookupTransform(ee1_frame_, world_, stamp, tf_timeout_);
        tf2::Matrix3x3 R_e1_world; tf2::Vector3 ttmp;
        Rt_from_tf(T_e1_world, R_e1_world, ttmp);
        surface_normal_e1_ = R_e1_world * n_world;
        double L2 = surface_normal_e1_.length();
        if (L2 < 1e-6) {
          RCLCPP_WARN(get_logger(), "Transformed surface normal has zero length, ignoring");
          surface_normal_set_ = false;
          return;
        }
        surface_normal_e1_ /= L2;
        surface_normal_set_ = true;
        auto T_ee2_ee1 = tf_buffer_.lookupTransform(ee2, ee1, stamp, tf_timeout_);
        tf2::Matrix3x3 R_21;
        Rt_from_tf(T_ee2_ee1, R_21, ttmp);
        tf2::Vector3 n2 = R_21 * surface_normal_e1_;
        double d2 = n2.dot(v2);
        v2 = v2 - n2 * d2;
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to transform normal ee1->ee2: %s", ex.what());
      }
    }

    // LINE mode: constrain linear motion to the line between ee1 and ee2
    if (line_motion_) {
      try {
        auto T_e1_e2 = tf_buffer_.lookupTransform(ee1, ee2, stamp, tf_timeout_);
        tf2::Vector3 t12(T_e1_e2.transform.translation.x,
                         T_e1_e2.transform.translation.y,
                         T_e1_e2.transform.translation.z);
        double L = t12.length();
        if (L > 1e-9) {
          tf2::Vector3 u1 = t12 / L; // unit along line expressed in ee1
          // project v1 onto line in ee1
          double a1 = u1.dot(v1);
          v1 = u1 * a1;
          // express unit vector in ee2 and project v2
          auto T_ee2_ee1 = tf_buffer_.lookupTransform(ee2, ee1, stamp, tf_timeout_);
          tf2::Matrix3x3 R_21; tf2::Vector3 _t;
          Rt_from_tf(T_ee2_ee1, R_21, _t);
          tf2::Vector3 u2 = R_21 * u1;
          double a2 = u2.dot(v2);
          v2 = u2 * a2;

        }
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "LINE mode TF failed: %s", ex.what());
      }
    }


    bool single_robot = (neura_1_enabled_ != neura_2_enabled_);
    // -------- Apply feedback correction so neura_2 follows neura_1 displacements ----------
    // compute correction (in ee2 frame) and add to v2 (linear)
    if (master_slave_control_ && !single_robot) {
      auto [fb_corr_pos, fb_corr_ang] = feedback_controller_ -> master_slave_feedback_correction(mode_, stamp);
      v2 += fb_corr_pos; // v2 is in ee2 frame; fb_corr expressed in ee2
      w2 += fb_corr_ang;
    }
    else if (!master_slave_control_ && !single_robot){
      auto [v1_corr, v2_corr]  = feedback_controller_ -> sensor_vel_feedback_correction(mode_, v1, v2, stamp);
      v1 += v1_corr; // v2 is in ee2 frame; fb_corr expressed in ee2
      v2 += v2_corr; // v2 is in ee2 frame; fb_corr expressed in ee2
    } 


    

    // 3) Build outputs (IMPORTANT: publish to the correct topics)
    // Robot 2: constrained result in ee2
    geometry_msgs::msg::TwistStamped out2;
    out2.header.stamp    = this->now();
    out2.header.frame_id = ee2;                 // publish in the frame the consumer expects
    out2.twist.linear.x  = v2.x();  out2.twist.linear.y  = v2.y();  out2.twist.linear.z  = v2.z();
    out2.twist.angular.x = w2.x();  out2.twist.angular.y = w2.y();  out2.twist.angular.z = w2.z();

    geometry_msgs::msg::TwistStamped out1;
    out1.header.stamp    = out2.header.stamp;
    out1.header.frame_id = ee1;
    out1.twist.linear.x  = v1.x();  out1.twist.linear.y  = v1.y();  out1.twist.linear.z  = v1.z();
    out1.twist.angular.x = w1.x();  out1.twist.angular.y = w1.y();  out1.twist.angular.z = w1.z();

    // If only one robot is enabled, zero the other robot's twist
    if (single_robot) {
      if (neura_1_enabled_ && !neura_2_enabled_) {
        // Only neura_1 moves, zero neura_2
        out2.twist.linear.x = out2.twist.linear.y = out2.twist.linear.z = 0.0;
        out2.twist.angular.x = out2.twist.angular.y = out2.twist.angular.z = 0.0;
      } else if (!neura_1_enabled_ && neura_2_enabled_) {
        // Only neura_2 moves, zero neura_1
        out1.twist.linear.x = out1.twist.linear.y = out1.twist.linear.z = 0.0;
        out1.twist.angular.x = out1.twist.angular.y = out1.twist.angular.z = 0.0;
      }
    }

    // 4) Latency (processing + end-to-end if source stamp present)
    const rclcpp::Time t_pub_steady = steady_clock_.now();
    const rclcpp::Time t_pub_ros    = this->now();
    const double proc_ms = (t_pub_steady - t_recv_steady).nanoseconds() / 1e6;
    double ros_network_ms = std::numeric_limits<double>::quiet_NaN();
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
      ros_network_ms = (t_recv_ros - rclcpp::Time(msg->header.stamp)).nanoseconds() / 1e6;
    RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, 2000,
                        "Latency: processing=%.3f ms  end-to-end=%.3f ms", proc_ms, ros_network_ms);

    // 5) Publish (correct order!)
    pub1_->publish(out1);   // neura_1/cmd_vel
    pub2_->publish(out2);   // neura_2/cmd_vel
    rescue_override_ = false; // override only applies to one message
  }


  // Members
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr surface_normal_sub_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status1_sub_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr status2_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub1_, pub2_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_err_x_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_type_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rescue_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  //---------- PID feedback members ----------
  double fb_kp_ms_ = 0.30;
  double fb_ki_ms_ = 0.0;
  double fb_kd_ms_ = 0.0;
  double fb_kp_sv_ = 0.30;
  double fb_ki_sv_ = 0.0;
  double fb_kd_sv_ = 0.0;
  double fb_kp_obj_fetched_ = 0.01;
  double fb_ki_obj_fetched_ = 0.0001;
  double fb_kd_obj_fetched_ = 0.0;
  double fb_max_corr_ = 0.03; // m/s max per-axis absolute
  double fb_speed_unit_ = 0.005; // base speed step (m/s) that PID scales
  double deadband_ = 0.001;  // m/s

  bool feedback_initialized_ = false;
  tf2::Vector3 prev_pos_e1_;
  tf2::Vector3 prev_pos_e2_;
  tf2::Vector3 prev_ang_e1_;
  tf2::Vector3 prev_ang_e2_;
  tf2::Vector3 init_pos_diff;
  tf2::Vector3 init_ang_diff;
  tf2::Vector3 pid_integral_pos_;
  tf2::Vector3 pid_integral_ang_;
  tf2::Vector3 prev_error_pos_;
  tf2::Vector3 prev_error_ang_; 
  rclcpp::Time last_feedback_time_;

  // Config
  std::string source_topic_, out1_topic_, out2_topic_;
  std::string base1_, base2_, world_, ee1_frame_, ee2_frame_;
  Mode mode_;
  bool neura_1_enabled_;
  bool neura_2_enabled_;
  double surface_normal_x_;
  double surface_normal_y_;
  double surface_normal_z_;
  bool master_slave_control_;

  bool show_surface_markers_ = true;   // toggle with a param if you like

  // panel dimensions
  double panel_w_ = 2.0;   // width  (m)
  double panel_h_ = 2.0;   // height (m)
  double panel_t_ = 0.005;  // thickness for visibility (m)

  tf2::Vector3 n_world;

  // Surface normal stored in ee1 frame
  tf2::Vector3 surface_normal_e1_;
  bool surface_normal_set_ = false;
  // servo status flags
  bool servo1_bad_ = false;
  bool servo2_bad_ = false;
  // rescue_robots override flags and timer
  bool rescue_requested_ = false;
  bool rescue_override_ = false;
  rclcpp::TimerBase::SharedPtr rescue_check_timer_;
  std::chrono::milliseconds tf_timeout_;
  // Line-motion modifier (true if incoming motion_type contains "line")
  bool line_motion_ = false;
  std::unique_ptr<Constraints> constraints_;
  std::unique_ptr<FeedbackController> feedback_controller_;
 }; 
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseRouter>());
  rclcpp::shutdown();
  return 0;
}
