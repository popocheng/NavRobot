#ifndef GLOBAL2LOCAL_H
#define GLOBAL2LOCAL_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <msg_set_msgs/msg/area_sample.h>
#include <msg_set_msgs/msg/cover_task.hpp>
#include <msg_set_msgs/msg/jump_task.hpp>
#include <msg_set_msgs/msg/multi_goal.hpp>
#include <msg_set_msgs/msg/multi_task.hpp>
#include <msg_set_msgs/msg/object_detection.hpp>
#include <msg_set_msgs/msg/targets_pose.hpp>
#include <msg_set_msgs/msg/usm_algo_cmd.hpp>
#include <msg_set_msgs/msg/usm_data_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

class Global2Local : public rclcpp::Node
{
public:
    explicit Global2Local();

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  world_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          modified_odom_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::UsmAlgoCmd>::SharedPtr    modified_setpoint_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::MultiTask>::SharedPtr     multitask_world_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::JumpTask>::SharedPtr      jumptask_world_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::MultiTask>::SharedPtr     returntask_world_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::CoverTask>::SharedPtr     covertask_world_pub_;
    rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr   coverpath_world_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr covergoalpoint_world_pub_;
    rclcpp::Publisher<msg_set_msgs::msg::TargetsPose>::SharedPtr   targetgps_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr armworld_pub_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr        global_pos_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::UsmDataInfo>::SharedPtr     arm_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::UsmAlgoCmd>::SharedPtr      setpoint_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::MultiTask>::SharedPtr       multitask_global_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::JumpTask>::SharedPtr        jumptask_global_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::MultiTask>::SharedPtr       returntask_global_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::CoverTask>::SharedPtr       covertask_global_sub_;
    rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr     coverpath_global_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr   covergoalpoint_global_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr        origin_gps_sub_;
    rclcpp::Subscription<msg_set_msgs::msg::ObjectDetection>::SharedPtr targetpose_body_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr               usm_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr            tof_info_sub_;

    double                           origin_lat_, origin_lon_, origin_alt_;
    double                           origin_x_ = 0.0, origin_y_ = 0.0, origin_z_ = 0.0;
    double                           drone_lat_ = -1.0, drone_lon_ = -1.0, drone_alt_ = -1.0;
    int                              drone_id_, origin_id_;
    Eigen::Vector3d                  world_local_position_, current_local_position_, dif_pose_;
    bool                             arm_gps_flage_ = false, arm_pose_flag_ = false, armed_flag_ = false;
    bool                             origin_use_drone_;
    bool                             log_out;
    double                           arm_pose_[3];
    double                           arm_gps_[3];
    double                           tof_sensor_hgt_;
    double                           uav_state_;
    bool                             get_arm_from_usm   = false;
    bool                             messagePrinted     = false;
    bool                             getglobal_flag     = false;
    bool                             get_origin_from_ui = false;
    msg_set_msgs::msg::MultiGoal     move_pose_, waypoint_;
    msg_set_msgs::msg::MultiTask     multitask_global, multitask_local, returntask_global, returntask_local;
    msg_set_msgs::msg::JumpTask      jumptask_global, jumptask_local;
    msg_set_msgs::msg::CoverTask     covertask_local, covertask_global;
    mavros_msgs::msg::WaypointList   coverpath_local, coverpath_global;
    std_msgs::msg::Float32MultiArray goal_point_global, goal_point_local;
    nav_msgs::msg::Odometry          drone_world_odom;
    geometry_msgs::msg::PoseStamped  pose_msg;
    nav_msgs::msg::Odometry          modified_odom;
    Eigen::Vector3d                  ypr;

    void globalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void setpointCallback(const msg_set_msgs::msg::UsmAlgoCmd::SharedPtr msg);
    void armCallback(const msg_set_msgs::msg::UsmDataInfo::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void multitaskCallback(const msg_set_msgs::msg::MultiTask::SharedPtr msg);
    void jumptaskCallback(const msg_set_msgs::msg::JumpTask::SharedPtr msg);
    void returntaskCallback(const msg_set_msgs::msg::MultiTask::SharedPtr msg);
    void covertaskCallback(const msg_set_msgs::msg::CoverTask::SharedPtr msg);
    void coverpathCallback(const mavros_msgs::msg::WaypointList::SharedPtr msg);
    void covergoalpointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void targetposeCallback(const msg_set_msgs::msg::ObjectDetection::SharedPtr msg);
    void usmStateCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void originGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void tofInfoCallBack(const sensor_msgs::msg::Range::SharedPtr msg);

    void setAreaSampleGlobalToWorld(msg_set_msgs::msg::AreaSample& as);
    void setWaypointListGlobalToWorld(mavros_msgs::msg::WaypointList& wl);
    void setpointGlobalToWorld(std_msgs::msg::Float32MultiArray& fm);
    void checkDistance(double originLat, double originLon, double droneLat, double droneLon);

    Eigen::Vector3d local_coordinates(double lat, double lon, double alt, double new_lat, double new_lon, double new_alt, double local_x, double local_y,
                                      double local_z);
    Eigen::Vector3d local_to_global(double local_x, double local_y, double local_z);
    Eigen::Vector3d calculateTargetPosition(double x, double y, double z, double dx, double dy, double dz, double yaw_rad);
};

#endif  // GLOBAL2LOCAL_H
