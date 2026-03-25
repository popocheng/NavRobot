#include "global2local.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
auto        qos_sensor  = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 10), rmw_qos_profile_sensor_data);
auto        qos_default = rclcpp::SystemDefaultsQoS();
std::string config_path = "src/msg_set_msgs/config/topic_key.yaml";
YAML::Node  config      = YAML::LoadFile(config_path);
std::string node_name   = "global2local_node";

Global2Local::Global2Local() : Node("Global2Local")
{
    origin_lat_ = this->declare_parameter("origin_lat", 47.3977418);
    origin_lon_ = this->declare_parameter("origin_lon", 8.5455938);
    origin_alt_ = this->declare_parameter("origin_alt", 488.166);
    drone_id_   = this->declare_parameter("drone_id", 0);
    // log_save_path = this->declare_parameter("log_save_path", "/home/wth/swarmhub/project/swarm_ws/src/global2local/log111/");
    // std::cout << "\033[1;32mlog_save_path: " << log_save_path << "\033[0m" << std::endl;
    // ~~~~~~~~~~~~~glog~~~~~~~~~~~~~
    // set_log_destination(node_name, log_save_path, drone_id_);
    // LOG_NODE_VAR_VALUE("drone_lat_", "drone_lon_", "drone_alt_", "px", "py", "pz", "vx", "vy", "vz", "yaw", "pitch", "roll", "algo_id", "algo_hold_state",
    //                    "set_type_mask", "set_px", "set_py", "set_pz", "set_vx", "set_vy", "set_vz", "set_ax", "set_ay", "set_az", "set_yaw", "set_yaw_rate");
    // ~~~~~~~~~~~~~glog~~~~~~~~~~~~~
    std::cout << "\033[1;32morigin_lat_: " << origin_lat_ << ", origin_lon_: " << origin_lon_ << "\033[0m" << std::endl;

    // Publisher
    world_pose_pub_    = this->create_publisher<geometry_msgs::msg::PoseStamped>(config["global_to_local"]["world_pose_topic"].as<std::string>(), qos_default);
    modified_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(config["global_to_local"]["world_odom_topic"].as<std::string>(), qos_default);
    modified_setpoint_pub_ =
        this->create_publisher<msg_set_msgs::msg::UsmAlgoCmd>(config["uav_state_machine"]["algo_cmd_topic"].as<std::string>(), qos_default);
    // xz
    multitask_world_pub_ =
        this->create_publisher<msg_set_msgs::msg::MultiTask>(config["global_to_local"]["world_multitask_topic"].as<std::string>(), qos_default);
    jumptask_world_pub_ =
        this->create_publisher<msg_set_msgs::msg::JumpTask>(config["global_to_local"]["world_jump_task_topic"].as<std::string>(), qos_default);
    returntask_world_pub_ =
        this->create_publisher<msg_set_msgs::msg::MultiTask>(config["global_to_local"]["world_return_task_topic"].as<std::string>(), qos_default);
    covertask_world_pub_ =
        this->create_publisher<msg_set_msgs::msg::CoverTask>(config["global_to_local"]["world_cover_task_topic"].as<std::string>(), qos_default);
    coverpath_world_pub_ =
        this->create_publisher<mavros_msgs::msg::WaypointList>(config["global_to_local"]["world_cover_path_topic"].as<std::string>(), qos_default);
    covergoalpoint_world_pub_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(config["global_to_local"]["world_cover_goalpoint_topic"].as<std::string>(), qos_default);
    targetgps_pub_ = this->create_publisher<msg_set_msgs::msg::TargetsPose>(config["global_to_local"]["target_pose_topic"].as<std::string>(), qos_default);
    armworld_pub_  = this->create_publisher<std_msgs::msg::Float32MultiArray>("global2local/world/arm_pose", qos_default);

    // Subscriber
    global_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("mavros/global_position/global", qos_sensor,
                                                                             std::bind(&Global2Local::globalPositionCallback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("mavros/local_position/odom", qos_sensor, std::bind(&Global2Local::odomCallback, this, _1));
    arm_sub_  = this->create_subscription<msg_set_msgs::msg::UsmDataInfo>(config["uav_state_machine"]["usm_info_topic"].as<std::string>(), qos_default,
                                                                         std::bind(&Global2Local::armCallback, this, _1));
    setpoint_sub_ = this->create_subscription<msg_set_msgs::msg::UsmAlgoCmd>(config["global_to_local"]["world_algo_cmd_topic"].as<std::string>(), qos_default,
                                                                             std::bind(&Global2Local::setpointCallback, this, _1));
    origin_gps_sub_ =
        this->create_subscription<sensor_msgs::msg::NavSatFix>("/ui_to_mqtt_ref_gps", qos_default, std::bind(&Global2Local::originGpsCallback, this, _1));

    // xz
    multitask_global_sub_      = this->create_subscription<msg_set_msgs::msg::MultiTask>(config["backend"]["multitask"].as<std::string>(), qos_default,
                                                                                    std::bind(&Global2Local::multitaskCallback, this, _1));
    jumptask_global_sub_       = this->create_subscription<msg_set_msgs::msg::JumpTask>(config["backend"]["jump_task"].as<std::string>(), qos_default,
                                                                                  std::bind(&Global2Local::jumptaskCallback, this, _1));
    returntask_global_sub_     = this->create_subscription<msg_set_msgs::msg::MultiTask>(config["backend"]["return_task"].as<std::string>(), qos_default,
                                                                                     std::bind(&Global2Local::returntaskCallback, this, _1));
    covertask_global_sub_      = this->create_subscription<msg_set_msgs::msg::CoverTask>(config["backend"]["cover_task"].as<std::string>(), qos_default,
                                                                                    std::bind(&Global2Local::covertaskCallback, this, _1));
    coverpath_global_sub_      = this->create_subscription<mavros_msgs::msg::WaypointList>(config["backend"]["cover_path"].as<std::string>(), qos_default,
                                                                                      std::bind(&Global2Local::coverpathCallback, this, _1));
    covergoalpoint_global_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        config["backend"]["cover_goalpoint"].as<std::string>(), qos_default, std::bind(&Global2Local::covergoalpointCallback, this, _1));
    targetpose_body_sub_ = this->create_subscription<msg_set_msgs::msg::ObjectDetection>(config["perception"]["object_detection_topic"].as<std::string>(),
                                                                                         qos_default, std::bind(&Global2Local::targetposeCallback, this, _1));
    usm_state_sub_       = this->create_subscription<std_msgs::msg::Int32>(config["uav_state_machine"]["usm_state_topic"].as<std::string>(), qos_default,
                                                                     std::bind(&Global2Local::usmStateCallback, this, std::placeholders::_1));
    tof_info_sub_        = this->create_subscription<sensor_msgs::msg::Range>("mavros/distance_sensor/hrlv_ez4_pub", qos_sensor,
                                                                       std::bind(&Global2Local::tofInfoCallBack, this, std::placeholders::_1));
}

void Global2Local::originGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    origin_lat_        = msg->latitude;
    origin_lon_        = msg->longitude;
    get_origin_from_ui = true;
    std::cout << "\033[32mref gps from front_end update\033[0m" << std::endl;
}

void Global2Local::tofInfoCallBack(const sensor_msgs::msg::Range::SharedPtr msg)
{
    tof_sensor_hgt_ = msg->range;
    // std::cout << "pitch: " << ypr[1] * 180 / M_PI + 180 << " roll: " << ypr[2] * 180 / M_PI + 180 << " yaw: " << ypr[0] * 180 / M_PI + 180 << std::endl;
    if ((uav_state_ == 2) && (tof_sensor_hgt_ > 0.5) && (tof_sensor_hgt_ < 8.0) && (fabs(ypr[1] * 180 / M_PI + 180) + fabs(ypr[2] * 180 / M_PI + 180)) < 6)
    {
        std::cout << "Calibrate the altitude according to the TOF" << std::endl;
        origin_alt_ = drone_alt_ - tof_sensor_hgt_;
    }
}

void Global2Local::usmStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    uav_state_ = msg->data;
}

void Global2Local::globalPositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    getglobal_flag = true;
    drone_lat_     = msg->latitude;
    drone_lon_     = msg->longitude;
    drone_alt_     = msg->altitude;
    // std::cout << "origin_alt_= " << origin_alt_ << std::endl;
    checkDistance(origin_lat_, origin_lon_, drone_lat_, drone_lon_);
    world_local_position_ =
        local_coordinates(origin_lat_, origin_lon_, origin_alt_, msg->latitude, msg->longitude, msg->altitude, origin_x_, origin_y_, origin_z_);
    if (arm_gps_flage_ && arm_pose_flag_)
    {
        arm_gps_flage_ = false;
        arm_pose_flag_ = false;
        dif_pose_      = world_local_position_ - current_local_position_;
        if (!get_arm_from_usm)
        {
            get_arm_from_usm = true;
            std::cout << "\033[1;32mget armed gps from usm and calculate world pose\033[0m" << std::endl;
        }
    }
    else
    {
        if (!get_arm_from_usm)
        {
            if (!log_out)
            {
                std::cout << "\033[31m"
                          << "cannot get armed gps from usm or calculate world pose"
                          << "\033[0m" << std::endl;
                log_out = true;
            }
        }
    }
}

void Global2Local::setpointCallback(const msg_set_msgs::msg::UsmAlgoCmd::SharedPtr msg)
{
    msg_set_msgs::msg::UsmAlgoCmd modified_msg = *msg;
    modified_msg.pos_target.coordinate_frame   = 1;  // 1: world frame, 2: body frame
    modified_msg.pos_target.position.x -= dif_pose_.x();
    modified_msg.pos_target.position.y -= dif_pose_.y();
    modified_msg.pos_target.position.z -= dif_pose_.z();
    modified_setpoint_pub_->publish(modified_msg);
    // LOG_NODE_VAR_VALUE("/", "/", "/", "/", "/", "/", "/", "/", "/", "/", "/", "/", static_cast<int>(modified_msg.algo_id), modified_msg.algo_hold_state,
    //                    modified_msg.pos_target.type_mask, modified_msg.pos_target.position.x, modified_msg.pos_target.position.y,
    //                    modified_msg.pos_target.position.z, modified_msg.pos_target.velocity.x, modified_msg.pos_target.velocity.y,
    //                    modified_msg.pos_target.velocity.z, modified_msg.pos_target.acceleration_or_force.x, modified_msg.pos_target.acceleration_or_force.y,
    //                    modified_msg.pos_target.acceleration_or_force.z, modified_msg.pos_target.yaw, modified_msg.pos_target.yaw_rate);
}

void Global2Local::armCallback(const msg_set_msgs::msg::UsmDataInfo::SharedPtr msg)
{
    std_msgs::msg::Float32MultiArray msg_out;
    msg_out.data.resize(3);  // 3D 坐标数据

    for (int i = 0; i < 3; ++i)
    {
        arm_pose_[i] = msg->ref_pos[i];
        arm_gps_[i]  = msg->ref_gps[i];
    }
    if (get_origin_from_ui)
    {
        Eigen::Vector3d arm_world =
            local_coordinates(origin_lat_, origin_lon_, origin_alt_, arm_gps_[0], arm_gps_[1], arm_gps_[2], origin_x_, origin_y_, origin_z_);
        msg_out.data[0] = arm_world.x();
        msg_out.data[1] = arm_world.y();
        msg_out.data[2] = arm_world.z();
        armworld_pub_->publish(msg_out);
    }

    if (uav_state_ == 2 && !armed_flag_)
    {
        armed_flag_ = true;
        // std::cout << "get armed gps from usm" << std::endl;
        origin_alt_    = arm_gps_[2];
        arm_pose_flag_ = true;
        arm_gps_flage_ = true;
    }
}

Eigen::Vector3d quaternionToYPR(const geometry_msgs::msg::Quaternion& q_msg)
{
    // Convert quaternion to Yaw-Pitch-Roll (YPR) using Euler angles
    Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    Eigen::Vector3d    ypr = q.toRotationMatrix().eulerAngles(2, 1, 0);  // YPR order

    // Adjust yaw (Y) to be within [-pi, pi]
    if (ypr[0] > M_PI)
        ypr[0] -= 2 * M_PI;
    else if (ypr[0] < -M_PI)
        ypr[0] += 2 * M_PI;

    return ypr;
}

void Global2Local::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    geometry_msgs::msg::Quaternion orientation = msg->pose.pose.orientation;
    // 将四元数转换为旋转矩阵
    tf2::Quaternion quat;
    tf2::fromMsg(orientation, quat);
    tf2::Matrix3x3 rot_matrix(quat);
    // 将速度从机体系转换到惯性系或地面坐标系
    tf2::Vector3 linear_velocity_inertial(
        rot_matrix.getRow(0).dot(tf2::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z)),
        rot_matrix.getRow(1).dot(tf2::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z)),
        rot_matrix.getRow(2).dot(tf2::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z)));

    modified_odom = *msg;
    if (!get_origin_from_ui)
    {
        // std::cout << "cannot get origin from ui" << std::endl;
        return;
    }
    current_local_position_.x()        = modified_odom.pose.pose.position.x;
    current_local_position_.y()        = modified_odom.pose.pose.position.y;
    current_local_position_.z()        = modified_odom.pose.pose.position.z;
    modified_odom.pose.pose.position.x = world_local_position_.x();
    modified_odom.pose.pose.position.y = world_local_position_.y();
    modified_odom.pose.pose.position.z = world_local_position_.z();
    modified_odom.twist.twist.linear.x = linear_velocity_inertial.x();
    modified_odom.twist.twist.linear.y = linear_velocity_inertial.y();
    modified_odom.twist.twist.linear.z = linear_velocity_inertial.z();
    modified_odom_pub_->publish(modified_odom);

    pose_msg.header           = msg->header;
    pose_msg.pose.position.x  = world_local_position_.x();
    pose_msg.pose.position.y  = world_local_position_.y();
    pose_msg.pose.position.z  = world_local_position_.z();
    pose_msg.pose.orientation = msg->pose.pose.orientation;
    ypr                       = quaternionToYPR(pose_msg.pose.orientation);
    if (!getglobal_flag)
    {
        RCLCPP_ERROR(this->get_logger(), "dont get global position");
        return;
    }
    // ~~~~~~~~~~~~~glog~~~~~~~~~~~~~
    // LOG_NODE_VAR_VALUE(drone_lat_, drone_lon_, drone_alt_, modified_odom.pose.pose.position.x, modified_odom.pose.pose.position.y,
    //                    modified_odom.pose.pose.position.z, modified_odom.twist.twist.linear.x, modified_odom.twist.twist.linear.y,
    //                    modified_odom.twist.twist.linear.z, ypr[0], ypr[1], ypr[2], "/", "/", "/", "/", "/", "/", "/", "/", "/", "/", "/", "/");
    // ~~~~~~~~~~~~~glog~~~~~~~~~~~~~
    world_pose_pub_->publish(pose_msg);
}

void Global2Local::checkDistance(double originLat, double originLon, double droneLat, double droneLon)
{
    double latDiff = std::fabs(originLat - droneLat);
    double lonDiff = std::fabs(originLon - droneLon);

    if ((latDiff > 0.1 || lonDiff > 0.1) && !messagePrinted)
    {
        std::cout << "\033[1;31m距离原点过远\033[0m" << std::endl;
        messagePrinted = true;
    }
}

void Global2Local::multitaskCallback(const msg_set_msgs::msg::MultiTask::SharedPtr msg)
{
    std::cout << "received multitask" << std::endl;
    multitask_global = *msg;
    if (!get_origin_from_ui)
    {
        std::cout << "multitask cannot get origin from ui" << std::endl;
        return;
    }
    multitask_local = multitask_global;
    for (auto& single_task : multitask_local.task_stream)
    {
        setAreaSampleGlobalToWorld(single_task.task_area);
        setWaypointListGlobalToWorld(single_task.arrive_path);
    }
    for (auto& single_fence : multitask_local.fence)
    {
        setAreaSampleGlobalToWorld(single_fence);
    }
    for (auto& single_fence : multitask_local.threat_area)
    {
        setAreaSampleGlobalToWorld(single_fence);
    }
    multitask_world_pub_->publish(multitask_local);
}

void Global2Local::jumptaskCallback(const msg_set_msgs::msg::JumpTask::SharedPtr msg)
{
    std::cout << "received jump task" << std::endl;
    jumptask_global = *msg;
    jumptask_local  = jumptask_global;
    setWaypointListGlobalToWorld(jumptask_local.arrive_path);
    jumptask_world_pub_->publish(jumptask_local);
}

void Global2Local::returntaskCallback(const msg_set_msgs::msg::MultiTask::SharedPtr msg)
{
    std::cout << "received return task" << std::endl;
    returntask_global = *msg;
    returntask_local  = returntask_global;
    for (auto& single_task : multitask_local.task_stream)
    {
        setAreaSampleGlobalToWorld(single_task.task_area);
        setWaypointListGlobalToWorld(single_task.arrive_path);
    }
    for (auto& single_fence : multitask_local.fence)
    {
        setAreaSampleGlobalToWorld(single_fence);
    }
    for (auto& single_fence : multitask_local.threat_area)
    {
        setAreaSampleGlobalToWorld(single_fence);
    }
    returntask_world_pub_->publish(returntask_local);
}

void Global2Local::covertaskCallback(const msg_set_msgs::msg::CoverTask::SharedPtr msg)
{
    std::cout << "received cover task" << std::endl;
    covertask_global = *msg;
    covertask_local  = covertask_global;
    setWaypointListGlobalToWorld(covertask_local.path_points);
    covertask_world_pub_->publish(covertask_local);
}

void Global2Local::coverpathCallback(const mavros_msgs::msg::WaypointList::SharedPtr msg)
{
    std::cout << "received coverpath task" << std::endl;
    coverpath_global = *msg;
    coverpath_local  = coverpath_global;
    setWaypointListGlobalToWorld(coverpath_local);
    coverpath_world_pub_->publish(coverpath_local);
}
void Global2Local::covergoalpointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::cout << "received covergoalpoint task" << std::endl;
    goal_point_global = *msg;
    goal_point_local  = goal_point_global;
    setpointGlobalToWorld(goal_point_local);
    covergoalpoint_world_pub_->publish(goal_point_local);
}

void Global2Local::setAreaSampleGlobalToWorld(msg_set_msgs::msg::AreaSample& as)
{
    if (as.type == 0 || as.type == 1 || as.type == 3 || as.type == 4)
    {
        if (as.data.size() < 2)
            return;
        Eigen::Vector3d point_local =
            local_coordinates(origin_lat_, origin_lon_, origin_alt_, as.data[0], as.data[1], origin_alt_, origin_x_, origin_y_, origin_z_);
        as.data[0] = point_local.x();
        as.data[1] = point_local.y();
    }
    else if (as.type == 2)
    {
        int size = as.data.size();
        for (int i = 0; i + 1 < size; i = i + 2)
        {
            Eigen::Vector3d point_local =
                local_coordinates(origin_lat_, origin_lon_, origin_alt_, as.data[i], as.data[i + 1], origin_alt_, origin_x_, origin_y_, origin_z_);
            as.data[i]     = point_local.x();
            as.data[i + 1] = point_local.y();
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "unknown AreaSample type");
    }
}

void Global2Local::setWaypointListGlobalToWorld(mavros_msgs::msg::WaypointList& wl)
{
    for (auto& waypoint : wl.waypoints)
    {
        Eigen::Vector3d waypoints_local =
            local_coordinates(origin_lat_, origin_lon_, origin_alt_, waypoint.x_lat, waypoint.y_long, waypoint.z_alt, origin_x_, origin_y_, origin_z_);
        waypoint.x_lat  = waypoints_local.x();
        waypoint.y_long = waypoints_local.y();
        // z默认为world系
        waypoint.frame = mavros_msgs::msg::Waypoint::FRAME_LOCAL_NED;
    }
}

void Global2Local::setpointGlobalToWorld(std_msgs::msg::Float32MultiArray& fm)
{
    Eigen::Vector3d waypoints_local =
        local_coordinates(origin_lat_, origin_lon_, origin_alt_, fm.data[0], fm.data[1], fm.data[2], origin_x_, origin_y_, origin_z_);
    fm.data[0] = waypoints_local.x();
    fm.data[1] = waypoints_local.y();
    // z默认为world系
}

Eigen::Vector3d Global2Local::local_coordinates(double lat, double lon, double alt, double new_lat, double new_lon, double new_alt, double local_x,
                                                double local_y, double local_z)
{
    Eigen::Vector3d               pos;
    GeographicLib::LocalCartesian geoConverter;
    geoConverter.Reset(lat, lon, alt);
    geoConverter.Forward(new_lat, new_lon, new_alt, pos.x(), pos.y(), pos.z());
    pos.x() += local_x;
    pos.y() += local_y;
    pos.z() += local_z;
    return pos;
}

Eigen::Vector3d Global2Local::local_to_global(double local_x, double local_y, double local_z)
{
    double                        global_lat, global_lon, global_alt;
    GeographicLib::LocalCartesian geoConverter;
    // 设置参考点
    geoConverter.Reset(origin_lat_, origin_lon_, origin_alt_);
    // 从本地坐标转换到全球坐标
    geoConverter.Reverse(local_x, local_y, local_z, global_lat, global_lon, global_alt);

    return Eigen::Vector3d(global_lat, global_lon, global_alt);
}

Eigen::Vector3d Global2Local::calculateTargetPosition(double x, double y, double z, double dx, double dy, double dz, double yaw_rad)
{
    // 计算目标在全局坐标系中的位置 (应用旋转矩阵)
    double target_x = x + dx * cos(yaw_rad) - dy * sin(yaw_rad);
    double target_y = y + dx * sin(yaw_rad) + dy * cos(yaw_rad);
    double target_z = z + dz;  // z坐标不受yaw角影响

    // // 输出目标的全局坐标
    // std::cout << "Target position in world coordinates: (" << target_x << ", " << target_y << ", " << target_z << ")\n";

    return Eigen::Vector3d(target_x, target_y, target_z);  // 确保返回一个 Eigen::Vector3d
}

void Global2Local::targetposeCallback(const msg_set_msgs::msg::ObjectDetection::SharedPtr msg)
{
    std::cout << "received targetpose" << std::endl;
    // 创建 TargetsPose 消息
    msg_set_msgs::msg::TargetsPose targetspose;

    targetspose.header = msg->header;
    // 遍历 ObjectDetection 消息中的 info 数组
    for (const auto& object : msg->info)
    {
        Eigen::Vector3d target_world =
            calculateTargetPosition(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, object.x, object.y, object.z, ypr[0]);
        Eigen::Vector3d target_global = local_to_global(target_world.x(), target_world.y(), target_world.z());

        // 创建 TargetPose 实例
        msg_set_msgs::msg::TargetPose target;
        target.label = object.label;  // 从原始目标提取 label
        target.id    = object.id;     // 从原始目标提取 id
        target.lat   = target_global[0];
        target.lon   = target_global[1];
        target.alt   = target_global[2];
        target.x     = target_world.x();
        target.y     = target_world.y();
        target.z     = target_world.z();
        target.px1   = object.x1;
        target.py1   = object.y1;
        target.px2   = object.x2;
        target.py2   = object.y2;

        // 将目标添加到 TargetsPose 消息中
        targetspose.targets_pose.push_back(target);
    }

    // 此处可以发布 targetgps 消息，或进行其他处理
    // 例如：
    targetgps_pub_->publish(targetspose);
}

int main(int argc, char** argv)
{
    // // ~~~~~~~~~~~~~glog~~~~~~~~~~~~~
    // google::InitGoogleLogging(argv[0]);
    // // ~~~~~~~~~~~~~glog~~~~~~~~~~~~~

    // 初始化 ROS 2 节点
    rclcpp::init(argc, argv);

    // 创建节点对象
    auto global2local = std::make_shared<Global2Local>();

    // 使用多线程执行器来执行节点
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(global2local->get_node_base_interface());
    exe.spin();

    // 关闭 ROS 2 节点
    rclcpp::shutdown();

    // // ~~~~~~~~~~~~~glog~~~~~~~~~~~~~
    // google::ShutdownGoogleLogging();
    // // ~~~~~~~~~~~~~glog~~~~~~~~~~~~~
    return 0;
}
