#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <functional>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> doesn't work on Ubuntu20.04
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "FusionEstimator/fusion_estimator.h"

class FusionEstimatorNode : public rclcpp::Node
{
public:
    FusionEstimatorNode(const rclcpp::NodeOptions &options)
    : Node("fusion_estimator_node", options)
    {        
        /* ────────────── Read ROS2 parameters ────────────── */
        this->get_parameter_or("RobotType", robot_type_id, 99.0);

        std::string sub_imu_topic;
        this->get_parameter_or("sub_imu_topic", sub_imu_topic, std::string("NoYamlRead/Go2IMU"));
        std::string sub_joint_topic;
        this->get_parameter_or("sub_joint_topic", sub_joint_topic, std::string("NoYamlRead/Go2Joint"));
        std::string sub_mode_topic;
        this->get_parameter_or("sub_mode_topic", sub_mode_topic, std::string("NoYamlRead/SportCmd"));
        std::string pub_odom_topic;
        this->get_parameter_or("pub_odom_topic", pub_odom_topic, std::string("NoYamlRead/Odom"));
        std::string pub_odom_2d_topic;
        this->get_parameter_or("pub_odom2d_topic", pub_odom_2d_topic, std::string("NoYamlRead/Odom_2D"));
        this->get_parameter_or("odom_frame", odom_frame_id, std::string("odom"));
        this->get_parameter_or("base_frame", child_frame_id, std::string("base_link"));
        this->get_parameter_or("base_frame_2d", child_frame_2d_id, std::string("base_link_2D"));

        this->get_parameter_or("pub_body_joint_marker_enable", pub_body_joint_marker_enable, false);
        this->get_parameter_or("pub_body_joint_marker_topic", pub_body_joint_marker_topic, std::string("body_joint_markers"));

        this->get_parameter_or("imu_data_enable", imu_data_enable, true);
        this->get_parameter_or("leg_pos_enable", leg_pos_enable, true);
        this->get_parameter_or("leg_vel_enable", leg_vel_enable, true);
        this->get_parameter_or("leg_ori_enable", leg_ori_enable, false);
        this->get_parameter_or("slope_mode_enable", slope_mode_enable, false);
        
        this->get_parameter_or("contact_sensor_threshold", contact_sensor_threshold, 20.0);
        this->get_parameter_or("foot_force_threshold", foot_force_threshold, -30.0);
        this->get_parameter_or("min_stair_height", min_stair_height, 0.08);

        /* ────────────── Configure estimator status ────────────── */
        double status[100] = {0};
        
        status[IndexInOrOut] = robot_type_id;
        fe_.fusion_estimator_status(status);
        
        status[IndexInOrOut] = 2;
        fe_.fusion_estimator_status(status);

        status[IndexInOrOut] = 1;
        // enable
        status[IndexIMUAccEnable]                = imu_data_enable ? 1.0 : 0.0;
        status[IndexIMUQuaternionEnable]         = imu_data_enable ? 1.0 : 0.0;
        status[IndexIMUGyroEnable]               = imu_data_enable ? 1.0 : 0.0;
        status[IndexJointsXYZEnable]             = leg_pos_enable ? 1.0 : 0.0;
        status[IndexJointsVelocityXYZEnable]     = leg_vel_enable ? 1.0 : 0.0;
        status[IndexJointsRPYEnable]             = leg_ori_enable ? 1.0 : 0.0;
        status[IndexSlopeEstimationEnable]       = slope_mode_enable ? 1.0 : 0.0;
        // Threshold/Weight
        status[IndexLegFootForceThreshold]       = foot_force_threshold;
        status[IndexLegMinStairHeight]           = min_stair_height;
        fe_.fusion_estimator_status(status);

        /* ────────────── Create ROS communication interfaces ────────────── */
        go2_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(sub_imu_topic, 10, std::bind(&FusionEstimatorNode::imu_callback, this, std::placeholders::_1));
        go2_joint_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(sub_joint_topic, 10, std::bind(&FusionEstimatorNode::joint_callback, this, std::placeholders::_1));
        joystick_cmd_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            sub_mode_topic, 10,
            std::bind(&FusionEstimatorNode::joystick_cmd_callback, this, std::placeholders::_1));

        SMXFE_publisher   = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic, 10);
        SMXFE_2D_publisher= this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_2d_topic, 10);

        if (pub_body_joint_marker_enable)
            body_joint_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(pub_body_joint_marker_topic, 10);

        SMXFE_odom.header.frame_id    = odom_frame_id;
        SMXFE_odom.child_frame_id     = child_frame_id;

        // 设置位姿（pose）协方差：6x6 矩阵（行优先排列）
        // 初始化全部置零
        for (int i = 0; i < 36; ++i) {
            SMXFE_odom.pose.covariance[i] = 0.0;
        }
        // 位置 (x, y, z) 的协方差设为 0.01
        SMXFE_odom.pose.covariance[0]  = 0.1;   // x
        SMXFE_odom.pose.covariance[7]  = 0.1;   // y
        SMXFE_odom.pose.covariance[14] = 0.1;   // z
        // 姿态（roll, pitch, yaw）的协方差设为 0.0001
        SMXFE_odom.pose.covariance[21] = 0.1; // roll
        SMXFE_odom.pose.covariance[28] = 0.1; // pitch
        SMXFE_odom.pose.covariance[35] = 0.1; // yaw

        // 设置 twist 协方差：6x6 矩阵（行优先排列）
        for (int i = 0; i < 36; ++i) {
            SMXFE_odom.twist.covariance[i] = 0.0;
        }
        // 线速度 (x, y, z) 的协方差设为 0.1
        SMXFE_odom.twist.covariance[0]  = 0.1;   // linear x
        SMXFE_odom.twist.covariance[7]  = 0.1;   // linear y
        SMXFE_odom.twist.covariance[14] = 0.1;   // linear z
        // 角速度 (x, y, z) 的协方差设为 0.01
        SMXFE_odom.twist.covariance[21] = 0.1;  // angular x
        SMXFE_odom.twist.covariance[28] = 0.1;  // angular y
        SMXFE_odom.twist.covariance[35] = 0.1;  // angular z

        SMXFE_odom_2D.header.frame_id = odom_frame_id;
        SMXFE_odom_2D.child_frame_id  = child_frame_2d_id;
        // 设置位姿（pose）协方差：6x6 矩阵（行优先排列）
        // 初始化全部置零
        for (int i = 0; i < 36; ++i) {
            SMXFE_odom_2D.pose.covariance[i] = 0.0;
        }
        // 位置 (x, y, z) 的协方差设为 0.01
        SMXFE_odom_2D.pose.covariance[0]  = 0.1;   // x
        SMXFE_odom_2D.pose.covariance[7]  = 0.1;   // y
        SMXFE_odom_2D.pose.covariance[14] = 0.1;   // z
        // 姿态（roll, pitch, yaw）的协方差设为 0.0001
        SMXFE_odom_2D.pose.covariance[21] = 0.1; // roll
        SMXFE_odom_2D.pose.covariance[28] = 0.1; // pitch
        SMXFE_odom_2D.pose.covariance[35] = 0.1; // yaw

        // 设置 twist 协方差：6x6 矩阵（行优先排列）
        for (int i = 0; i < 36; ++i) {
            SMXFE_odom_2D.twist.covariance[i] = 0.0;
        }
        // 线速度 (x, y, z) 的协方差设为 0.1
        SMXFE_odom_2D.twist.covariance[0]  = 0.1;   // linear x
        SMXFE_odom_2D.twist.covariance[7]  = 0.1;   // linear y
        SMXFE_odom_2D.twist.covariance[14] = 0.1;   // linear z
        // 角速度 (x, y, z) 的协方差设为 0.01
        SMXFE_odom_2D.twist.covariance[21] = 0.1;  // angular x
        SMXFE_odom_2D.twist.covariance[28] = 0.1;  // angular y
        SMXFE_odom_2D.twist.covariance[35] = 0.1;  // angular z
    }

private:
    FusionEstimatorCore fe_;  // 核心融合器（已封装好）
    LowlevelState st_;        // “静态”输入缓存（节点生命周期内一直保留）
    Odometer odom_;           // “静态”输出缓存（仅 joint 回调更新）

    nav_msgs::msg::Odometry SMXFE_odom;
    nav_msgs::msg::Odometry SMXFE_odom_2D;

    std::string odom_frame_id, child_frame_id, child_frame_2d_id, pub_body_joint_marker_topic;

    bool imu_data_enable, leg_pos_enable, leg_vel_enable, leg_ori_enable, slope_mode_enable, pub_body_joint_marker_enable;
    bool msg_received[2] = {0,0};

    double robot_type_id, contact_sensor_threshold, foot_force_threshold, min_stair_height;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr go2_imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr go2_joint_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_cmd_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr SMXFE_publisher, SMXFE_2D_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr body_joint_marker_publisher;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);
        rclcpp::Time now = ros_clock.now();
        rclcpp::Time msg_time(msg->header.stamp, now.get_clock_type());

        if (now > msg_time) SMXFE_odom.header.stamp = now;
        else                SMXFE_odom.header.stamp = msg_time + rclcpp::Duration(0, 1);

        SMXFE_odom_2D.header.stamp = SMXFE_odom.header.stamp;

        st_.imu.timestamp = static_cast<int64_t>(1e3 * msg->header.stamp.sec + 1e-6 * msg->header.stamp.nanosec);

        st_.imu.accelerometer[0] = msg->linear_acceleration.x;
        st_.imu.accelerometer[1] = msg->linear_acceleration.y;
        st_.imu.accelerometer[2] = msg->linear_acceleration.z;

        st_.imu.gyroscope[0] = msg->angular_velocity.x;
        st_.imu.gyroscope[1] = msg->angular_velocity.y;
        st_.imu.gyroscope[2] = msg->angular_velocity.z;

        st_.imu.quaternion[0] = msg->orientation.w;
        st_.imu.quaternion[1] = msg->orientation.x;
        st_.imu.quaternion[2] = msg->orientation.y;
        st_.imu.quaternion[3] = msg->orientation.z;

        msg_received[0] = 1;
    }

    void joint_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (st_.imu.timestamp <= 0.0) return; 
        const auto& arr = msg->data;
        const size_t n = arr.size();

        // Wheel Type:
        // data[0..15]   : q
        // data[16..31]  : dq
        // data[32..47]  : motor tau
        if (n != 48)
        {
            RCLCPP_ERROR(this->get_logger(),"Wheel mode expects joint array size 48, but got %zu",n);
            return;
        }

        for (int i = 0; i < 16; ++i)
        {
            st_.motorState[i].q      = arr[i];
            st_.motorState[i].dq     = arr[16 + i];
            st_.motorState[i].tauEst = arr[32 + i];
        }
        
        for (int leg = 0; leg < 4; ++leg)
        {
            if(contact_sensor_threshold!=0 && st_.motorState[4*leg+0].tauEst==0 && st_.motorState[4*leg+1].tauEst==0 && st_.motorState[4*leg+3].tauEst==0)
                st_.motorState[4*leg+2].tauEst = (st_.motorState[4*leg+2].tauEst >= contact_sensor_threshold) ? 100.0 : 0.0;
        }

        // ---------------- Estimation is triggered only by the joint callback ----------------
        // ---------------- 估计更新只在 joint 回调中触发 ----------------
        odom_ = fe_.fusion_estimator(st_);

        SMXFE_odom.pose.pose.position.x = odom_.XPos;
        SMXFE_odom.pose.pose.position.y = odom_.YPos;
        SMXFE_odom.pose.pose.position.z = odom_.ZPos;

        SMXFE_odom.twist.twist.linear.x = odom_.XVel;
        SMXFE_odom.twist.twist.linear.y = odom_.YVel;
        SMXFE_odom.twist.twist.linear.z = odom_.ZVel;

        tf2::Quaternion q;
        q.setRPY(odom_.RollRad, odom_.PitchRad, odom_.YawRad);
        SMXFE_odom.pose.pose.orientation = tf2::toMsg(q);

        SMXFE_odom.twist.twist.angular.x = odom_.RollVel;
        SMXFE_odom.twist.twist.angular.y = odom_.PitchVel;
        SMXFE_odom.twist.twist.angular.z = odom_.YawVel;

        SMXFE_odom_2D.pose.pose.position.x = odom_.XPos;
        SMXFE_odom_2D.pose.pose.position.y = odom_.YPos;
        SMXFE_odom_2D.pose.pose.position.z = 0;

        SMXFE_odom_2D.twist.twist.linear.x = odom_.XVel;
        SMXFE_odom_2D.twist.twist.linear.y = odom_.YVel;
        SMXFE_odom_2D.twist.twist.linear.z = 0;

        q.setRPY(0, 0, odom_.YawRad);
        SMXFE_odom_2D.pose.pose.orientation = tf2::toMsg(q);

        SMXFE_odom_2D.twist.twist.angular.x = odom_.RollVel;
        SMXFE_odom_2D.twist.twist.angular.y = odom_.PitchVel;
        SMXFE_odom_2D.twist.twist.angular.z = odom_.YawVel;

        msg_received[1] = 1;
        Msg_Publish();
    }

    void BodyJointMarkerPublish()
    {
        double status[100] = {0};
        status[IndexInOrOut] = 2;
        fe_.fusion_estimator_status(status);

        visualization_msgs::msg::MarkerArray markers;

        const char* leg_names[4]  = {"FL", "FR", "RL", "RR"};
        const char* node_names[4] = {"Hip", "Thigh", "Calf", "Foot"};

        for (int leg = 0; leg < 4; ++leg)
        {
            visualization_msgs::msg::Marker line;

            line.header.stamp = SMXFE_odom.header.stamp;
            line.header.frame_id = odom_frame_id;
            line.ns = std::string(leg_names[leg]) + "_LegLine";
            line.id = 100 + leg;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.015;
            line.color.r = 1.0;
            line.color.g = 1.0;
            line.color.b = 1.0;
            line.color.a = 1.0;

            for (int node = 0; node < 4; ++node)
            {
                const int pos_id = 12 + leg * 12 + node * 3;

                const double x = status[20 + pos_id + 0] + odom_.XPos;
                const double y = status[20 + pos_id + 1] + odom_.YPos;
                const double z = status[20 + pos_id + 2] + odom_.ZPos;

                visualization_msgs::msg::Marker marker;

                marker.header.stamp = SMXFE_odom.header.stamp;
                marker.header.frame_id = odom_frame_id;
                marker.ns = std::string(leg_names[leg]) + "_" + node_names[node];
                marker.id = leg * 4 + node;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = x;
                marker.pose.position.y = y;
                marker.pose.position.z = z;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                if (node == 0)
                {
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }
                else if (node == 1)
                {
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }
                else if (node == 2)
                {
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
                }
                else
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                }

                marker.color.a = 1.0;

                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
                line.points.push_back(p);

                markers.markers.push_back(marker);
            }

            markers.markers.push_back(line);
            const int foot_pos_id = 12 + leg * 12 + 3 * 3;
            const int force_id = leg * 3;

            geometry_msgs::msg::Point p0, p1;

            p0.x = status[20 + foot_pos_id + 0] + odom_.XPos;
            p0.y = status[20 + foot_pos_id + 1] + odom_.YPos;
            p0.z = status[20 + foot_pos_id + 2] + odom_.ZPos;

            p1.x = p0.x - status[20 + force_id + 0] / 1000.0;
            p1.y = p0.y - status[20 + force_id + 1] / 1000.0;
            p1.z = p0.z - status[20 + force_id + 2] / 1000.0;

            visualization_msgs::msg::Marker force_arrow;
            force_arrow.header.stamp = SMXFE_odom.header.stamp;
            force_arrow.header.frame_id = odom_frame_id;
            force_arrow.ns = std::string(leg_names[leg]) + "_Force";
            force_arrow.id = 200 + leg;
            force_arrow.type = visualization_msgs::msg::Marker::ARROW;
            force_arrow.action = visualization_msgs::msg::Marker::ADD;
            force_arrow.pose.orientation.w = 1.0;

            force_arrow.scale.x = 0.01;
            force_arrow.scale.y = 0.03;
            force_arrow.scale.z = 0.03;

            force_arrow.color.r = 1.0;
            force_arrow.color.g = 0.0;
            force_arrow.color.b = 0.0;
            force_arrow.color.a = 1.0;

            force_arrow.points.push_back(p0);
            force_arrow.points.push_back(p1);

            markers.markers.push_back(force_arrow);
        }

        body_joint_marker_publisher->publish(markers);
    }

    void Msg_Publish()
    {
        if (!msg_received[0] || !msg_received[1])
            return;
        msg_received[0] = 0;
        msg_received[1] = 0;

        // 发布 odometry 消息
        SMXFE_publisher->publish(SMXFE_odom);
        SMXFE_2D_publisher->publish(SMXFE_odom_2D);
        if(pub_body_joint_marker_enable)
            BodyJointMarkerPublish();
    }

    // 回调函数，处理SportCmd消息
    void joystick_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data[0] == 25140000)
        {
            double status[100] = {0};
            status[IndexInOrOut] = 3;
            fe_.fusion_estimator_status(status);
        }
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
    .arguments({
        "--ros-args",
        "--params-file", "/home/smx/WorkSpace/GDS_LeggedRobot/src/CAPO-LeggedRobotOdometry/config.yaml"
    });

  auto node = std::make_shared<FusionEstimatorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}