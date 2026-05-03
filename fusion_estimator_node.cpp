#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <functional>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> doesn't work on Ubuntu20.04
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "FusionEstimator/fusion_estimator.h"

class FusionEstimatorNode : public rclcpp::Node
{
public:
    FusionEstimatorNode(const rclcpp::NodeOptions &options)
    : Node("fusion_estimator_node", options)
    {        
        /* ────────────── Read ROS2 parameters ────────────── */
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
        this->get_parameter_or("RobotType", robot_type_, std::string("Leg"));

        this->get_parameter_or("imu_data_enable", imu_data_enable, true);
        this->get_parameter_or("leg_pos_enable", leg_pos_enable, true);
        this->get_parameter_or("leg_vel_enable", leg_vel_enable, true);
        this->get_parameter_or("foot_force_threshold", foot_force_threshold, 20.0);
        this->get_parameter_or("min_stair_height", min_stair_height, 0.08);
        this->get_parameter_or("stair_height_fogotten", stair_height_fogotten, 60.0);

        this->get_parameter_or("leg_ori_enable", leg_ori_enable, true);
        this->get_parameter_or("leg_ori_init_weight", leg_ori_init_weight, 0.001);
        this->get_parameter_or("leg_ori_time_wight", leg_ori_time_wight, 100.0);
        this->get_parameter_or("slope_mode_enable", slope_mode_enable, false);
        if(!(leg_ori_init_weight>-1.0 && leg_ori_init_weight<=1.0))
        {
            leg_ori_init_weight = 0.001;
            RCLCPP_INFO(get_logger(), "leg_ori_init_weight to %lf",leg_ori_init_weight);
        }
        if(!(leg_ori_time_wight>0.0 && leg_ori_time_wight<=1000000.0))
        {
            leg_ori_time_wight = 100.0;
            RCLCPP_INFO(get_logger(), "leg_ori_time_wight to %lf", leg_ori_time_wight);
        }


        /* ────────────── Configure estimator status ────────────── */
        double status[200] = {0};
        status[IndexInOrOut] = 1;
        // enable
        status[IndexIMUAccEnable]         = imu_data_enable ? 1.0 : 0.0;
        status[IndexIMUQuaternionEnable]  = imu_data_enable ? 1.0 : 0.0;
        status[IndexIMUGyroEnable]        = imu_data_enable ? 1.0 : 0.0;
        status[IndexJointsXYZEnable]          = leg_pos_enable ? 1.0 : 0.0;
        status[IndexJointsVelocityXYZEnable]  = leg_vel_enable ? 1.0 : 0.0;
        status[IndexJointsRPYEnable]          = leg_ori_enable ? 1.0 : 0.0;
        status[IndexSlopeEstimationEnable]       = slope_mode_enable ? 1.0 : 0.0;
        // Threshold/Weight
        status[IndexLegFootForceThreshold]       = -40.0;
        status[IndexLegMinStairHeight]           = min_stair_height;
        status[IndexStairHeightFogotten]         = stair_height_fogotten;
        status[IndexLegOrientationInitialWeight] = leg_ori_init_weight;
        status[IndexLegOrientationTimeWeight]    = leg_ori_time_wight;


        fe_.fusion_estimator_status(status);
        
        if (robot_type_ == "Wheel")
        {
            status[IndexInOrOut] = 98;
            fe_.fusion_estimator_status(status);
        }
        else{
            status[IndexInOrOut] = 99;
            fe_.fusion_estimator_status(status);
        }

        /* ────────────── Create ROS communication interfaces ────────────── */
        go2_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(sub_imu_topic, 10, std::bind(&FusionEstimatorNode::imu_callback, this, std::placeholders::_1));
        go2_joint_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(sub_joint_topic, 10, std::bind(&FusionEstimatorNode::joint_callback, this, std::placeholders::_1));
        joystick_cmd_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            sub_mode_topic, 10,
            std::bind(&FusionEstimatorNode::joystick_cmd_callback, this, std::placeholders::_1));

        SMXFE_publisher   = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic, 10);
        SMXFE_2D_publisher= this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_2d_topic, 10);

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

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr SMXFE_publisher, SMXFE_2D_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr go2_imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr go2_joint_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_cmd_sub;

    nav_msgs::msg::Odometry SMXFE_odom;
    nav_msgs::msg::Odometry SMXFE_odom_2D;

    std::string odom_frame_id, child_frame_id, child_frame_2d_id, robot_type_;
    bool imu_data_enable, leg_pos_enable, leg_vel_enable, leg_ori_enable, slope_mode_enable;
    bool msg_received[2] = {0,0};
    double foot_force_threshold, min_stair_height, stair_height_fogotten;

    double leg_ori_init_weight, leg_ori_time_wight;

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

        if (robot_type_ == "Wheel")
        {
            // Wheel Type:
            // data[0..15]   : q
            // data[16..31]  : dq
            // data[32..35]  : foot force / contact 
            // or
            // data[32..47]  : motor tau
            if (n != 36 && n != 48)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Wheel mode expects joint array size 36 or 48, but got %zu",
                            n);
                return;
            }

            static const int desired_joints[] = {
                0, 1, 2, 3,
                4, 5, 6, 7,
                8, 9, 10, 11,
                12, 13, 14, 15
            };

            for (int i = 0; i < 16; ++i)
            {
                const int mid = desired_joints[i];

                st_.motorState[mid].q = arr[i];
                st_.motorState[mid].dq = arr[16 + i];

                if (n == 36)
                {
                    const int leg = i / 4;
                    const bool on = (arr[32 + leg] >= foot_force_threshold);
                    st_.motorState[mid].tauEst = on ? 100.0 : 0.0;
                }
                else // n == 48
                {
                    st_.motorState[mid].tauEst = arr[32 + i];
                }
            }
        }
        else
        {
            // Leg Type:
            // data[0..11]   : q
            // data[12..23]  : dq
            // data[24..27]  : foot force / contact
            if (n != 28 && n != 36)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Leg mode expects joint array size 26 or 36, but got %zu",
                            n);
                return;
            }

            static const int desired_joints[] = {
                0, 1, 2,
                4, 5, 6,
                8, 9, 10,
                12, 13, 14
            };

            for (int i = 0; i < 12; ++i)
            {
                const int mid = desired_joints[i];
                st_.motorState[mid].q = arr[i];
                st_.motorState[mid].dq = arr[12 + i];
                
                if (n == 28)
                {
                    const int leg = i / 3;
                    const bool on = (arr[24 + leg] >= foot_force_threshold);
                    st_.motorState[mid].tauEst = on ? 100.0 : 0.0;
                }
                else // n == 36
                {
                    st_.motorState[mid].tauEst = arr[24 + i];
                }
            }
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

    void Msg_Publish()
    {
        if (!msg_received[0] || !msg_received[1])
            return;
        msg_received[0] = 0;
        msg_received[1] = 0;

        // 发布 odometry 消息
        SMXFE_publisher->publish(SMXFE_odom);
        SMXFE_2D_publisher->publish(SMXFE_odom_2D);
    }

    // 回调函数，处理SportCmd消息
    void joystick_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data[0] == 25140000)
        {
            double status[200] = {0};
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