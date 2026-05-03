#pragma once

#include "SensorBase.h"

namespace DataFusion
{
  static constexpr int MAX_CONTACT_CHAIN = 4;
  static constexpr int MAX_CHAIN_NODE = 12;
  static constexpr int MAX_PITCH_SUM_JOINT = 8;

  enum TFJointAxis
  {
    TF_AXIS_FIXED = -1,
    TF_AXIS_X = 0,
    TF_AXIS_Y = 1,
    TF_AXIS_Z = 2
  };

  // ---------------- Joint axis enumeration for kinematic TF nodes ----------------
  // ---------------- 运动学 TF 节点的关节轴枚举 ----------------
  struct TFNode
  {
    // parent node index, -1 means the body frame
    // 父节点编号，-1 表示机体坐标系（body）
    int parent = -1;

    // index of q in joint[0..15], -1 means this node is fixed
    // q 在 joint[0..15] 中的索引，-1 表示该节点没有主动关节
    int q_index = -1;

    // index of dq in joint[16..31], -1 means unavailable
    // dq 在 joint[16..31] 中的索引，-1 表示该节点没有可用角速度输入
    int dq_index = -1;

    // index of tau in joint[32..47], -1 means unavailable
    // tau 在 joint[32..47] 中的索引，-1 表示该节点没有可用力矩输入
    int tau_index = -1;

    // rotation axis of this joint
    // 该关节的转轴方向
    int axis = TF_AXIS_FIXED;

    // fixed translation from parent frame to this node, expressed in parent frame
    // 从父节点到当前节点的固定平移，表达在父节点坐标系下
    double t[3] = {0.0, 0.0, 0.0};

    // fixed installation rotation from parent to this node
    // 从父节点到当前节点的固定安装旋转
    double q_fix[4] = {1.0, 0.0, 0.0, 0.0};

    TFNode() = default;

    TFNode(int parent_, int q_index_, int dq_index_, int tau_index_, int axis_, double x, double y, double z, double roll, double pitch, double yaw)
        : parent(parent_), q_index(q_index_), dq_index(dq_index_), tau_index(tau_index_), axis(axis_)
    {
      t[0] = x; t[1] = y; t[2] = z;
      eulerZYX_to_quat(roll, pitch, yaw, q_fix);
    }
  };

  // ---------------- One contact chain / one leg kinematic description ----------------
  // ---------------- 单条接触链 / 单条腿的运动学描述 ----------------
  struct LegTFChain
  {
    // number of active TF nodes used by this chain
    // 当前链实际使用的 TF 节点数量
    int node_num = 0;

    // TF nodes from body to end-effector
    // 从机体到末端的 TF 节点序列
    TFNode node[MAX_CHAIN_NODE];

    // fixed transform from the last joint node to the end-effector / contact point
    // 从最后一个关节节点到末端 / 接触点的固定变换
    TFNode ee;

    // wheel radius for wheel-foot models
    // 轮足模型的轮半径
    double wheel_radius = 0.0;

    // wheel angle index in joint[0..15]
    // 轮电机角度在 joint[0..15] 中的索引
    int wheel_q_index = -1;

    // wheel velocity index in joint[16..31]
    // 轮电机角速度在 joint[16..31] 中的索引
    int wheel_dq_index = -1;

    // number of pitch-related joints used for wheel swing compensation
    // 用于轮滚动摆角补偿的 pitch 相关关节数量
    int pitch_joint_num = 0;

    int pitch_q_index[MAX_PITCH_SUM_JOINT]  = {-1,-1,-1,-1,-1,-1,-1,-1};
    int pitch_dq_index[MAX_PITCH_SUM_JOINT] = {-1,-1,-1,-1,-1,-1,-1,-1};
  };

  class SensorLegsPos : public Sensors
  {
    public:

    SensorLegsPos(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
    {
      for(int i=0;i<MAX_CONTACT_CHAIN;i++)
      {
        FootIsOnGround[i] = 1;
        FootWasOnGround[i] = 1;
        FootLanding[i] = 0;
        FootLastMotion[i] = 1;
      }

      UseGo2P();
    }
        
    ~SensorLegsPos() override = default;

    int ContactChainNum = 4;
    LegTFChain LegChains_[MAX_CONTACT_CHAIN];
    double FootEffortThreshold = -80.0, Environement_Height_Scope = 0.08, Data_Fading_Time = 1200.0;

    // ---------------- Go2 point-foot preset ----------------
    // ---------------- Go2 点足参数预设 ----------------
    /*
    * This preset describes a 3-DOF point-foot leg:
    *   node[0] : body -> q1 origin, with q1 mounted on this node
    *   node[1] : q1   -> q2 origin, with q2 mounted on this node
    *   node[2] : q2   -> q3 origin, with q3 mounted on this node
    *   ee      : q3   -> fixed foot contact point
    *
    * For point-foot models:
    * 1) wheel_radius = 0
    * 2) ee.z stores the full effective distal length
    * 3) no wheel compensation is used
    *
    * 本预设描述一个三自由度点足腿：
    *   node[0] : body -> q1 髋关节电机原点，并在该节点挂 q1
    *   node[1] : q1   -> q2 大腿关节电机原点，并在该节点挂 q2
    *   node[2] : q2   -> q3 小腿关节电机原点，并在该节点挂 q3
    *   ee      : q3   -> 固定足端接触点
    *
    * 对于点足模型：
    * 1）wheel_radius = 0
    * 2）ee.z 直接表示末端总有效长度
    * 3）不进行轮滚动补偿
    */
    void UseGo2P()
    {  
      for (int i = 0; i < MAX_CONTACT_CHAIN; ++i) LegChains_[i] = LegTFChain();

      ContactChainNum = 4;
      Environement_Height_Scope = 0.08;
      FootEffortThreshold = -1.0;

      // FL: q=(0,1,2), dq=(16,17,18), tau=(32,33,34)
      // q1 is fixed on body (0.1934, -0.0465, 0)
      // q1 motor rotate along axis X，q2/q3 ~ axis Y
      // q1->q2  Y = +0.0955
      // q2->q3  hip lenght = 0.213
      // q3->foot shank and foot length = 0.235 = 0.213 + 0.022
      LegChains_[0].node_num = 3;
      // SensorDataHandle(double* Message, double Time)----Message[0 16 32]分别是一号电机的角度、角速度、力矩
      LegChains_[0].node[0] = TFNode(-1,  0, 16, 32, TF_AXIS_X,  0.1934, -0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[1] = TFNode( 0,  1, 17, 33, TF_AXIS_Y,  0.0000,  0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[2] = TFNode( 1,  2, 18, 34, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[0].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2350, 0.0, 0.0, 0.0);
      LegChains_[0].wheel_radius = 0.0;
      LegChains_[0].wheel_q_index = -1;
      LegChains_[0].wheel_dq_index = -1;
      LegChains_[0].pitch_joint_num = 0;
      // FR
      LegChains_[1].node_num = 3;
      LegChains_[1].node[0] = TFNode(-1,  4, 20, 36, TF_AXIS_X,  0.1934,  0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[1] = TFNode( 0,  5, 21, 37, TF_AXIS_Y,  0.0000, -0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[2] = TFNode( 1,  6, 22, 38, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[1].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2350, 0.0, 0.0, 0.0);
      LegChains_[1].wheel_radius = 0.0;
      LegChains_[1].wheel_q_index = -1;
      LegChains_[1].wheel_dq_index = -1;
      LegChains_[1].pitch_joint_num = 0;
      // RL
      LegChains_[2].node_num = 3;
      LegChains_[2].node[0] = TFNode(-1,  8, 24, 40, TF_AXIS_X, -0.1934, -0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[1] = TFNode( 0,  9, 25, 41, TF_AXIS_Y,  0.0000,  0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[2] = TFNode( 1, 10, 26, 42, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[2].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2350, 0.0, 0.0, 0.0);
      LegChains_[2].wheel_radius = 0.0;
      LegChains_[2].wheel_q_index = -1;
      LegChains_[2].wheel_dq_index = -1;
      LegChains_[2].pitch_joint_num = 0;
      // RR
      LegChains_[3].node_num = 3;
      LegChains_[3].node[0] = TFNode(-1, 12, 28, 44, TF_AXIS_X, -0.1934,  0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[1] = TFNode( 0, 13, 29, 45, TF_AXIS_Y,  0.0000, -0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[2] = TFNode( 1, 14, 30, 46, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[3].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2350, 0.0, 0.0, 0.0);
      LegChains_[3].wheel_radius = 0.0;
      LegChains_[3].wheel_q_index = -1;
      LegChains_[3].wheel_dq_index = -1;
      LegChains_[3].pitch_joint_num = 0;
    }

    void UseMP()
    {
      for (int i = 0; i < MAX_CONTACT_CHAIN; ++i) LegChains_[i] = LegTFChain();

      ContactChainNum = 4;
      Environement_Height_Scope = 0.08;
      FootEffortThreshold = -80.0;

      LegChains_[0].node_num = 3;
      LegChains_[0].node[0] = TFNode(-1,  0, 16, 32, TF_AXIS_X,  0.2878,  0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[1] = TFNode( 0,  1, 17, 33, TF_AXIS_Y,  0.0000,  0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[2] = TFNode( 1,  2, 18, 34, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[0].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2900, 0.0, 0.0, 0.0);
      LegChains_[0].wheel_radius = 0.0;
      LegChains_[0].wheel_q_index = -1;
      LegChains_[0].wheel_dq_index = -1;
      LegChains_[0].pitch_joint_num = 0;

      LegChains_[1].node_num = 3;
      LegChains_[1].node[0] = TFNode(-1,  4, 20, 36, TF_AXIS_X,  0.2878, -0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[1] = TFNode( 0,  5, 21, 37, TF_AXIS_Y,  0.0000, -0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[2] = TFNode( 1,  6, 22, 38, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[1].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2900, 0.0, 0.0, 0.0);
      LegChains_[1].wheel_radius = 0.0;
      LegChains_[1].wheel_q_index = -1;
      LegChains_[1].wheel_dq_index = -1;
      LegChains_[1].pitch_joint_num = 0;

      LegChains_[2].node_num = 3;
      LegChains_[2].node[0] = TFNode(-1,  8, 24, 40, TF_AXIS_X, -0.2878,  0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[1] = TFNode( 0,  9, 25, 41, TF_AXIS_Y,  0.0000,  0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[2] = TFNode( 1, 10, 26, 42, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[2].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2900, 0.0, 0.0, 0.0);
      LegChains_[2].wheel_radius = 0.0;
      LegChains_[2].wheel_q_index = -1;
      LegChains_[2].wheel_dq_index = -1;
      LegChains_[2].pitch_joint_num = 0;

      LegChains_[3].node_num = 3;
      LegChains_[3].node[0] = TFNode(-1, 12, 28, 44, TF_AXIS_X, -0.2878, -0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[1] = TFNode( 0, 13, 29, 45, TF_AXIS_Y,  0.0000, -0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[2] = TFNode( 1, 14, 30, 46, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[3].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2900, 0.0, 0.0, 0.0);
      LegChains_[3].wheel_radius = 0.0;
      LegChains_[3].wheel_q_index = -1;
      LegChains_[3].wheel_dq_index = -1;
      LegChains_[3].pitch_joint_num = 0;
    }
    
    // ---------------- LW wheel-foot preset ----------------
    // ---------------- LW 轮足参数预设 ----------------
    /*
    * This preset describes a 3-DOF leg plus one wheel motor.
    * The kinematic chain itself still uses q1/q2/q3.
    * Wheel rolling displacement is NOT injected into forward kinematics directly.
    * Instead, wheel radius and wheel angle increments are used later in
    * FootFallPositionRecord() to compensate rolling motion along the support direction.
    *
    * pitch_joint_num / pitch_q_index / pitch_dq_index are used to remove the
    * apparent wheel rotation caused by shank swing.
    *
    * 本预设描述一个三自由度腿加一个轮电机的轮足模型。
    * 运动学链本身仍然只使用 q1/q2/q3。
    * 轮滚动位移不会直接写进前向运动学，而是在 FootFallPositionRecord() 中
    * 利用轮半径和轮角增量，对支撑方向上的滚动位移进行补偿。
    *
    * pitch_joint_num / pitch_q_index / pitch_dq_index
    * 用于扣除小腿摆动造成的“轮子假转动”。
    */
    void UseLW()
    {
      for (int i = 0; i < MAX_CONTACT_CHAIN; ++i) LegChains_[i] = LegTFChain();

      ContactChainNum = 4;
      Environement_Height_Scope = 0.10;
      FootEffortThreshold = -120.0;

      // FL
      LegChains_[0].node_num = 3;
      LegChains_[0].node[0] = TFNode(-1,  0, 16, 32, TF_AXIS_X,  0.3405,  0.1000, -0.0666, 0.0, 0.0, 0.0);
      LegChains_[0].node[1] = TFNode( 0,  1, 17, 33, TF_AXIS_Y,  0.0000,  0.1522,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[2] = TFNode( 1,  2, 18, 34, TF_AXIS_Y,  0.0000,  0.0000, -0.2700, 0.0, 0.0, 0.0);
      LegChains_[0].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.3510, 0.0, 0.0, 0.0);
      LegChains_[0].wheel_radius = 0.195 / 2.0;
      LegChains_[0].wheel_q_index = 3;
      LegChains_[0].wheel_dq_index = 19;
      LegChains_[0].pitch_joint_num = 2;
      LegChains_[0].pitch_q_index[0] = 1;
      LegChains_[0].pitch_q_index[1] = 2;
      LegChains_[0].pitch_dq_index[0] = 17;
      LegChains_[0].pitch_dq_index[1] = 18;
      // FR
      LegChains_[1].node_num = 3;
      LegChains_[1].node[0] = TFNode(-1,  4, 20, 36, TF_AXIS_X,  0.3405, -0.1000, -0.0666, 0.0, 0.0, 0.0);
      LegChains_[1].node[1] = TFNode( 0,  5, 21, 37, TF_AXIS_Y,  0.0000, -0.1522,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[2] = TFNode( 1,  6, 22, 38, TF_AXIS_Y,  0.0000,  0.0000, -0.2700, 0.0, 0.0, 0.0);
      LegChains_[1].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.3510, 0.0, 0.0, 0.0);
      LegChains_[1].wheel_radius = 0.195 / 2.0;
      LegChains_[1].wheel_q_index = 7;
      LegChains_[1].wheel_dq_index = 23;
      LegChains_[1].pitch_joint_num = 2;
      LegChains_[1].pitch_q_index[0] = 5;
      LegChains_[1].pitch_q_index[1] = 6;
      LegChains_[1].pitch_dq_index[0] = 21;
      LegChains_[1].pitch_dq_index[1] = 22;
      // RL
      LegChains_[2].node_num = 3;
      LegChains_[2].node[0] = TFNode(-1,  8, 24, 40, TF_AXIS_X, -0.3405,  0.1000, -0.0666, 0.0, 0.0, 0.0);
      LegChains_[2].node[1] = TFNode( 0,  9, 25, 41, TF_AXIS_Y,  0.0000,  0.1522,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[2] = TFNode( 1, 10, 26, 42, TF_AXIS_Y,  0.0000,  0.0000, -0.2700, 0.0, 0.0, 0.0);
      LegChains_[2].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.3510, 0.0, 0.0, 0.0);
      LegChains_[2].wheel_radius = 0.195 / 2.0;
      LegChains_[2].wheel_q_index = 11;
      LegChains_[2].wheel_dq_index = 27;
      LegChains_[2].pitch_joint_num = 2;
      LegChains_[2].pitch_q_index[0] = 9;
      LegChains_[2].pitch_q_index[1] = 10;
      LegChains_[2].pitch_dq_index[0] = 25;
      LegChains_[2].pitch_dq_index[1] = 26;
      // RR
      LegChains_[3].node_num = 3;
      LegChains_[3].node[0] = TFNode(-1, 12, 28, 44, TF_AXIS_X, -0.3405, -0.1000, -0.0666, 0.0, 0.0, 0.0);
      LegChains_[3].node[1] = TFNode( 0, 13, 29, 45, TF_AXIS_Y,  0.0000, -0.1522,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[2] = TFNode( 1, 14, 30, 46, TF_AXIS_Y,  0.0000,  0.0000, -0.2700, 0.0, 0.0, 0.0);
      LegChains_[3].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.3510, 0.0, 0.0, 0.0);
      LegChains_[3].wheel_radius = 0.195 / 2.0;
      LegChains_[3].wheel_q_index = 15;
      LegChains_[3].wheel_dq_index = 31;
      LegChains_[3].pitch_joint_num = 2;
      LegChains_[3].pitch_q_index[0] = 13;
      LegChains_[3].pitch_q_index[1] = 14;
      LegChains_[3].pitch_dq_index[0] = 29;
      LegChains_[3].pitch_dq_index[1] = 30;
    }
    
    void UseMW()
    {
      for (int i = 0; i < MAX_CONTACT_CHAIN; ++i) LegChains_[i] = LegTFChain();

      ContactChainNum = 4;
      Environement_Height_Scope = 0.05;
      FootEffortThreshold = -85.0;

      LegChains_[0].node_num = 3;
      LegChains_[0].node[0] = TFNode(-1,  0, 16, 32, TF_AXIS_X,  0.2878,  0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[1] = TFNode( 0,  1, 17, 33, TF_AXIS_Y,  0.0000,  0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[2] = TFNode( 1,  2, 18, 34, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[0].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[0].wheel_radius = 0.195 / 2.0;
      LegChains_[0].wheel_q_index = 3;
      LegChains_[0].wheel_dq_index = 19;
      LegChains_[0].pitch_joint_num = 2;
      LegChains_[0].pitch_q_index[0] = 1;
      LegChains_[0].pitch_q_index[1] = 2;
      LegChains_[0].pitch_dq_index[0] = 17;
      LegChains_[0].pitch_dq_index[1] = 18;

      LegChains_[1].node_num = 3;
      LegChains_[1].node[0] = TFNode(-1,  4, 20, 36, TF_AXIS_X,  0.2878, -0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[1] = TFNode( 0,  5, 21, 37, TF_AXIS_Y,  0.0000, -0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[2] = TFNode( 1,  6, 22, 38, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[1].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[1].wheel_radius = 0.195 / 2.0;
      LegChains_[1].wheel_q_index = 7;
      LegChains_[1].wheel_dq_index = 23;
      LegChains_[1].pitch_joint_num = 2;
      LegChains_[1].pitch_q_index[0] = 5;
      LegChains_[1].pitch_q_index[1] = 6;
      LegChains_[1].pitch_dq_index[0] = 21;
      LegChains_[1].pitch_dq_index[1] = 22;

      LegChains_[2].node_num = 3;
      LegChains_[2].node[0] = TFNode(-1,  8, 24, 40, TF_AXIS_X, -0.2878,  0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[1] = TFNode( 0,  9, 25, 41, TF_AXIS_Y,  0.0000,  0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[2] = TFNode( 1, 10, 26, 42, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[2].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[2].wheel_radius = 0.195 / 2.0;
      LegChains_[2].wheel_q_index = 11;
      LegChains_[2].wheel_dq_index = 27;
      LegChains_[2].pitch_joint_num = 2;
      LegChains_[2].pitch_q_index[0] = 9;
      LegChains_[2].pitch_q_index[1] = 10;
      LegChains_[2].pitch_dq_index[0] = 25;
      LegChains_[2].pitch_dq_index[1] = 26;

      LegChains_[3].node_num = 3;
      LegChains_[3].node[0] = TFNode(-1, 12, 28, 44, TF_AXIS_X, -0.2878, -0.0700,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[1] = TFNode( 0, 13, 29, 45, TF_AXIS_Y,  0.0000, -0.1709,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[2] = TFNode( 1, 14, 30, 46, TF_AXIS_Y,  0.0000,  0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[3].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2600, 0.0, 0.0, 0.0);
      LegChains_[3].wheel_radius = 0.195 / 2.0;
      LegChains_[3].wheel_q_index = 15;
      LegChains_[3].wheel_dq_index = 31;
      LegChains_[3].pitch_joint_num = 2;
      LegChains_[3].pitch_q_index[0] = 13;
      LegChains_[3].pitch_q_index[1] = 14;
      LegChains_[3].pitch_dq_index[0] = 29;
      LegChains_[3].pitch_dq_index[1] = 30;
    }
    
    void UseGo2W()
    {
      for (int i = 0; i < MAX_CONTACT_CHAIN; ++i) LegChains_[i] = LegTFChain();

      ContactChainNum = 4;
      Environement_Height_Scope = 0.08;
      FootEffortThreshold = -30.0;
      SlopeModeTimeThreshold  = 0.25;
      SlopeModeAngleThreshold = 15.0 / 180.0 * M_PI;
      SlopeModeStepHeightThreshold  = 0.08;
      SlopeModeFootForceAccept  = 0.2;

      // FL
      LegChains_[0].node_num = 3;
      LegChains_[0].node[0] = TFNode(-1,  0, 16, 32, TF_AXIS_X,  0.1934, -0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[1] = TFNode( 0,  1, 17, 33, TF_AXIS_Y,  0.0000,  0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[0].node[2] = TFNode( 1,  2, 18, 34, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[0].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[0].wheel_radius = 0.172 / 2.0;
      LegChains_[0].wheel_q_index = 3;
      LegChains_[0].wheel_dq_index = 19;
      LegChains_[0].pitch_joint_num = 2;
      LegChains_[0].pitch_q_index[0] = 1;
      LegChains_[0].pitch_q_index[1] = 2;
      LegChains_[0].pitch_dq_index[0] = 17;
      LegChains_[0].pitch_dq_index[1] = 18;
      // FR
      LegChains_[1].node_num = 3;
      LegChains_[1].node[0] = TFNode(-1,  4, 20, 36, TF_AXIS_X,  0.1934,  0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[1] = TFNode( 0,  5, 21, 37, TF_AXIS_Y,  0.0000, -0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[1].node[2] = TFNode( 1,  6, 22, 38, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[1].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[1].wheel_radius = 0.172 / 2.0;
      LegChains_[1].wheel_q_index = 7;
      LegChains_[1].wheel_dq_index = 23;
      LegChains_[1].pitch_joint_num = 2;
      LegChains_[1].pitch_q_index[0] = 5;
      LegChains_[1].pitch_q_index[1] = 6;
      LegChains_[1].pitch_dq_index[0] = 21;
      LegChains_[1].pitch_dq_index[1] = 22;
      // RL
      LegChains_[2].node_num = 3;
      LegChains_[2].node[0] = TFNode(-1,  8, 24, 40, TF_AXIS_X, -0.1934, -0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[1] = TFNode( 0,  9, 25, 41, TF_AXIS_Y,  0.0000,  0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[2].node[2] = TFNode( 1, 10, 26, 42, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[2].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[2].wheel_radius = 0.172 / 2.0;
      LegChains_[2].wheel_q_index = 11;
      LegChains_[2].wheel_dq_index = 27;
      LegChains_[2].pitch_joint_num = 2;
      LegChains_[2].pitch_q_index[0] = 9;
      LegChains_[2].pitch_q_index[1] = 10;
      LegChains_[2].pitch_dq_index[0] = 25;
      LegChains_[2].pitch_dq_index[1] = 26;
      // RR
      LegChains_[3].node_num = 3;
      LegChains_[3].node[0] = TFNode(-1, 12, 28, 44, TF_AXIS_X, -0.1934,  0.0465,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[1] = TFNode( 0, 13, 29, 45, TF_AXIS_Y,  0.0000, -0.0955,  0.0000, 0.0, 0.0, 0.0);
      LegChains_[3].node[2] = TFNode( 1, 14, 30, 46, TF_AXIS_Y,  0.0000,  0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[3].ee      = TFNode( 2, -1, -1, -1, TF_AXIS_FIXED, 0.0000, 0.0000, -0.2130, 0.0, 0.0, 0.0);
      LegChains_[3].wheel_radius = 0.172 / 2.0;
      LegChains_[3].wheel_q_index = 15;
      LegChains_[3].wheel_dq_index = 31;
      LegChains_[3].pitch_joint_num = 2;
      LegChains_[3].pitch_q_index[0] = 13;
      LegChains_[3].pitch_q_index[1] = 14;
      LegChains_[3].pitch_dq_index[0] = 29;
      LegChains_[3].pitch_dq_index[1] = 30;
    }

    bool JointsXYZEnable = true;
    bool JointsXYZVelocityEnable = true;

    void SensorDataHandle(double* Message, double Time)  override;
    void LoadedWeightCheck(double* Message, double Time);

    bool FootfallPositionRecordIsInitiated[MAX_CONTACT_CHAIN] = {0}, FootIsOnGround[MAX_CONTACT_CHAIN] = {0}, FootWasOnGround[MAX_CONTACT_CHAIN] = {0}, FootLastMotion[MAX_CONTACT_CHAIN] = {0}, FootLanding[MAX_CONTACT_CHAIN] = {0}, CalculateWeightEnable = false;
    double FootBodyEff_BF[MAX_CONTACT_CHAIN][3] = {0}, FootBodyEff_WF[MAX_CONTACT_CHAIN][3] = {0}, FeetEffort2BodyMotion[MAX_CONTACT_CHAIN][6] = {0};
    double FootBodyPos_BF[MAX_CONTACT_CHAIN][3] = {0}, FootBodyPos_WF[MAX_CONTACT_CHAIN][3] = {0}, FootBodyVel_WF[MAX_CONTACT_CHAIN][3] = {0}, FootfallPositionRecord[MAX_CONTACT_CHAIN][4] = {0}, FootfallAveragePosition[3] = {0}, FootfallProbability[MAX_CONTACT_CHAIN] = {0}, WheelAnglePrev[MAX_CONTACT_CHAIN] = {0};

    double MinimumWeight = 25.0, TimelyWeight = 25.0;
    
    bool SlopeModeEnable = true;
    double SlopeModeTimeThreshold  = 1.0;
    double SlopeModeAngleThreshold = 5.0 / 180.0 * M_PI;
    double SlopeModeStepHeightThreshold  = 0.03;
    double SlopeModeFootForceAccept  = 0.5;

    protected:

    void Joint2HipFoot(double *Message, int LegNumber);
    void FootFallPositionRecord(double *Message);
    void FeetEffort2Body(int LegNumber);
    void EstimateGroundPitchAlongHeading(double& move_dir_x, double& move_dir_y, double& move_dir_z);
  };

  class SensorLegsOri : public Sensors
  {
    public:

    SensorLegsOri(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_){}
    void SensorDataHandle(double* Message, double Time) override;

    inline void SetLegsPosRef(class SensorLegsPos* ref){ legs_pos_ref_ = ref; }
    double legori_init_weight = 0.001, legori_time_weight = 1000.0, legori_current_weight = 0.001, legori_correct = 0;

    bool JointsRPYEnable = false;

    protected:

    class SensorLegsPos* legs_pos_ref_ = nullptr;
    
  };

}