#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class PhysicsBridge:
    def __init__(self):
        rospy.init_node('car_physics_bridge')

        # === 1. 你的物理参数 ===
        self.m = 2.45        # 质量 (kg)
        self.J = 0.022       # 转动惯量 (kg*m^2)
        self.L = 0.215       # 左右轮距 (m)
        self.k_f = 0.0001226 # 力常数 (假设输入是 RPM，转换为力的系数)
        self.c_v = 2.4745    # 线性阻尼 (N / (m/s))
        self.c_J = 0.0381    # 旋转阻尼 (N*m / (rad/s))
        
        # === 2. 初始状态 ===
        self.v = 0.0  # 当前线速度 v_x
        self.w = 0.0  # 当前角速度 w_z
        self.dt = 0.01 # 计算频率 100Hz
        
        # 电机输入 [fl, fr, rl, rr] (默认0)
        self.u = np.array([0.0, 0.0, 0.0, 0.0]) 

        # === 3. ROS 通信 ===
        # 发布：告诉 Gazebo 该怎么动
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 订阅：接收你的电机输入 (RPM 或 PWM)
        # 消息格式：Float32MultiArray，顺序 [左前, 右前, 左后, 右后]
        rospy.Subscriber('/ROS_Car/motor_rpm', Float32MultiArray, self.input_callback)
        
        # 定时器：以 100Hz 频率进行物理运算
        rospy.Timer(rospy.Duration(self.dt), self.update_physics)

    def input_callback(self, msg):
        # 接收输入 u (例如 RPM)
        if len(msg.data) == 4:
            self.u = np.array(msg.data)
        else:
            rospy.logwarn("输入数据长度错误，需要 4 个电机的数值")

    def update_physics(self, event):
        # 提取四个轮子的输入
        u_fl, u_fr, u_rl, u_rr = self.u
        
        # === 4. 核心公式实现 ===
        
        # 输入总和 (用于线加速度)
        sum_u = u_fl + u_fr + u_rl + u_rr
        
        # 输入差值 (用于角加速度)
        # 逻辑：左侧轮子驱动正向旋转(右转/逆时针?)，右侧驱动反向
        # 根据你的公式：(u_fl - u_fr + u_rl - u_rr)
        # 这意味着：左边是加项，右边是减项 -> 典型的差速模型
        diff_u = u_fl - u_fr + u_rl - u_rr
        
        # --- 计算加速度 (基于力的平衡) ---
        # v_dot = (Force_motor - Force_friction) / m
        v_dot = ((self.k_f * sum_u) - (self.c_v * self.v)) / self.m
        
        # w_dot = (Torque_motor - Torque_friction) / J
        # 注意：力臂是 L/2 还是 L？你的公式里写的是 (k_f * L / 2J)，说明 L 是全宽
        w_dot = ((self.k_f * self.L * diff_u / 2.0) - (self.c_J * self.w)) / self.J
        
        # --- 积分更新速度 (Euler Integration) ---
        self.v += v_dot * self.dt
        self.w += w_dot * self.dt
        
        # --- 5. 发送给 Gazebo ---
        twist = Twist()
        twist.linear.x = self.v
        twist.angular.z = self.w
        self.pub_cmd.publish(twist)

if __name__ == '__main__':
    try:
        node = PhysicsBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass