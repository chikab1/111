#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist # <-- 核心：新的控制输入类型
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
import numpy as np
import time

# 定义轮子关节名称 (用于数据采集，保持不变)
JOINT_NAMES = [
    'joint_front_right_wheel',
    'joint_front_left_wheel',
    'joint_rear_right_wheel',
    'joint_rear_left_wheel'
]

class PI_TCN_DataGenerator:
    def __init__(self):
        rospy.init_node('pi_tcn_data_generator_node', anonymous=True)
        
        # --- 1. 发布器 (控制输入 U) ---
        # 目标话题：/car_controller/cmd_vel (DiffDriveController 的标准话题)
        self.pub_twist_cmd = rospy.Publisher('/car_controller/cmd_vel', 
                                              Twist, queue_size=1)
        
        # --- 2. 订阅器 (实际状态 X 和加速度 X_dot 的来源) ---
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        rospy.Subscriber('/ROS_Car/imu', Imu, self.imu_callback)
        
        # --- 3. 实验数据存储与状态 ---
        self.last_imu_time = rospy.Time.now()
        self.current_wheel_vel = np.zeros(4)
        
        # 定义一个简单的轨迹（例如：绕圈）
        self.experiment_timer = rospy.Timer(rospy.Duration(0.1), self.experiment_callback)
        self.start_time = rospy.Time.now()

        rospy.loginfo("PI-TCN Data Generator Node ready. Publishing Twist commands.")
        
        # 用于安全保持静止的定时器
        rospy.Timer(rospy.Duration(0.5), self.safety_timer_callback)


    def safety_timer_callback(self, event):
        """如果长时间没有收到命令，则发送零速指令，确保小车静止"""
        # 由于我们使用 DiffDriveController，它能更好地处理零速平衡
        
        # 创建一个 Twist 消息：vx=0, wz=0
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        
        # 仅在需要保持静止时才发布
        if (rospy.Time.now() - self.start_time).to_sec() > 1000: # 1000秒后停止
             self.pub_twist_cmd.publish(stop_cmd)


    def experiment_callback(self, event):
        """生成并发布一个简单的实验轨迹（例如：正弦曲线或绕圈）"""
        
        current_time_sec = (rospy.Time.now() - self.start_time).to_sec()
        
        if current_time_sec < 50.0:
            # 示例轨迹：以 0.5 m/s 前进，并以 0.5 rad/s 绕圈
            vx = 0.5
            wz = 0.5 * np.sin(current_time_sec * 0.5) # 角速度随时间变化
        else:
            # 实验结束，发送静止命令
            vx = 0.0
            wz = 0.0
            self.experiment_timer.shutdown() # 停止实验计时器
            rospy.loginfo("Experiment finished. Sending final stop command.")
        
        twist_cmd = Twist()
        twist_cmd.linear.x = vx
        twist_cmd.angular.z = wz
        
        self.pub_twist_cmd.publish(twist_cmd)


    def joint_state_callback(self, msg):
        """订阅关节状态，获取实际轮子角速度 (X 的来源)"""
        # 提取轮子速度数据。
        # 实际PI-TCN需要的状态：X = [vx, vy, wz, w_wheel_FL, w_wheel_FR, ...]
        
        # 这里是数据采集和记录的起点。你需要将此数据写入 rosbag 或文件。
        
        # wheel_velocities = ... (匹配 JOINT_NAMES 提取速度)
        # self.current_wheel_vel = np.array(wheel_velocities)
        pass


    def imu_callback(self, msg):
        """订阅 IMU 数据 (X_dot 的来源)"""
        # 实际角加速度 X_dot 的主要来源：线加速度 (linear_acceleration) 和 角速度 (angular_velocity)
        
        # 这里是数据采集和记录的起点。你需要将此数据写入 rosbag 或文件。
        
        # linear_accel = msg.linear_acceleration
        # angular_vel = msg.angular_velocity
        pass


if __name__ == '__main__':
    try:
        PI_TCN_DataGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass