#!/usr/bin/env python3
import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState

class SCurveDriver:
    def __init__(self):
        rospy.init_node('s_curve_driver_node')
        
        # [修改 1] 只发布给 Bridge 的输入接口
        # Bridge 会负责分发给 Gazebo (控制) 和 Rosbag (录制)
        self.pub = rospy.Publisher('/ROS_Car/set_input', JointState, queue_size=1)
        
        # 映射关系: 假设 1m/s 对应满 PWM (255)
        self.PWM_PER_MPS = 255.0 
        
        self.rate = rospy.Rate(20) # 20Hz
        
        rospy.loginfo("S型轨迹发生器 (Bridge Mode) 已启动...")
        rospy.sleep(1.0)

    def publish_msg(self, inputs):
        """统一封装发布函数"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.name = ['front_left', 'front_right', 'rear_left', 'rear_right']
        msg.velocity = inputs 
        self.pub.publish(msg)

    def send_cmd(self, pwm_l, pwm_r):
        """发送指令辅助函数"""
        # 限制范围 [-255, 255]
        pwm_l = max(min(pwm_l, 255.0), -255.0)
        pwm_r = max(min(pwm_r, 255.0), -255.0)
        
        # 构建 4 轮数据 [左前, 右前, 左后, 右后]
        inputs = [pwm_l, pwm_r, pwm_l, pwm_r]
        self.publish_msg(inputs)

    def run_sine_wave(self, base_speed_mps, turn_intensity, period, duration):
        """执行正弦波 S 弯"""
        start_time = rospy.Time.now().to_sec()
        
        base_pwm = base_speed_mps * self.PWM_PER_MPS
        MAX_DIFF_PWM = 80.0 
        amp_pwm = turn_intensity * MAX_DIFF_PWM 
        
        rospy.loginfo(f"执行S型: 基速={base_speed_mps}m/s, 差速PWM={amp_pwm:.1f}")

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            elapsed = now - start_time
            
            if elapsed > duration:
                break
            
            # 正弦因子
            sine_factor = math.sin(2 * math.pi * (1.0/period) * elapsed)
            
            # 计算左右轮 PWM
            diff = sine_factor * amp_pwm
            current_l = base_pwm - diff
            current_r = base_pwm + diff
            
            self.send_cmd(current_l, current_r)
            self.rate.sleep()
            
        # 阶段结束归零
        self.send_cmd(0, 0)
        rospy.sleep(1.0)

    def run_trajectory(self):
        # ==========================================
        # [修改 2] 静默预热期 (Pre-roll)
        # ==========================================
        warmup_time = 3.0
        rospy.loginfo(f"正在预热 {warmup_time} 秒 (发送 0 PWM)...")
        rospy.loginfo("请确保 rosbag record 已经在运行！")
        
        start_warmup = rospy.Time.now()
        while (rospy.Time.now() - start_warmup).to_sec() < warmup_time and not rospy.is_shutdown():
            self.publish_msg([0.0, 0.0, 0.0, 0.0])
            self.rate.sleep()
            
        rospy.loginfo("预热结束，开始执行完整训练轨迹...")
        
        # ================= 阶段 1: 中速 大波浪 (前进 S弯) =================
        self.run_sine_wave(base_speed_mps=0.4, turn_intensity=0.8, period=8.0, duration=16.0)

        # ================= 阶段 2: 中速 大波浪 (倒车 S弯) =================
        self.run_sine_wave(base_speed_mps=-0.4, turn_intensity=0.8, period=8.0, duration=16.0)

        # ================= 阶段 3: 高速 小碎步 (快速变向) =================
        self.run_sine_wave(base_speed_mps=0.7, turn_intensity=0.6, period=3.0, duration=12.0)
        
        # ================= 阶段 4: 变频变速 S弯 (复杂工况) =================
        rospy.loginfo("执行变频变速 S 型 (Rich Data)...")
        start_t = rospy.Time.now().to_sec()
        duration = 20.0
        
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            t = now - start_t
            if t > duration: break
            
            # 速度线性增加
            v_ratio = t / duration
            v = 0.2 + (0.6 * v_ratio)
            base_pwm = v * self.PWM_PER_MPS
            
            # 频率线性增加
            freq = 0.2 + (0.8 * v_ratio) 
            
            sine_val = math.sin(2 * math.pi * freq * t)
            diff = sine_val * 60.0 
            
            self.send_cmd(base_pwm - diff, base_pwm + diff)
            self.rate.sleep()
            
        self.send_cmd(0, 0)
        rospy.loginfo("所有训练数据采集完成！")

if __name__ == '__main__':
    try:
        driver = SCurveDriver()
        driver.run_trajectory()
    except rospy.ROSInterruptException:
        pass