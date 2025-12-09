#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray

class Figure8Driver:
    def __init__(self):
        rospy.init_node('figure8_driver_node')
        self.pub = rospy.Publisher('/ROS_Car/motor_rpm', Float32MultiArray, queue_size=1)
        
        # 物理参数
        self.RPM_PER_MPS = 255.0 
        
        self.rate = rospy.Rate(20)
        
        rospy.loginfo("慢速大半径 8字形轨迹发生器已就绪...")
        rospy.sleep(1.0)

    def send_rpm(self, rpm_l, rpm_r):
        msg = Float32MultiArray()
        msg.data = [rpm_l, rpm_r, rpm_l, rpm_r]
        self.pub.publish(msg)

    def run_trajectory(self):
        start_time = rospy.Time.now().to_sec()
        
        # === 参数设置 (慢速版) ===
        # 速度降低到 0.3 m/s (约 76 RPM)
        v_target = 0.3
        
        # 1. 周期延长
        period = 10.0
        
        # 2. 差速减小: 速度慢了，如果要维持大半径，差速也得变小
        # 设为 25 RPM (非常温柔的转向)
        max_diff_rpm = 25.0 
        
        # 总时长设为 2个周期 (80秒)
        duration = period * 2.0 

        rospy.loginfo(f"开始执行: 速度={v_target}m/s, 最大差速={max_diff_rpm}RPM, 周期={period}s")

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            t = now - start_time
            
            if t > duration: break
            
            # 基础 RPM
            base_rpm = v_target * self.RPM_PER_MPS
            
            # 差速控制
            diff = math.sin(2 * math.pi * t / period) * max_diff_rpm
            
            self.send_rpm(base_rpm - diff, base_rpm + diff)
            
            self.rate.sleep()

        # 停车
        self.send_rpm(0, 0)
        rospy.loginfo("8字形测试完成！")

if __name__ == '__main__':
    try:
        Figure8Driver().run_trajectory()
    except rospy.ROSInterruptException:
        pass