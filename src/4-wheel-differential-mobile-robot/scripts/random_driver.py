#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState

class RandomDriver:
    def __init__(self):
        rospy.init_node('random_driver')
        
        self.pub = rospy.Publisher('/ROS_Car/set_input', JointState, queue_size=1)
        
        self.rate = rospy.Rate(50) # 驱动逻辑可以是 50Hz，不影响 Bridge 的 100Hz 输出
        rospy.sleep(1.0) 

    def publish_msg(self, inputs):
        # 依然使用 JointState 格式发送指令，因为 Bridge 的回调是按 JointState 解包的
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        # velocity 字段用来装 PWM 值
        msg.velocity = inputs 
        self.pub.publish(msg)

    def send_speed(self, v_left_ratio, v_right_ratio, duration):
        pwm_left = max(min(v_left_ratio * 255.0, 255.0), -255.0)
        pwm_right = max(min(v_right_ratio * 255.0, 255.0), -255.0)
        
        inputs = [pwm_left, pwm_right, pwm_left, pwm_right]
        start_time = rospy.Time.now()
        
        # 持续发送直到时间结束
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.publish_msg(inputs)
            self.rate.sleep()

    def run(self):
        # 1. 预热阶段 (发 0)
        warmup_time = 3.0
        rospy.loginfo(f"预热 {warmup_time} 秒 (发送 0 PWM)...")
        start_warmup = rospy.Time.now()
        while (rospy.Time.now() - start_warmup).to_sec() < warmup_time:
            self.publish_msg([0.0, 0.0, 0.0, 0.0])
            self.rate.sleep()
            
        rospy.loginfo("开始随机激励...")
        
        # 2. 随机循环
        while not rospy.is_shutdown():
            v_left = np.random.uniform(-1.0, 1.0)
            v_right = np.random.uniform(-1.0, 1.0)
            duration = np.random.uniform(0.5, 2.0)
            
            self.send_speed(v_left, v_right, duration)
            
            # 随机停车
            if np.random.rand() < 0.2: 
                self.send_speed(0, 0, np.random.uniform(0.5, 1.5))

if __name__ == '__main__':
    try:
        driver = RandomDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass