#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import JointState

class SignalGenerator:
    def __init__(self):
        rospy.init_node('signal_generator')

        # [修改] 只发布给 Bridge 的输入接口
        self.pub = rospy.Publisher('/ROS_Car/set_input', JointState, queue_size=1)

        self.rate = rospy.Rate(50) 
        
        rospy.loginfo("正弦波发生器已启动 (Mode: /set_input)")
        rospy.sleep(1.0) # 等待连接

    def get_user_input(self, t):
        # =========== 用户修改区域 ===========
        # 产生平滑的正弦波输入 (-255 到 255)
        val = 255.0 * math.sin(0.5 * t)
        inputs = [val, val, val, val]
        # ==================================
        return inputs

    def publish_msg(self, inputs):
        """统一封装发布函数"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.name = ['front_left', 'front_right', 'rear_left', 'rear_right']
        msg.velocity = inputs 
        self.pub.publish(msg)

    def run(self):
        # ==========================================
        # 1. 静默预热期 (Pre-roll) - 关键！
        # ==========================================
        warmup_time = 3.0
        rospy.loginfo(f"正在预热 {warmup_time} 秒 (发送 0 PWM)...")
        
        start_warmup = rospy.Time.now()
        while (rospy.Time.now() - start_warmup).to_sec() < warmup_time and not rospy.is_shutdown():
            self.publish_msg([0.0, 0.0, 0.0, 0.0])
            self.rate.sleep()
            
        rospy.loginfo("预热结束，开始执行正弦波！")

        # ==========================================
        # 2. 正式开始正弦波
        # ==========================================
        # 重置时间，让正弦波从 0 相位开始 (sin(0)=0)
        self.start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            t = (now - self.start_time).to_sec()

            inputs = self.get_user_input(t)
            self.publish_msg(inputs)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SignalGenerator()
        node.run()
    except rospy.ROSInterruptException:
        pass