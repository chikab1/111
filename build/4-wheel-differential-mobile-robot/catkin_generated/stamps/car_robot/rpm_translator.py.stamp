#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class CommandBridge:
    def __init__(self):
        rospy.init_node('cmd_bridge')
        
        # 1. 内部状态记忆 (只要 Bridge 不关，这个值就一直在)
        self.current_pwm = [0.0, 0.0, 0.0, 0.0]
        
        # 2. 订阅 "设置指令" (来自 random_driver 或其他脚本)
        # 话题名加上命名空间前缀，防止迷路
        self.sub = rospy.Subscriber('/ROS_Car/set_input', JointState, self.callback)
        
        # 3. 发布 "持久化数据" (给 rosbag 录制用)
        self.pub_record = rospy.Publisher('/ROS_Car/motor_rpm', JointState, queue_size=1)
        
        # 4. 发布 "控制指令" (给 Gazebo 底层用)
        self.pub_control = rospy.Publisher('/ROS_Car/cmd_pwm', Float32MultiArray, queue_size=1)
        
        # 5. 100Hz 持续心跳发布 (和您的 process_data 采样率对齐)
        self.rate = rospy.Rate(100)
        

    def callback(self, msg):
        """收到外部指令，更新内部记忆"""
        try:
            if len(msg.velocity) >= 4:
                self.current_pwm = list(msg.velocity)
        except Exception as e:
            rospy.logwarn(f"Input Error: {e}")

    def run(self):
        """主循环：不断复读当前状态"""
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # --- 1. 发布给 rosbag (保持 motor_rpm 不断) ---
            rec_msg = JointState()
            rec_msg.header.stamp = now # 实时更新时间戳
            rec_msg.header.frame_id = "base_link"
            rec_msg.name = ['front_left', 'front_right', 'rear_left', 'rear_right']
            rec_msg.velocity = self.current_pwm # 使用记忆中的值
            self.pub_record.publish(rec_msg)

            # --- 2. 发布给 Gazebo (保持控制不断) ---
            ctrl_msg = Float32MultiArray()
            ctrl_msg.data = self.current_pwm
            self.pub_control.publish(ctrl_msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # [修复] 必须把实例赋值给变量，才能调用 run
        node = CommandBridge()
        node.run()
    except rospy.ROSInterruptException:
        pass