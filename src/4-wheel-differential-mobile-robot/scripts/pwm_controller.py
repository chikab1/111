#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float64

class PureTorqueController:
    def __init__(self):
        rospy.init_node('pure_torque_controller')
        
        self.pwm_max = 255.0
        self.torque_constant = 0.35  # 力矩系数
        self.dead_zone = 10          # 死区
        
        self.current_pwm = [0.0, 0.0, 0.0, 0.0]
        
        # --- 修正点 2: 分离指令话题 ---
        # 订阅: /ROS_Car/cmd_pwm (来自信号发生器)
        # 发布: /ROS_Car/motor_pwm (反馈给数据记录器，但这其实由信号发生器记录更好)
        rospy.Subscriber('/ROS_Car/cmd_pwm', Float32MultiArray, self.pwm_callback)
        
        self.fl_pub = rospy.Publisher('/front_left_effort_controller/command', Float64, queue_size=1)
        self.fr_pub = rospy.Publisher('/front_right_effort_controller/command', Float64, queue_size=1)
        self.rl_pub = rospy.Publisher('/rear_left_effort_controller/command', Float64, queue_size=1)
        self.rr_pub = rospy.Publisher('/rear_right_effort_controller/command', Float64, queue_size=1)
        
        rospy.loginfo("控制器已启动 | 力矩系数: %.2f", self.torque_constant)

    def pwm_to_torque(self, pwm):
        if abs(pwm) < self.dead_zone:
            return 0.0
        # 简单的线性映射： PWM/255 * 系数
        return (pwm / self.pwm_max) * self.torque_constant
        
    def pwm_callback(self, msg):
        if len(msg.data) == 4:
            # 接收指令 -> 转换为力矩 -> 立即发送给 Gazebo
            torques = [self.pwm_to_torque(p) for p in msg.data]
            
            self.fl_pub.publish(Float64(torques[0]))
            self.fr_pub.publish(Float64(torques[1]))
            self.rl_pub.publish(Float64(torques[2]))
            self.rr_pub.publish(Float64(torques[3]))
        else:
            rospy.logwarn("PWM指令长度错误")

if __name__ == '__main__':
    try:
        PureTorqueController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass