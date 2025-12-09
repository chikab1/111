#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import JointState

class IdentificationExperiments:
    def __init__(self):
        rospy.init_node('id_experiments')

        # [标准架构] 只发布给 Bridge 的输入接口
        self.pub = rospy.Publisher('/ROS_Car/set_input', JointState, queue_size=1)

        self.rate = rospy.Rate(50) 
        
        rospy.loginfo("物理参数辨识实验脚本已启动...")
        rospy.sleep(1.0) # 等待连接

    def publish_msg(self, inputs):
        """统一封装发布函数"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.name = ['front_left', 'front_right', 'rear_left', 'rear_right']
        msg.velocity = inputs 
        self.pub.publish(msg)

    def warmup(self):
        """通用预热逻辑：发送 3 秒的 0 值，确保数据对齐"""
        warmup_time = 3.0
        rospy.loginfo(f"正在预热 {warmup_time} 秒 (发送 0 PWM)...")
        
        start_warmup = rospy.Time.now()
        while (rospy.Time.now() - start_warmup).to_sec() < warmup_time and not rospy.is_shutdown():
            self.publish_msg([0.0, 0.0, 0.0, 0.0])
            self.rate.sleep()
        rospy.loginfo("预热结束！")

    # =========================================================================
    # 实验 1: 测线阻尼系数 c_v (加速 -> 滑行)
    # 计算公式: c_v = (m * |a_decel|) / v
    # =========================================================================
    def run_linear_drag_test(self):
        rospy.loginfo(">>> 开始实验 1: 线阻尼测试 (全速 -> 滑行) <<<")
        self.warmup()
        
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_time).to_sec()
            
            # 阶段 A: 0-5秒，满油门加速，冲到极速
            if t < 5.0:
                val = 255.0
                rospy.loginfo_throttle(1, f"阶段 A: 全速冲刺 (t={t:.1f})")
            
            # 阶段 B: 5秒后，切断动力 (PWM 0)，测量滑行减速度
            elif t < 10.0:
                val = 0.0
                rospy.loginfo_throttle(1, f"阶段 B: 自由滑行 (t={t:.1f})")
            
            # 实验结束
            else:
                self.publish_msg([0.0, 0.0, 0.0, 0.0])
                rospy.loginfo("实验结束。请停止录制。")
                break
                
            self.publish_msg([val, val, val, val])
            self.rate.sleep()

    # =========================================================================
    # 实验 2: 测推力系数 k_f (静止 -> 阶跃)
    # 计算公式: k_f = (m * a_start) / sum(u)
    # =========================================================================
    def run_thrust_test(self):
        rospy.loginfo(">>> 开始实验 2: 推力系数测试 (静止 -> 阶跃) <<<")
        self.warmup() # 这里的预热本身就是静止阶段
        
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_time).to_sec()
            
            # 阶段 A: 0-2秒，继续保持绝对静止 (再次确认)
            if t < 2.0:
                val = 0.0
                rospy.loginfo_throttle(1, f"阶段 A: 保持静止 (t={t:.1f})")
            
            # 阶段 B: 2-5秒，给固定 PWM 100 (阶跃响应)
            # 注意：如果车太重动不起来，可以把 100 改大
            elif t < 5.0:
                val = 100.0
                rospy.loginfo_throttle(1, f"阶段 B: 阶跃 PWM 100 (t={t:.1f})")
                
            else:
                self.publish_msg([0.0, 0.0, 0.0, 0.0])
                rospy.loginfo("实验结束。")
                break
            
            self.publish_msg([val, val, val, val])
            self.rate.sleep()

    # =========================================================================
    # 实验 3: 测旋转阻尼系数 c_J (原地旋转)
    # 计算公式: c_J = (k_f * 400 * L/2) / w_max
    # =========================================================================
    def run_angular_drag_test(self):
        rospy.loginfo(">>> 开始实验 3: 旋转阻尼测试 (原地打转) <<<")
        self.warmup()
        
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_time).to_sec()
            
            # 持续旋转 8 秒，确保达到最大角速度
            if t < 30.0:
                # 左轮反转(-100)，右轮正转(100) -> 原地逆时针旋转
                # inputs = [左前, 右前, 左后, 右后]
                inputs = [100.0, 200.0, 100.0, 200.0]
                self.publish_msg(inputs)
                rospy.loginfo_throttle(1, f"正在旋转... (t={t:.1f})")
                
            else:
                self.publish_msg([0.0, 0.0, 0.0, 0.0])
                rospy.loginfo("实验结束。")
                break
                
            self.rate.sleep()

    def run(self):
        # ================= [用户选择区域] =================
        # 请每次只解除一行注释来运行对应的实验
        # ================================================
        
        # 1. 测 c_v (先跑这个)
        # self.run_linear_drag_test()

        # 2. 测 k_f (再跑这个)
        # self.run_thrust_test()

        # 3. 测 c_J (最后跑这个，需要用到 k_f)
        self.run_angular_drag_test()

if __name__ == '__main__':
    try:
        node = IdentificationExperiments()
        node.run()
    except rospy.ROSInterruptException:
        pass