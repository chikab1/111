#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState

class TrajectoryTest:
    def __init__(self):
        rospy.init_node('trajectory_test_node')

        # [修改 1] 只发布给 Bridge 的输入接口
        # Bridge 会负责分发给 Gazebo (控制) 和 Rosbag (录制)
        self.pub = rospy.Publisher('/ROS_Car/set_input', JointState, queue_size=1)

        self.rate = rospy.Rate(50) # 50Hz 发布频率
        rospy.sleep(1.0) # 等待连接建立

    def publish_msg(self, inputs):
        """统一封装发布函数"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.name = ['front_left', 'front_right', 'rear_left', 'rear_right']
        msg.velocity = inputs 
        self.pub.publish(msg)

    def send_speed(self, v_left_ratio, v_right_ratio, duration):
        """
        发送指定速度持续一段时间
        """
        # 将比例 (0~1) 映射到 PWM (0~255)，并限制范围
        pwm_left = max(min(v_left_ratio * 255.0, 255.0), -255.0)
        pwm_right = max(min(v_right_ratio * 255.0, 255.0), -255.0)

        # 四轮映射: [左前, 右前, 左后, 右后]
        inputs = [pwm_left, pwm_right, pwm_left, pwm_right]

        rospy.loginfo(f"执行指令 -> 左轮: {v_left_ratio:.2f} | 右轮: {v_right_ratio:.2f} | 持续: {duration}s")

        start_time = rospy.Time.now()
        
        # 在 duration 时间内循环发布，保持心跳
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.publish_msg(inputs)
            self.rate.sleep()

    def run(self):
        # ==========================================
        # [修改 2] 静默预热期 (Pre-roll)
        # ==========================================
        warmup_time = 3.0
        rospy.loginfo(f"正在预热 {warmup_time} 秒 (发送 0 PWM)...")
        
        start_warmup = rospy.Time.now()
        while (rospy.Time.now() - start_warmup).to_sec() < warmup_time and not rospy.is_shutdown():
            self.publish_msg([0.0, 0.0, 0.0, 0.0])
            self.rate.sleep()
            
        rospy.loginfo("预热结束，开始执行轨迹测试序列...")

        # ================= 阶段 4: 左转测试 (差速) =================
        # 右轮固定，左轮下降
        rospy.loginfo("=== 阶段 4: 左转差速测试 ===")
        # 右轮从 0.9 递减到 0.05
        for v_fixed in np.arange(0.9, 0.05, -0.1): 
            rospy.loginfo(f"--- 子阶段: 右轮固定 {v_fixed:.1f} ---")
            
            # 左轮从比固定值小 0.1 开始递减到 0
            start_v_change = max(v_fixed - 0.1, 0.0)
            
            for v_change in np.arange(start_v_change, -0.05, -0.1):
                if v_change < 0: v_change = 0.0
                
                # 执行动作：左轮慢(v_change)，右轮快(v_fixed) -> 左转
                self.send_speed(v_change, v_fixed, 3.0)
            
            # 每一大组做完，停车休息 2 秒，方便数据分割
            self.send_speed(0, 0, 2.0) 

        # ================= 阶段 5: 右转测试 (差速) =================
        # 左轮固定，右轮下降
        rospy.loginfo("=== 阶段 5: 右转差速测试 ===")
        for v_fixed in np.arange(0.9, 0.05, -0.1):
            rospy.loginfo(f"--- 子阶段: 左轮固定 {v_fixed:.1f} ---")
            
            start_v_change = max(v_fixed - 0.1, 0.0)
            
            for v_change in np.arange(start_v_change, -0.05, -0.1):
                if v_change < 0: v_change = 0.0
                
                # 执行动作：左轮快(v_fixed)，右轮慢(v_change) -> 右转
                self.send_speed(v_fixed, v_change, 3.0)
            
            self.send_speed(0, 0, 2.0)

        rospy.loginfo("所有轨迹测试完成！")
        # 发送最终停止指令
        self.send_speed(0, 0, 1.0)

if __name__ == '__main__':
    try:
        node = TrajectoryTest()
        node.run()
    except rospy.ROSInterruptException:
        pass