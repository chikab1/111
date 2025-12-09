#!/usr/bin/env python3
import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans

class BacksteppingTracker:
    def __init__(self):
        rospy.init_node('backstepping_tracker')

        # ================= 1. 车辆与控制参数 =================
        # [cite_start]轮距 (Track Width): 引用自 pwmcontrol.yaml [cite: 27]
        self.L = 0.215  
        
        # 速度 -> PWM 映射系数 (之前测试得出: 255 PWM ≈ 0.96 m/s)
        self.k_pwm = 255.0 / 0.96  

        # --- 反步法控制增益 (核心调参项) ---
        # c1: x轴误差增益 (控制线速度收敛)
        # c2: y轴误差增益 (控制角速度收敛)
        # c3: 角度误差增益 (控制航向收敛)
        self.c1 = 1.5
        self.c2 = 4.0
        self.c3 = 2.0

        # ================= 2. 轨迹生成参数 (画圆形) =================
        self.radius = 2.0
        self.center = [0.0, 0.0]
        self.ref_speed = 0.5 # 期望线速度 (m/s)
        self.angular_speed = self.ref_speed / self.radius # 期望角速度
        self.start_time = rospy.Time.now().to_sec()

        # ================= 3. ROS 通信 =================
        # 订阅真值里程计 (如果以后有PI-TCN预测的里程计，改这里的话题即可)
        self.sub_odom = rospy.Subscriber('/ROS_Car/odom', Odometry, self.odom_callback)
        self.pub_pwm = rospy.Publisher('/ROS_Car/cmd_pwm', Float32MultiArray, queue_size=1)
        self.pub_ref_path = rospy.Publisher('/ROS_Car/ref_path', Path, queue_size=1, latch=True)

        # 发布一下参考轨迹供 Rviz 显示
        self.publish_reference_path()
        
        rospy.loginfo("反步法控制器已启动 | Gains: c1=%.1f, c2=%.1f, c3=%.1f", self.c1, self.c2, self.c3)

    def publish_reference_path(self):
        """生成并发布参考轨迹的可视化"""
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()
        
        # 生成一圈的点
        for t in np.arange(0, 2*np.pi/self.angular_speed, 0.1):
            x = self.center[0] + self.radius * math.cos(self.angular_speed * t)
            y = self.center[1] + self.radius * math.sin(self.angular_speed * t)
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)
        self.pub_ref_path.publish(path)

    def get_reference_state(self, t):
        """
        获取当前时刻 t 的期望状态 (xr, yr, thetar, vr, wr)
        """
        # 参数方程: x = R*cos(w*t), y = R*sin(w*t)
        angle = self.angular_speed * t
        
        xr = self.center[0] + self.radius * math.cos(angle)
        yr = self.center[1] + self.radius * math.sin(angle)
        
        # 期望航向角 (圆的切线方向)
        thetar = angle + math.pi / 2.0 
        # 归一化到 -pi ~ pi
        thetar = math.atan2(math.sin(thetar), math.cos(thetar))
        
        vr = self.ref_speed
        wr = self.angular_speed
        
        return xr, yr, thetar, vr, wr

    def odom_callback(self, msg):
        # 1. 获取当前状态
        curr_t = rospy.Time.now().to_sec() - self.start_time
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, theta = tf_trans.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 2. 获取期望状态
        xr, yr, thetar, vr, wr = self.get_reference_state(curr_t)

        # 3. 计算误差 (转换到车身坐标系)
        # Matrix form: [xe; ye; theta_e] = [cos sin 0; -sin cos 0; 0 0 1] * [xr-x; yr-y; thetar-theta]
        ex = math.cos(theta) * (xr - x) + math.sin(theta) * (yr - y)
        ey = -math.sin(theta) * (xr - x) + math.cos(theta) * (yr - y)
        etheta = thetar - theta
        # 角度归一化
        etheta = math.atan2(math.sin(etheta), math.cos(etheta))

        # ================= 4. 反步法控制律 (核心) =================
        # 经典的非完整约束移动机器人控制律
        # v = vr * cos(etheta) + c1 * ex
        # w = wr + c2 * vr * ey + c3 * sin(etheta)
        
        v_cmd = vr * math.cos(etheta) + self.c1 * ex
        w_cmd = wr + self.c2 * vr * ey + self.c3 * math.sin(etheta)

        # ================= 5. 差速运动学解算 & PWM 映射 =================
        # v_left = v - w * L/2
        # v_right = v + w * L/2
        vl = v_cmd - w_cmd * (self.L / 2.0)
        vr_wheel = v_cmd + w_cmd * (self.L / 2.0)

        # 转换为 PWM
        pwm_l = vl * self.k_pwm
        pwm_r = vr_wheel * self.k_pwm

        # 6. 下发指令 (增加限幅)
        self.send_pwm(pwm_l, pwm_r)

    def send_pwm(self, l, r):
        # 限幅到 [-255, 255]
        l = max(min(l, 255.0), -255.0)
        r = max(min(r, 255.0), -255.0)
        
        msg = Float32MultiArray()
        msg.data = [l, r, l, r] # 四轮驱动: 左前=左后, 右前=右后
        self.pub_pwm.publish(msg)

if __name__ == '__main__':
    try:
        BacksteppingTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass