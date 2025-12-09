#!/usr/bin/env python3
import rospy
import torch
import torch.nn as nn
import numpy as np
import collections
import pickle
import os
import math

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# === 1. TCN 模型定义 (必须与训练代码一致) ===
from pytorch_tcn import TCN

# 默认物理参数 (会被文件覆盖)
PA, PB, PC, PD = 0.00177, 1.35035, -0.00021, 0.01759 
m, J, L = 2.45, 0.022, 0.215
k_f, c_v, c_J = 0.0001226, 2.4745, 0.0381
DT = 0.01 # 100Hz

class PI_TCN(nn.Module):
    def __init__(self, input_channels=6, output_size=2):
        super(PI_TCN, self).__init__()
        self.tcn = TCN(input_channels, [16]*4, kernel_size=2, dropout=0.1)
        self.mlp = nn.Sequential(
            nn.Linear(16, 64), nn.ReLU(),
            nn.Linear(64, 32), nn.ReLU(),
            nn.Linear(32, 32), nn.ReLU(),
            nn.Linear(32, output_size)
        )
        self.lambda_blend = nn.Parameter(torch.tensor(0.0))

    def forward(self, x_history):
        y_bb = self.mlp(self.tcn(x_history)[:, :, -1])
        return y_bb, torch.sigmoid(self.lambda_blend)

# === 2. 伴飞节点 ===
class LiveInferenceNode:
    def __init__(self):
        rospy.init_node('live_inference')
        
        # --- A. 加载参数 ---
        script_dir = os.path.dirname(os.path.realpath(__file__))
        
        # 读取物理参数
        try:
            with open(os.path.join(script_dir, "physics_params.txt"), "r") as f:
                params = f.read().split(',')
                global PA, PB, PC, PD
                PA, PB, PC, PD = [float(x) for x in params]
            rospy.loginfo(f"物理参数已加载: Av={PA:.5f}, Bv={PB:.5f}...")
        except:
            rospy.logwarn("未找到 physics_params.txt，使用默认参数")

        # --- B. 加载模型 ---
        self.device = torch.device("cpu")
        self.model = PI_TCN().to(self.device)
        
        model_path = os.path.join(script_dir, "pitcn_car_model_paper_arch.pth")
        scaler_path = os.path.join(script_dir, "pitcn_scalers_paper_arch.pkl")
        
        if os.path.exists(model_path):
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            self.model.eval()
            rospy.loginfo("模型权重加载成功!")
        else:
            rospy.logerr("找不到模型文件，请检查路径！")
            return

        with open(scaler_path, 'rb') as f:
            scalers = pickle.load(f)
        self.scaler_hist = scalers['history']
        self.scaler_label = scalers['label']
        
        self.lambda_val = torch.sigmoid(self.model.lambda_blend).item()
        rospy.loginfo(f"当前 Lambda: {self.lambda_val:.4f}")

        # --- C. 状态初始化 ---
        self.seq_len = 20
        self.input_buffer = collections.deque(maxlen=self.seq_len)
        
        # 两个“幽灵”的状态: [x, y, theta, v, w]
        self.state_phys = np.zeros(5)
        self.state_pitcn = np.zeros(5)
        
        self.current_u = [0.0]*4
        self.initialized = False
        self.last_reset_time = rospy.Time.now()
        
        # 自动重置时间 (秒) -> 设置为 0 表示不自动重置，一直跑
        # 设置为 5.0 表示每5秒拉回真实位置一次，方便看局部精度
        self.reset_interval = 10.0 
        
        # --- D. ROS 通信 ---
        rospy.Subscriber('/ROS_Car/motor_rpm', Float32MultiArray, self.motor_cb)
        rospy.Subscriber('/ROS_Car/odom', Odometry, self.odom_cb)
        
        self.pub_real = rospy.Publisher('/path_real', Path, queue_size=10)
        self.pub_phys = rospy.Publisher('/path_phys', Path, queue_size=10)
        self.pub_pitcn = rospy.Publisher('/path_pitcn', Path, queue_size=10)
        
        self.path_real = Path(); self.path_real.header.frame_id = "odom"
        self.path_phys = Path(); self.path_phys.header.frame_id = "odom"
        self.path_pitcn = Path(); self.path_pitcn.header.frame_id = "odom"

        rospy.Timer(rospy.Duration(DT), self.control_loop)

    def motor_cb(self, msg):
        self.current_u = msg.data

    def odom_cb(self, msg):
        # 获取真实位姿
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        _, _, theta = euler_from_quaternion([o.x, o.y, o.z, o.w])
        
        now = rospy.Time.now()
        
        # 1. 初始化
        if not self.initialized:
            self.state_phys = np.array([p.x, p.y, theta, v, w])
            self.state_pitcn = np.array([p.x, p.y, theta, v, w])
            self.initialized = True
            rospy.loginfo(">>> 状态已同步，开始伴飞！")
        
        # 2. 定时重置 (可选)
        if self.reset_interval > 0 and (now - self.last_reset_time).to_sec() > self.reset_interval:
            rospy.loginfo(">>> 重置位置到真实状态 (Sync)...")
            self.state_phys = np.array([p.x, p.y, theta, v, w])
            self.state_pitcn = np.array([p.x, p.y, theta, v, w])
            self.last_reset_time = now
            # 清空轨迹看起来更清爽
            self.path_real.poses = []
            self.path_phys.poses = []
            self.path_pitcn.poses = []

        # 3. 发布真实轨迹
        self.append_path(self.path_real, self.pub_real, p.x, p.y)

    def control_loop(self, event):
        if not self.initialized: return
        
        # 1. 更新输入缓冲 (全闭环：使用模型自己的 v, w)
        # 为了公平对比，这里统一使用 PI-TCN 的状态 v, w 进入历史窗口
        curr_feat = np.concatenate(([self.state_pitcn[3], self.state_pitcn[4]], self.current_u))
        self.input_buffer.append(curr_feat)
        
        if len(self.input_buffer) < self.seq_len: return

        # 2. 神经网络推理
        # 构造数据 (与训练一致：按时间步交替)
        raw_data = np.array(self.input_buffer) # (20, 6)
        
        # 这里需要极度小心：verify_model 里的逻辑是 [feat1_t-19...t, feat2...]
        # 我们的 buffer 是 [t-19, t-18...]
        # 转置 -> (6, 20) -> 展平 -> (120)
        flat_input = raw_data.T.flatten().reshape(1, -1)
        
        # 归一化
        scaled_input = self.scaler_hist.transform(flat_input)
        
        X_tensor = torch.tensor(scaled_input, dtype=torch.float32).to(self.device)
        # Reshape -> (Batch, Channels, Seq) -> (1, 6, 20)
        X_tensor = X_tensor.view(1, 6, 20) 
        
        with torch.no_grad():
            bb_pred, _ = self.model(X_tensor)
            bb_res = self.scaler_label.inverse_transform(bb_pred.cpu().numpy())[0]

        # 3. 物理 + 积分 (RK4)
        u = self.current_u
        
        # 更新纯物理模型
        self.state_phys = self.rk4_step(self.state_phys, u, None, 0.0, False)
        
        # 更新 PI-TCN 模型
        self.state_pitcn = self.rk4_step(self.state_pitcn, u, bb_res, self.lambda_val, True)
        
        # 4. 发布轨迹
        self.append_path(self.path_phys, self.pub_phys, self.state_phys[0], self.state_phys[1])
        self.append_path(self.path_pitcn, self.pub_pitcn, self.state_pitcn[0], self.state_pitcn[1])

    def get_derivatives(self, state, u, nn_res, lam, use_hybrid):
        x, y, theta, v, w = state
        u_sum = sum(u)
        u_diff = (u[0]+u[2]) - (u[1]+u[3])
        
        # 物理公式 (使用拟合参数)
        phys_vdot = PA * u_sum - PB * v
        phys_wdot = PC * u_diff - PD * w
        
        if use_hybrid:
            final_vdot = lam * phys_vdot + (1-lam) * nn_res[0]
            final_wdot = lam * phys_wdot + (1-lam) * nn_res[1]
        else:
            final_vdot, final_wdot = phys_vdot, phys_wdot
            
        return np.array([v*math.cos(theta), v*math.sin(theta), w, final_vdot, final_wdot])

    def rk4_step(self, state, u, nn_res, lam, use_hybrid):
        k1 = self.get_derivatives(state, u, nn_res, lam, use_hybrid)
        k2 = self.get_derivatives(state + 0.5*DT*k1, u, nn_res, lam, use_hybrid)
        k3 = self.get_derivatives(state + 0.5*DT*k2, u, nn_res, lam, use_hybrid)
        k4 = self.get_derivatives(state + DT*k3, u, nn_res, lam, use_hybrid)
        return state + (DT/6.0)*(k1 + 2*k2 + 2*k3 + k4)

    def append_path(self, path_msg, pub, x, y):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "odom"
        pose.pose.position.x = x
        pose.pose.position.y = y
        path_msg.poses.append(pose)
        if len(path_msg.poses) > 2000: path_msg.poses.pop(0)
        pub.publish(path_msg)

if __name__ == '__main__':
    try:
        LiveInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass