#!/usr/bin/env python3
import rospy
import torch
import numpy as np
import pandas as pd
import pickle
from collections import deque
import time
import sys
import os

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
from pytorch_tcn import TCN 

# ================= 0. CPU 优化配置 =================
device = torch.device("cpu")
torch.set_num_threads(4) 
print(f"Running on {device} with {torch.get_num_threads()} threads")

# ================= 1. 物理参数与模型 =================
m = 2.45
J = 0.022
L = 0.215
k_f = 0.0001226
c_v = 2.4745
c_J = 0.0381

class CarDataset:
    history_scaler = None
    current_scaler = None
    label_scaler = None

class PI_TCN(torch.nn.Module):
    def __init__(self, input_channels, output_size, tcn_channels, mlp_layers, kernel_size, dropout):
        super(PI_TCN, self).__init__()
        self.tcn = TCN(input_channels, tcn_channels, kernel_size=kernel_size, dropout=dropout)
        mlp_input = tcn_channels[-1]
        layers = []
        for h in mlp_layers:
            layers.append(torch.nn.Linear(mlp_input, h))
            layers.append(torch.nn.ReLU())
            mlp_input = h
        layers.append(torch.nn.Linear(mlp_input, output_size))
        self.mlp_bb = torch.nn.Sequential(*layers)
        
        self.m = torch.tensor(m)
        self.k_f = torch.tensor(k_f)
        self.c_v = torch.tensor(c_v)
        self.c_J = torch.tensor(c_J)
        self.L = torch.tensor(L)
        self.J = torch.tensor(J)
        self.lambda_blend = torch.nn.Parameter(torch.tensor(0.0))

    def physics_model(self, x_curr, scaler):
        x_phys = x_curr * scaler['scale'] + scaler['mean']
        v = x_phys[:, 0]
        w = x_phys[:, 1]
        u_fl = x_phys[:, 2]
        u_fr = x_phys[:, 3]
        u_rl = x_phys[:, 4]
        u_rr = x_phys[:, 5]

        u_sum = u_fl + u_fr + u_rl + u_rr
        # 保持与训练代码一致: 左 - 右
        u_diff = u_fl - u_fr + u_rl - u_rr 

        vdot = (self.k_f * u_sum - self.c_v * v) / self.m
        efficiency = 0.2
        torque = u_diff * self.k_f * (self.L / 2.0) * efficiency
        wdot = (torque - self.c_J * w) / self.J
        
        return torch.stack([vdot, wdot], dim=1)

    def forward(self, x_hist, x_curr, label_scaler_gpu, current_scaler_gpu):
        tcn_out = self.tcn(x_hist)[:, :, -1]
        y_bb = self.mlp_bb(tcn_out)
        y_pi_phys = self.physics_model(x_curr, current_scaler_gpu)
        y_pi = (y_pi_phys - label_scaler_gpu['mean']) / label_scaler_gpu['scale']
        lam = torch.sigmoid(self.lambda_blend)
        return lam * y_pi + (1.0 - lam) * y_bb

# ================= 2. MPPI 控制器 (圆形轨迹版) =================
class MPPIController:
    def __init__(self, model, scalers):
        self.model = model
        self.scalers = scalers
        
        self.K = 64
        self.T = 12
        self.noise_std = 30.0 
        self.lambda_mppi = 0.1
        
        # [修改 1] 初始猜测：不再是直行，而是给一个差速 (左转)
        # 左轮 60, 右轮 90 -> 产生向左的力矩 (因为 u_diff = 左-右 = -30, torque负, wdot负?)
        # 等等，之前的逻辑 u_diff = 左 - 右。
        # 如果要左转 (+w)，需要 u_diff 为正 (左 > 右)? 
        # 不对，左 > 右 是顺时针 (向右偏)。
        # 右 > 左 是逆时针 (向左偏)。
        # 我们的 physics_model 是 torque = (左-右) * ...
        # 如果 左-右 > 0 -> torque > 0 -> wdot > 0 -> 向左转。
        # 所以 U_mean 应该设置 左 > 右
        
        # 让我们设一个左转的初始值
        # [u_fl, u_fr, u_rl, u_rr]
        u_init = torch.tensor([80.0, 50.0, 80.0, 50.0], device=device) 
        self.U_mean = u_init.repeat(self.T, 1)
        
        # 重组 Scaler
        hist_mean = scalers['history'].mean_
        hist_scale = scalers['history'].scale_
        mean_states = hist_mean[:40].reshape(20, 2)
        scale_states = hist_scale[:40].reshape(20, 2)
        mean_inputs = hist_mean[40:].reshape(20, 4)
        scale_inputs = hist_scale[40:].reshape(20, 4)
        
        self.mean_matrix = np.hstack([mean_states, mean_inputs])
        self.scale_matrix = np.hstack([scale_states, scale_inputs])
        self.mean_tensor = torch.tensor(self.mean_matrix, device=device, dtype=torch.float32)
        self.scale_tensor = torch.tensor(self.scale_matrix, device=device, dtype=torch.float32)
        
        self.step_mean = self.mean_tensor[-1]
        self.step_scale = self.scale_tensor[-1]
        self.scaler_dict_gpu = {'mean': self.step_mean, 'scale': self.step_scale}
        
        self.scaler_Y = {
            'mean': torch.tensor(scalers['label'].mean_, device=device, dtype=torch.float32),
            'scale': torch.tensor(scalers['label'].scale_, device=device, dtype=torch.float32)
        }

    def compute_action(self, history_buffer, current_state_phys):
        raw_arr = np.array(history_buffer)
        scaled_arr = (raw_arr - self.mean_matrix) / self.scale_matrix
        hist_tensor = torch.tensor(scaled_arr, dtype=torch.float32, device=device).T.unsqueeze(0)
        
        noise = torch.randn(self.K, self.T, 4, device=device) * self.noise_std
        U_total = self.U_mean.unsqueeze(0) + noise
        U_total = torch.clamp(U_total, -255.0, 255.0)
        
        costs = self._rollout(hist_tensor, current_state_phys, U_total)
        
        min_cost = torch.min(costs)
        exp_costs = torch.exp(- (costs - min_cost) / self.lambda_mppi)
        
        denom = torch.sum(exp_costs)
        if denom < 1e-10: 
            weights = torch.ones_like(exp_costs) / self.K
        else:
            weights = exp_costs / denom
            
        weighted_noise = torch.sum(weights.view(-1, 1, 1) * noise, dim=0)
        
        with torch.no_grad():
            self.U_mean = torch.clamp(self.U_mean + weighted_noise, -255, 255)
        
        action = self.U_mean[0].detach().cpu().clone().numpy()
        self.U_mean = torch.roll(self.U_mean, -1, dims=0)
        self.U_mean[-1] = self.U_mean[-2]
        
        return action

    def _rollout(self, start_hist, start_state, U_total):
        curr_hist = start_hist.repeat(self.K, 1, 1)
        state_phys = torch.tensor(start_state, device=device, dtype=torch.float32).repeat(self.K, 1)
        dt = 0.05
        total_costs = torch.zeros(self.K, device=device)
        
        for t in range(self.T):
            curr_v = state_phys[:, 0:1]
            curr_w = state_phys[:, 1:2]
            curr_u = U_total[:, t, :]
            
            feat_phys = torch.cat([curr_v, curr_w, curr_u], dim=1)
            feat_scaled = (feat_phys - self.step_mean) / self.step_scale
            
            hist_shifted = curr_hist[:, :, 1:]
            new_state_col = feat_scaled.unsqueeze(2)
            curr_hist = torch.cat([hist_shifted, new_state_col], dim=2)
            
            pred_scaled = self.model(curr_hist, feat_scaled, self.scaler_Y, self.scaler_dict_gpu)
            pred_phys = pred_scaled * self.scaler_Y['scale'] + self.scaler_Y['mean']
            
            next_v = state_phys[:, 0] + pred_phys[:, 0] * dt
            next_w = state_phys[:, 1] + pred_phys[:, 1] * dt
            theta = state_phys[:, 4]
            next_x = state_phys[:, 2] + next_v * torch.cos(theta) * dt
            next_y = state_phys[:, 3] + next_v * torch.sin(theta) * dt
            next_theta = state_phys[:, 4] + next_w * dt
            
            state_phys = torch.stack([next_v, next_w, next_x, next_y, next_theta], dim=1)
            
            # --- [修改 2] 跑圆形轨迹 Cost Function ---
            # 目标: 半径 R=1.0m, 速度 v=0.3m/s -> 角速度 w = 0.3 rad/s
            TARGET_V = 0.3
            TARGET_W = 0.3 # 正数向左转(逆时针)，负数向右转
            
            # 1. 速度惩罚
            cost_vel = (state_phys[:, 0] - TARGET_V) ** 2
            
            # 2. 角速度惩罚 (这是画圆的关键)
            cost_w = (state_phys[:, 1] - TARGET_W) ** 2
            
            # 3. 动作平滑 (油费)
            cost_action = torch.sum(curr_u ** 2, dim=1) * 0.00001
            
            # 权重配置: 
            # 速度权重 100, 角速度权重 200 (保证转弯)
            total_costs += 100.0 * cost_vel + 200.0 * cost_w + cost_action
            
        return total_costs

# ================= 3. ROS Node =================
class MPPINode:
    def __init__(self):
        rospy.init_node('mppi_circle_cpu')
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "pitcn_car_model_paper_arch.pth")
        scaler_path = os.path.join(script_dir, "pitcn_scalers_paper_arch.pkl")

        print(f"Loading Model from: {script_dir}")
        tcn_channels = [8, 8, 8, 8]
        mlp_layers = [32, 16, 16]
        
        try:
            model = PI_TCN(6, 2, tcn_channels, mlp_layers, 2, 0.1).to(device)
            model.load_state_dict(torch.load(model_path, map_location=device), strict=False)
            model.eval()
            print("Model loaded successfully.")
            
            with open(scaler_path, 'rb') as f:
                scalers = pickle.load(f)
                
            self.controller = MPPIController(model, scalers)
            
            self.history = deque(maxlen=20)
            self.state = {'x':0, 'y':0, 'theta':0, 'v':0, 'w':0}
            self.last_pwm = [0,0,0,0]
            
            rospy.Subscriber('/ROS_Car/odom', Odometry, self.odom_cb)
            rospy.Subscriber('/ROS_Car/imu', Imu, self.imu_cb)
            rospy.Subscriber('/ROS_Car/set_input', JointState, self.pwm_cb)
            self.pub_cmd = rospy.Publisher('/ROS_Car/set_input', JointState, queue_size=1)
            
            self.control_freq = 20 
            self.rate = rospy.Rate(self.control_freq)
            
            print(f"MPPI Circle Tracker Running! Target: v=0.3, w=0.3")
            
        except Exception as e:
            rospy.logerr(f"ERROR: {e}")
            sys.exit(1)

    def odom_cb(self, msg):
        self.state['x'] = msg.pose.pose.position.x
        self.state['y'] = msg.pose.pose.position.y
        self.state['v'] = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])
        self.state['theta'] = r.as_euler('xyz')[2]

    def imu_cb(self, msg):
        self.state['w'] = msg.angular_velocity.z

    def pwm_cb(self, msg):
        if len(msg.velocity) >= 4:
            self.last_pwm = list(msg.velocity)

    def run(self):
        print("Waiting for history buffer to fill...")
        while len(self.history) < 20 and not rospy.is_shutdown():
            feat = [self.state['v'], self.state['w']] + self.last_pwm
            self.history.append(feat)
            self.rate.sleep()
            
        print("Buffer filled. Start circling!")
        
        while not rospy.is_shutdown():
            start_time = time.time()
            
            feat = [self.state['v'], self.state['w']] + self.last_pwm
            self.history.append(feat)
            
            curr_phys = [self.state['v'], self.state['w'], self.state['x'], self.state['y'], self.state['theta']]
            optimal_pwm = self.controller.compute_action(self.history, curr_phys)
            
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.velocity = optimal_pwm.tolist()
            self.pub_cmd.publish(msg)
            
            elapsed = time.time() - start_time
            if elapsed > (1.0 / self.control_freq):
                if elapsed > 0.1: 
                    rospy.logwarn(f"Slow loop: {elapsed:.3f}s")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = MPPINode()
        node.run()
    except rospy.ROSInterruptException:
        pass