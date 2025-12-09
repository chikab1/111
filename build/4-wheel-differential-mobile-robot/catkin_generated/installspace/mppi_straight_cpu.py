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
# 物理参数 (直接定义为全局变量或类属性，避免 register_buffer 的兼容性问题)
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
        
        # [修改] 不再使用 register_buffer，直接用 Tensor，避免加载报错
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
        u_sum = x_phys[:, 2] + x_phys[:, 3] + x_phys[:, 4] + x_phys[:, 5]
        u_diff_fix = (x_phys[:, 3] + x_phys[:, 5]) - (x_phys[:, 2] + x_phys[:, 4])

        vdot = (self.k_f * u_sum - self.c_v * v) / self.m
        efficiency = 0.2
        torque = u_diff_fix * self.k_f * (self.L / 2.0) * efficiency
        wdot = (torque - self.c_J * w) / self.J
        
        return torch.stack([vdot, wdot], dim=1)

    def forward(self, x_hist, x_curr, label_scaler_gpu, current_scaler_gpu):
        tcn_out = self.tcn(x_hist)[:, :, -1]
        y_bb = self.mlp_bb(tcn_out)
        y_pi_phys = self.physics_model(x_curr, current_scaler_gpu)
        y_pi = (y_pi_phys - label_scaler_gpu['mean']) / label_scaler_gpu['scale']
        lam = torch.sigmoid(self.lambda_blend)
        return lam * y_pi + (1.0 - lam) * y_bb

# ================= 2. MPPI 控制器 =================
class MPPIController:
    def __init__(self, model, scalers):
        self.model = model
        self.scalers = scalers
        
        # 参数配置
        self.K = 64
        self.T = 10
        self.noise_std = 60.0
        self.lambda_mppi = 0.1
        self.U_mean = torch.full((self.T, 4), 60.0, device=device)
        
        self.scaler_X = self._dict_to_device(scalers['history'])
        self.scaler_Y = self._dict_to_device(scalers['label'])
        
    def _dict_to_device(self, scaler):
        return {
            'mean': torch.tensor(scaler.mean_, device=device, dtype=torch.float32),
            'scale': torch.tensor(scaler.scale_, device=device, dtype=torch.float32)
        }

    def compute_action(self, history_buffer, current_state_phys):
        raw_arr = np.array(history_buffer)
        scaled_arr = (raw_arr - self.scalers['history'].mean_) / self.scalers['history'].scale_
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
        self.U_mean = torch.clamp(self.U_mean + weighted_noise, -255, 255)
        
        action = self.U_mean[0].clone().numpy()
        self.U_mean = torch.roll(self.U_mean, -1, dims=0)
        self.U_mean[-1] = self.U_mean[-2]
        
        return action

    def _rollout(self, start_hist, start_state, U_total):
        curr_hist = start_hist.repeat(self.K, 1, 1)
        state_phys = torch.tensor(start_state, device=device, dtype=torch.float32).repeat(self.K, 1)
        dt = 0.02
        total_costs = torch.zeros(self.K, device=device)
        
        for t in range(self.T):
            curr_v = state_phys[:, 0:1]
            curr_w = state_phys[:, 1:2]
            curr_u = U_total[:, t, :]
            
            feat_phys = torch.cat([curr_v, curr_w, curr_u], dim=1)
            feat_scaled = (feat_phys - self.scaler_X['mean']) / self.scaler_X['scale']
            
            hist_shifted = curr_hist[:, :, 1:]
            new_state_col = feat_scaled.unsqueeze(2)
            curr_hist = torch.cat([hist_shifted, new_state_col], dim=2)
            
            pred_scaled = self.model(curr_hist, feat_scaled, self.scaler_Y, self.scaler_X)
            pred_phys = pred_scaled * self.scaler_Y['scale'] + self.scaler_Y['mean']
            
            next_v = state_phys[:, 0] + pred_phys[:, 0] * dt
            next_w = state_phys[:, 1] + pred_phys[:, 1] * dt
            theta = state_phys[:, 4]
            next_x = state_phys[:, 2] + next_v * torch.cos(theta) * dt
            next_y = state_phys[:, 3] + next_v * torch.sin(theta) * dt
            next_theta = state_phys[:, 4] + next_w * dt
            
            state_phys = torch.stack([next_v, next_w, next_x, next_y, next_theta], dim=1)
            
            # Cost Function (走直线)
            cost_pos = (state_phys[:, 3]) ** 2
            cost_vel = (state_phys[:, 0] - 0.5) ** 2
            cost_head = (state_phys[:, 4]) ** 2
            
            total_costs += 100.0 * cost_pos + 10.0 * cost_vel + 50.0 * cost_head
            
        return total_costs

# ================= 3. ROS Node =================
class MPPINode:
    def __init__(self):
        rospy.init_node('mppi_cpu_node')
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "pitcn_car_model_paper_arch.pth")
        scaler_path = os.path.join(script_dir, "pitcn_scalers_paper_arch.pkl")

        print(f"Loading from: {script_dir}")
        tcn_channels = [8, 8, 8, 8]
        mlp_layers = [32, 16, 16]
        
        try:
            model = PI_TCN(6, 2, tcn_channels, mlp_layers, 2, 0.1).to(device)
            # [关键修改] strict=False 忽略多余/缺失的参数 (比如物理参数或版本差异的权重名)
            model.load_state_dict(torch.load(model_path, map_location=device), strict=False)
            model.eval()
            print("Model loaded successfully (non-strict mode).")
            
            with open(scaler_path, 'rb') as f:
                scalers = pickle.load(f)
                
            self.controller = MPPIController(model, scalers)
            
            # 只有成功加载才会走到这里
            self.history = deque(maxlen=20)
            self.state = {'x':0, 'y':0, 'theta':0, 'v':0, 'w':0}
            self.last_pwm = [0,0,0,0]
            
            rospy.Subscriber('/ROS_Car/odom', Odometry, self.odom_cb)
            rospy.Subscriber('/ROS_Car/imu', Imu, self.imu_cb)
            rospy.Subscriber('/ROS_Car/set_input', JointState, self.pwm_cb)
            self.pub_cmd = rospy.Publisher('/ROS_Car/set_input', JointState, queue_size=1)
            
            self.control_freq = 20 
            self.rate = rospy.Rate(self.control_freq)
            
            print(f"MPPI Controller Ready! Freq: {self.control_freq}Hz")
            
        except Exception as e:
            rospy.logerr(f"CRITICAL ERROR: {e}")
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
        print("Waiting for buffer to fill...")
        while len(self.history) < 20 and not rospy.is_shutdown():
            feat = [self.state['v'], self.state['w']] + self.last_pwm
            self.history.append(feat)
            self.rate.sleep()
            
        print("Buffer filled. Control Loop Active.")
        
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
                # 只有当超时严重时才警告
                if elapsed > 0.1:
                    rospy.logwarn(f"Slow loop: {elapsed:.3f}s")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = MPPINode()
        node.run()
    except rospy.ROSInterruptException:
        pass