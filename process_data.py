#!/usr/bin/env python3
import rospy
import bagpy
import pandas as pd
import numpy as np
import glob
import os
import shutil
from tqdm import tqdm
from scipy.signal import savgol_filter, butter, filtfilt
from scipy.spatial.transform import Rotation as scipy_rotation

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

# --- Matplotlib 设置 ---
plt.rcParams["figure.figsize"] = (19.20, 10.80)
font = {"family" : "sans-serif", "weight" : "normal", "size" : 22}
matplotlib.rc("font", **font)
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42
colors = ["#7d7376","#365282","#e84c53","#edb120"]

# --- 辅助函数 ---
def initFolders():
    dirs = ["info", "train", "test"]
    for d in dirs:
        if os.path.exists(d): shutil.rmtree(d)
        os.makedirs(d)

def parseBag(topic, path):
    try:
        b = bagpy.bagreader(path, verbose=False)
        csv_file = b.message_by_topic(topic)
        return pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading topic {topic}: {e}")
        return pd.DataFrame()

def appendHistory(df, data_columns, label_columns, history_length):
    """构建时间序列历史数据"""
    state_columns = [col for col in data_columns if "u_" not in col]
    aux_state_cols = ["p_x", "p_y", "theta_z"]
    available_state_cols = [c for c in state_columns + aux_state_cols if c in df.columns]
    df_state = df[available_state_cols]

    # 使用 bfill 消除警告
    df_state_history = df_state.rename(columns={col: col + "_t" for col in df_state.columns})
    for j in range(1, 1 + history_length):
        shifted_df = df_state.shift(j)
        shifted_df.bfill(inplace=True)
        col_names = {col: col + "_t-" + str(j) for col in list(df_state.columns)}
        shifted_df.rename(columns=col_names, inplace=True)
        df_state_history = pd.concat([shifted_df, df_state_history], axis=1)

    input_columns = [col for col in data_columns if "u_" in col]
    df_input = df[input_columns]
    df_input_history = df_input.rename(columns={col: col + "_t" for col in input_columns})
    for j in range(1, 1 + history_length):
        shifted_df = df_input.shift(j)
        shifted_df.bfill(inplace=True)
        col_names = {col: col + "_t-" + str(j) for col in list(df_input.columns)}
        shifted_df.rename(columns=col_names, inplace=True)
        df_input_history = pd.concat([shifted_df, df_input_history], axis=1)

    df_history = pd.concat([df_state_history, df_input_history, df[label_columns]], axis=1)
    df_history.dropna(inplace=True)
    return df_history

def computeSpline(name, arr, t, steps, cols):
    if len(t.shape) == 1: t = t.reshape((t.shape[0], 1))
    splines = {}
    for i in range(len(cols)):
        unique_t, indices = np.unique(t[:, 0], return_index=True)
        unique_data = arr[indices, i]
        # 线性插值
        splines[name + "_" + cols[i]] = np.interp(steps, unique_t, unique_data)
    return splines

def butter_lowpass_filter(data, cutoff, fs, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def clean_small_values(data, threshold=1e-3):
    """将绝对值小于 threshold 的数置为 0"""
    data[np.abs(data) < threshold] = 0.0
    return data

# --- 动力学模型 ---
def nominalModel(data, k_f, m, c_v, L, J, c_J):
    v = data["v_x"]
    w = data["w_z"]
    
    # PWM 对齐（解决通信延迟）
    delay_step = 10
    u_fl = np.roll(data["u"][:, 0], delay_step)
    u_fr = np.roll(data["u"][:, 1], delay_step)
    u_rl = np.roll(data["u"][:, 2], delay_step)
    u_rr = np.roll(data["u"][:, 3], delay_step)
    for arr in [u_fl, u_fr, u_rl, u_rr]: arr[:delay_step] = 0

    u_sum = u_fl + u_fr + u_rl + u_rr
    u_diff = (u_fr + u_rr) - (u_fl + u_rl)

    vdot_nom = ((k_f * u_sum) - (c_v * v)) / m
    torque_drive = u_diff * k_f * (L / 2.0)
    wdot_nom = (torque_drive - (c_J * w)) / J

    return vdot_nom.reshape(-1, 1), wdot_nom.reshape(-1, 1)

def saveTrajectory(df, file_name):
    pp = PdfPages("info/" + file_name + ".pdf")
    
    fig = plt.figure()
    plt.title('Position X-Y Trajectory')
    plt.plot(df["p_x"], df["p_y"], c=colors[0])
    plt.xlabel(r'$p_x$ $[m]$'); plt.ylabel(r'$p_y$ $[m]$'); plt.grid(); plt.axis('equal')
    pp.savefig(fig)

    fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True)
    fig.suptitle('Velocities')
    axes[0].plot(df["t"], df["v_x"], c=colors[1])
    axes[1].plot(df["t"], df["w_z"], c=colors[1])
    axes[0].set_ylabel(r'$v$ $[m/s]$'); axes[1].set_ylabel(r'$\omega$ $[rad/s]$')
    axes[0].grid(); axes[1].grid()
    pp.savefig(fig)

    fig, axes = plt.subplots(nrows=2, ncols=1, sharex=True)
    fig.suptitle('Accelerations (Label vs Nominal)')
    axes[0].plot(df["t"], df["vdot_x"], c=colors[1], label="Actual")
    axes[0].plot(df["t"], df["vdot_nom_x"], c=colors[2], linestyle='--', label="Nominal")
    axes[1].plot(df["t"], df["wdot_z"], c=colors[1])
    axes[1].plot(df["t"], df["wdot_nom_z"], c=colors[2], linestyle='--')
    axes[0].set_ylabel(r'$\dot{v}$ $[m/s^2]$'); axes[1].set_ylabel(r'$\dot{\omega}$ $[rad/s^2]$')
    axes[0].legend(); axes[0].grid(); axes[1].grid()
    pp.savefig(fig)
    
    fig, axes = plt.subplots(nrows=4, ncols=1, sharex=True)
    axes[0].plot(df["t"], df["u_fl"], c=colors[3], label="u_fl")
    axes[1].plot(df["t"], df["u_fr"], c=colors[3], label="u_fr")
    axes[2].plot(df["t"], df["u_rl"], c=colors[3], label="u_rl")
    axes[3].plot(df["t"], df["u_rr"], c=colors[3], label="u_rr")
    axes[0].legend(loc="upper right")
    pp.savefig(fig)

    pp.close(); plt.close('all')

def processBag(path):
    # ================= 物理参数 =================
    m = 2.45; J = 0.022; L = 0.215
    k_f = 0.005513
    c_v = 2.351952
    c_J = 0.093184
    # ==========================================

    frequency = 100.0 
    
    print(f"Processing {path}...")
    df_odom = parseBag('/ROS_Car/odom', path)
    df_imu = parseBag('/ROS_Car/imu', path)
    df_motor = parseBag('/ROS_Car/motor_rpm', path)

    if df_odom.empty or df_motor.empty: return None

    # 时间戳提取
    t_odom = df_odom["header.stamp.secs"] + df_odom["header.stamp.nsecs"] * 1e-9
    t_imu = df_imu["header.stamp.secs"] + df_imu["header.stamp.nsecs"] * 1e-9
    t_motor = df_motor["header.stamp.secs"] + df_motor["header.stamp.nsecs"] * 1e-9
    
    t_odom = t_odom.to_numpy().reshape(-1, 1)
    t_imu = t_imu.to_numpy().reshape(-1, 1)
    t_motor = t_motor.to_numpy().reshape(-1, 1)

    start_time = max(t_odom[0], t_imu[0], t_motor[0])
    end_time = min(t_odom[-1], t_imu[-1], t_motor[-1])
    start_time = np.ceil(start_time * frequency) / frequency
    end_time = np.floor(end_time * frequency) / frequency
    sampling_steps = np.arange(start_time, end_time, 1.0 / frequency)
    sampled_data = {"t": sampling_steps - sampling_steps[0]}

    # --- 1. 姿态 & 位置 ---
    q_cols = ["pose.pose.orientation.x", "pose.pose.orientation.y", "pose.pose.orientation.z", "pose.pose.orientation.w"]
    quat = df_odom[q_cols].to_numpy()
    r = scipy_rotation.from_quat(quat)
    euler = r.as_euler('xyz')
    theta_z = euler[:, 2].reshape(-1, 1)
    sampled_data.update(computeSpline("theta", theta_z, t_odom, sampling_steps, ["z"]))

    p = df_odom[["pose.pose.position.x", "pose.pose.position.y"]].to_numpy()
    sampled_data.update(computeSpline("p", p, t_odom, sampling_steps, ["x", "y"]))

    # --- 2. 速度 (Odom) & 清洗 ---
    v_x = df_odom[["twist.twist.linear.x"]].to_numpy()
    sampled_data.update(computeSpline("v", v_x, t_odom, sampling_steps, ["x"]))
    sampled_data["v_x"] = clean_small_values(sampled_data["v_x"], threshold=1e-4)

    # --- 3. 角速度 (IMU) & 清洗 ---
    w_z = df_imu[["angular_velocity.z"]].to_numpy()
    sampled_data.update(computeSpline("w", w_z, t_imu, sampling_steps, ["z"]))
    sampled_data["w_z"] = clean_small_values(sampled_data["w_z"], threshold=1e-4)

    # --- 4. PWM 输入 & 强制取整 ---
    possible_cols = ["velocity_0", "velocity_1", "velocity_2", "velocity_3"]
    if not all(col in df_motor.columns for col in possible_cols):
        print("Warning: Standard velocity columns not found, using fallback...")
        u = df_motor.iloc[:, [6, 7, 8, 9]].to_numpy()
    else:
        u = df_motor[possible_cols].to_numpy()
        
    sampled_data.update(computeSpline("u", u, t_motor, sampling_steps, ["fl", "fr", "rl", "rr"]))
    
    u_keys = ["u_fl", "u_fr", "u_rl", "u_rr"]
    for k in u_keys:
        sampled_data[k] = np.round(sampled_data[k])
        sampled_data[k] = clean_small_values(sampled_data[k], threshold=0.1)
    
    # 构建临时矩阵用于物理计算
    sampled_data["u"] = np.column_stack([sampled_data[k] for k in u_keys])

    # --- 5. 滤波与求导 ---
    v_smooth = butter_lowpass_filter(sampled_data["v_x"], cutoff=5.0, fs=frequency)
    w_smooth = butter_lowpass_filter(sampled_data["w_z"], cutoff=5.0, fs=frequency)
    
    vdot_x = savgol_filter(v_smooth, window_length=51, polyorder=2, deriv=1, delta=1.0/frequency)
    wdot_z = savgol_filter(w_smooth, window_length=51, polyorder=2, deriv=1, delta=1.0/frequency)
    
    sampled_data["vdot_x"] = clean_small_values(vdot_x, threshold=1e-3)
    sampled_data["wdot_z"] = clean_small_values(wdot_z, threshold=1e-3)

    # --- 6. 名义模型 ---
    vdot_nom, wdot_nom = nominalModel(sampled_data, k_f, m, c_v, L, J, c_J)
    sampled_data["vdot_nom_x"] = vdot_nom.flatten()
    sampled_data["wdot_nom_z"] = wdot_nom.flatten()

    del sampled_data["u"] 

    return sampled_data

def main():
    debug_mode = False
    history_length = 20
    num_test_traj = 0

    data_columns = ["v_x", "w_z", "u_fl", "u_fr", "u_rl", "u_rr"]
    label_columns = ["vdot_x", "wdot_z"]

    initFolders()
    bag_paths = glob.glob("*.bag")
    if not bag_paths:
        print("No .bag files found!")
        return

    if debug_mode:
        print("⚠️ DEBUG MODE: Only 1 bag")
        bag_paths = bag_paths[:1]

    processed_list = []
    for path in tqdm(bag_paths, desc="Processing"):
        data = processBag(path)
        if data is not None:
            processed_list.append((path, data))
            bag_name = os.path.splitext(path)[0]
            if os.path.exists(bag_name): shutil.rmtree(bag_name)

    for i, (path, data) in tqdm(enumerate(processed_list), total=len(processed_list), desc="Saving"):
        file_name = os.path.basename(path)[:-4]
        df = pd.DataFrame(data)
        saveTrajectory(df, file_name)

        df_history = appendHistory(df, data_columns, label_columns, history_length)
        
        if i < num_test_traj:
            df_history.to_csv(f"test/{file_name}.csv", index=False)
        else:
            header = (i == num_test_traj)
            mode = 'w' if (i == num_test_traj) else 'a'
            df_history.to_csv("train/data.csv", index=False, mode=mode, header=header)

    print("Done! Data saved.")

if __name__ == "__main__":
    main()
