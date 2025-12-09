#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import tf.transformations as tft
import numpy as np

class GroundTruthOdom:
    def __init__(self):
        rospy.init_node('ground_truth_odom')
        self.robot_name = 'car'  # 确保和 launch 文件里的 -model 名字一致
        self.pub = rospy.Publisher('/ROS_Car/odom', Odometry, queue_size=1)
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)

    def callback(self, msg):
        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            
            # 1. 获取模型在世界坐标系下的姿态和速度
            pose = msg.pose[idx]
            twist_world = msg.twist[idx]
            
            # 2. 【核心修复】将世界坐标系速度 投影回 车身坐标系
            # 这样无论车头朝哪，向前开永远是 +x 速度
            q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            # 计算旋转矩阵 (Body -> World)
            rot = tft.quaternion_matrix(q)[:3, :3]
            # 计算逆矩阵 (World -> Body)
            rot_inv = rot.T 
            
            v_world = np.array([twist_world.linear.x, twist_world.linear.y, twist_world.linear.z])
            v_body = np.dot(rot_inv, v_world) # 投影到车身
            
            w_world = np.array([twist_world.angular.x, twist_world.angular.y, twist_world.angular.z])
            w_body = np.dot(rot_inv, w_world)

            # 3. 发布修正后的 Odom
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose = pose
            
            # 这里填入转换后的车身速度！
            odom.twist.twist.linear.x = v_body[0] 
            odom.twist.twist.linear.y = v_body[1]
            odom.twist.twist.linear.z = v_body[2]
            odom.twist.twist.angular.x = w_body[0]
            odom.twist.twist.angular.y = w_body[1]
            odom.twist.twist.angular.z = w_body[2]
            
            self.pub.publish(odom)

if __name__ == '__main__':
    try:
        GroundTruthOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass