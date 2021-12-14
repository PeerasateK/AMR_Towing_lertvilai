#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from math import *
from amr_dd.msg import boundary_cart
import numpy as np
from tf import transformations
from scipy.spatial.transform import Rotation as R
from math import *

pos_amr = [0,0]
pos_hook = 0
pub = rospy.Publisher('pos_cart',boundary_cart,queue_size=1)

def find_pos_cart(pos_amr,Q,pos_hook):
    pos_w_cart=[]
    euler = transformations.euler_from_quaternion(Q)
    yaw_amr = euler[2]
    homo_w_amr = np.array([[np.cos(yaw_amr), -np.sin(yaw_amr), 0,pos_amr[0]],
                        [np.sin(yaw_amr), np.cos(yaw_amr), 0,pos_amr[1]],
                        [0, 0, 1,0],
                        [0,0,0,1]])
    homo_amr_cart = np.array([[np.cos(pos_hook), -np.sin(pos_hook), 0,-1.826*cos(pos_hook)],
                        [np.sin(pos_hook), np.cos(pos_hook), 0,-1.826*sin(pos_hook)],
                        [0, 0, 1,0],
                        [0,0,0,1]])
    homo_cart_p1 = np.array([[1, 0, 0,1],
                        [0, 1, 0,0.465],
                        [0, 0, 1,0],
                        [0,0,0,1]])
    homo_cart_p2 = np.array([[1, 0, 0,1],
                        [0, 1, 0,-0.465],
                        [0, 0, 1,0],
                        [0,0,0,1]])
    homo_cart_p3 = np.array([[1, 0, 0,-1],
                        [0, 1, 0,0.465],
                        [0, 0, 1,0],
                        [0,0,0,1]])
    homo_cart_p4 = np.array([[1, 0, 0,-1],
                        [0, 1, 0,-0.465],
                        [0, 0, 1,0],
                        [0,0,0,1]])

    # pos_w_cart.append((homo_w_amr @ homo_amr_cart)[0][3])
    # pos_w_cart.append((homo_w_amr @ homo_amr_cart)[1][3])
    pos_w_cart.append((homo_w_amr @ homo_amr_cart @ homo_cart_p1)[0][3])
    pos_w_cart.append((homo_w_amr @ homo_amr_cart @ homo_cart_p1)[1][3])
    pos_w_cart.append((homo_w_amr @ homo_amr_cart @ homo_cart_p2)[0][3])
    pos_w_cart.append((homo_w_amr @ homo_amr_cart @ homo_cart_p2)[1][3])
    pos_w_cart.append((homo_w_amr @ homo_amr_cart @ homo_cart_p3)[0][3])
    pos_w_cart.append((homo_w_amr @ homo_amr_cart @ homo_cart_p3)[1][3])
    pos_w_cart.append((homo_w_amr @ homo_amr_cart @ homo_cart_p4)[0][3])
    pos_w_cart.append((homo_w_amr @ homo_amr_cart @ homo_cart_p4)[1][3])
    homo_w_cart = homo_w_amr @ homo_amr_cart
    pose_w_cart = []
    pose_w_cart.append(homo_w_cart[:3][0][3])
    pose_w_cart.append(homo_w_cart[:3][1][3])
    a = np.array([homo_w_cart[:3][0][:3],homo_w_cart[:3][1][:3],homo_w_cart[:3][2][:3]])
    r = R.from_matrix(a)
    rad_w_cart = r.as_rotvec()[2]
    pose_w_cart.append(rad_w_cart)
    print(pose_w_cart,pos_hook)
    # print(pos_amr[0],pos_amr[1],yaw_amr)
    return pos_w_cart

def joint_state_callback(data):
    global pos_hook
    pos_hook = data.position[0]
    # print("Position_hook: {}".format(data.position[0]))

def Position(odom_data):
    global pos_amr
    if abs(pos_amr[0]-odom_data.pose.pose.position.x)>=0.01 or abs(pos_amr[1]-odom_data.pose.pose.position.y)>=0.00001:
        pos_amr = [odom_data.pose.pose.position.x,odom_data.pose.pose.position.y]
        q=[odom_data.pose.pose.orientation.x,odom_data.pose.pose.orientation.y,odom_data.pose.pose.orientation.z,odom_data.pose.pose.orientation.w]
        pos_cart = find_pos_cart(pos_amr,q,pos_hook)
        # print(pos_cart)
        msg=boundary_cart()
        msg.point1.x=pos_cart[0]
        msg.point1.y=pos_cart[1]
        msg.point2.x=pos_cart[2]
        msg.point2.y=pos_cart[3]
        msg.point3.x=pos_cart[4]
        msg.point3.y=pos_cart[5]
        msg.point4.x=pos_cart[6]
        msg.point4.y=pos_cart[7]
        pub.publish(msg)


def listener():
    rospy.init_node('plot_sim', anonymous=False)
    rospy.Subscriber("/amr200/joint_states", JointState, joint_state_callback)
    rospy.Subscriber("/odom",Odometry,Position)
    
if __name__ == '__main__':
    listener()
    rospy.spin()