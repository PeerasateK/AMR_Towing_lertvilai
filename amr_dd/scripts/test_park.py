import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
from scipy.spatial.transform import Rotation as R
from math import *
import time

class map_navigation():

    def choose(self):
        choice='q'
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|PRESSE A KEY:")
        rospy.loginfo("|'0': Home")
        rospy.loginfo("|'1': Park")
        rospy.loginfo("|'q': Quit ")
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|WHERE TO GO?")
        choice = input()
        return choice

    def __init__(self):
        # declare the coordinates of interest
        self.xHome  = 0
        self.yHome  = 0
        self.rzHome = 0
        self.xGo  = 0
        self.yGo  = 0
        self.rzGo = 0
        self.xPark = 5
        self.yPark = -4
        self.rzPark = pi/2
        # self.xPark = 5
        # self.yPark = -0.7
        # self.rzPark = 0

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.position = Point()
        self.move_cmd = Twist()
        self.range_park = 3
        self.pose_amr = 0
        self.pos_hook = 0
        self.parkState = 0
        self.goalReached = False

        # initiliaze
        rospy.init_node('map_navigation', anonymous=False)
        rospy.Subscriber("/amr_dd/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/odom",Odometry,self.position_amr)
        choice = self.choose()
        if (choice == '0'):
            self.goalReached = self.moveToGoal(self.xHome, self.yHome, self.rzHome)

        elif (choice == '1'):
            park_pose = self.find_position(self.xPark, self.yPark, self.rzPark)
            self.goalReached = self.moveToGoal(park_pose[0], park_pose[1], park_pose[2])
            time.sleep(2)
            self.ReversePark(self.xPark, self.yPark, self.rzPark)

        if (choice!='q'):
            if (self.goalReached):
                rospy.loginfo("Congratulations!")
            #rospy.spin()
            else:
                rospy.loginfo("Hard Luck!")

        while choice != 'q':
            choice = self.choose()
            if (choice == '0'):
                self.goalReached = self.moveToGoal(self.xHome, self.yHome, self.rzHome)

            elif (choice == '1'):
                park_pose = self.find_position(self.xPark, self.yPark, self.rzPark)
                self.goalReached = self.moveToGoal(park_pose[0], park_pose[1], park_pose[2])
                time.sleep(2)
                self.ReversePark(self.xPark, self.yPark, self.rzPark)
                rospy.loginfo("Congratulations!")

            if (choice!='q'):
                if (self.goalReached):
                    rospy.loginfo("Congratulations!")
                    #rospy.spin()
                else:
                    rospy.loginfo("Hard Luck!")

    def joint_state_callback(self,data):
        self.pos_hook = data.position[0]

    def position_amr(self,odom_data):
        self.pose_amr = [odom_data.pose.pose.position.x,odom_data.pose.pose.position.y,odom_data.pose.pose.orientation]

    def find_position(self,xPark,yPark,rzPark):
        pose_park_p1 =[]
        homo_w_park  = np.array([[np.cos(rzPark), -np.sin(rzPark), 0,xPark],
                        [np.sin(rzPark), np.cos(rzPark), 0,yPark],
                        [0, 0, 1,0],
                        [0,0,0,1]])
        homo_park_p1 = np.array([[np.cos(-pi/2), -np.sin(-pi/2), 0,self.range_park],
                        [np.sin(-pi/2), np.cos(-pi/2), 0,0],
                        [0, 0, 1,0],
                        [0,0,0,1]])
        homo_w_p1 = homo_w_park @ homo_park_p1
        a = np.array([homo_w_p1[:3][0][:3],homo_w_p1[:3][1][:3],homo_w_p1[:3][2][:3]])
        r = R.from_matrix(a)
        pose_park_p1.append(homo_w_p1[:3][0][3])
        pose_park_p1.append(homo_w_p1[:3][1][3])
        pose_park_p1.append(r.as_rotvec()[2])
        return pose_park_p1

    def shutdown(self):
        # stop robot
        rospy.loginfo("Quit program")
        rospy.sleep()

    def ReversePark(self,xPark,yPark,rzPark):
        while self.parkState == 0:
            homo_w_park  = np.array([[np.cos(rzPark), -np.sin(rzPark), 0,xPark],
                        [np.sin(rzPark), np.cos(rzPark), 0,yPark],
                        [0, 0, 1,0],
                        [0,0,0,1]])
            homo_park_w = np.linalg.inv(homo_w_park)
            #find pose world cart
            quaternion = (self.pose_amr[2].x,self.pose_amr[2].y,self.pose_amr[2].z,self.pose_amr[2].w)
            euler = transformations.euler_from_quaternion(quaternion)
            yaw_amr = euler[2]
            homo_w_amr = np.array([[np.cos(yaw_amr), -np.sin(yaw_amr), 0,self.pose_amr[0]],
                        [np.sin(yaw_amr), np.cos(yaw_amr), 0,self.pose_amr[1]],
                        [0, 0, 1,0],
                        [0,0,0,1]])
            homo_amr_cart = np.array([[np.cos(self.pos_hook), -np.sin(self.pos_hook), 0,-1.4*cos(self.pos_hook)],
                        [np.sin(self.pos_hook), np.cos(self.pos_hook), 0,-1.4*sin(self.pos_hook)],
                        [0, 0, 1,0],
                        [0,0,0,1]])
            homo_w_cart = homo_w_amr @ homo_amr_cart
            homo_park_cart = homo_park_w @ homo_w_cart
            # a = np.array([homo_park_cart[:3][0][:3],homo_park_cart[:3][1][:3],homo_park_cart[:3][2][:3]])
            # r = R.from_matrix(a)
            # rad_park_cart = r.as_rotvec()[2]

            rad_park_cart = atan2(homo_park_cart[:3][1][3],homo_park_cart[:3][0][3])
            self.move_cmd.linear.x = 0.18
            self.move_cmd.angular.z = 0
            self.cmd_vel.publish(self.move_cmd)
            print(rad_park_cart)
            if rad_park_cart <= 0:
                self.parkState = 1
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
                self.cmd_vel.publish(self.move_cmd)

        while self.parkState == 1:
            print(self.pos_hook)
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0.1
            self.cmd_vel.publish(self.move_cmd)
            if self.pos_hook <= -1.570715:
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
                self.cmd_vel.publish(self.move_cmd)
                self.parkState = 2
        
        while self.parkState == 2:
            quaternion = (self.pose_amr[2].x,self.pose_amr[2].y,self.pose_amr[2].z,self.pose_amr[2].w)
            euler = transformations.euler_from_quaternion(quaternion)
            yaw_amr = euler[2]
            homo_w_amr = np.array([[np.cos(yaw_amr), -np.sin(yaw_amr), 0,self.pose_amr[0]],
                        [np.sin(yaw_amr), np.cos(yaw_amr), 0,self.pose_amr[1]],
                        [0, 0, 1,0],
                        [0,0,0,1]])
            homo_amr_cart = np.array([[np.cos(self.pos_hook), -np.sin(self.pos_hook), 0,-1.4*cos(self.pos_hook)],
                        [np.sin(self.pos_hook), np.cos(self.pos_hook), 0,-1.4*sin(self.pos_hook)],
                        [0, 0, 1,0],
                        [0,0,0,1]])
            homo_w_cart = homo_w_amr @ homo_amr_cart
            a = np.array([homo_w_cart[:3][0][:3],homo_w_cart[:3][1][:3],homo_w_cart[:3][2][:3]])
            r = R.from_matrix(a)
            rad_park_cart = r.as_rotvec()[2]
            self.move_cmd.linear.x = 0.5
            self.move_cmd.angular.z = 0.5/1.4
            self.cmd_vel.publish(self.move_cmd)
            print(rad_park_cart,rad_park_cart - self.rzPark)
            if abs(rad_park_cart - self.rzPark) <= 0.01 :
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
                self.cmd_vel.publish(self.move_cmd)
                self.parkState = 3

    def moveToGoal(self,xGoal,yGoal,rzGoal):

        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")


        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # eular to quaternion 
        quaternion = transformations.quaternion_from_euler(0,0,rzGoal,'sxyz')
        # moving towards the goal*/
        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(600))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False
    
if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")