import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from tf import transformations

class map_navigation():

    def choose(self):
        choice='q'
        rospy.loginfo("|-------------------------------|")
        rospy.loginfo("|PRESSE A KEY:")
        rospy.loginfo("|'0': Home")
        rospy.loginfo("|'1': Room")
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
        self.xRoom = 16
        self.yRoom = -26
        self.rzRoom = -0.7854
 
        self.goalReached = False
        # initiliaze
        rospy.init_node('map_navigation', anonymous=False)
        choice = self.choose()
        if (choice == '0'):
            self.goalReached = self.moveToGoal(self.xHome, self.yHome, self.rzHome)

        elif (choice == '1'):
            self.goalReached = self.moveToGoal(self.xRoom, self.yRoom, self.rzRoom)

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
                self.goalReached = self.moveToGoal(self.xRoom, self.yRoom, self.rzRoom)
                rospy.loginfo("Congratulations!")

            if (choice!='q'):   
                if (self.goalReached):
                    rospy.loginfo("Congratulations!")
                    #rospy.spin()
                else:
                    rospy.loginfo("Hard Luck!")


    def shutdown(self):
        # stop robot
        rospy.loginfo("Quit program")
        rospy.sleep()

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