import rospy
from geometry_msgs.msg import *

def boundary_plot():
    pub = rospy.Publisher('boundary_plot',Point,queue_size=1)
    rospy.init_node('graph_plot', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        msg_boundary = Point()
        msg_boundary.x= 14
        msg_boundary.y= 14
        pub.publish(msg_boundary)
        rospy.sleep(1) # Sleeps for 1 sec
        msg_boundary.x= 14
        msg_boundary.y= -3
        pub.publish(msg_boundary)
        rospy.sleep(1) # Sleeps for 1 sec
        msg_boundary.x= -3
        msg_boundary.y= -3
        pub.publish(msg_boundary)
        rospy.sleep(1) # Sleeps for 1 sec
        msg_boundary.x= -3
        msg_boundary.y= 14
        pub.publish(msg_boundary)
        rospy.sleep(1) # Sleeps for 1 sec

if __name__ == '__main__':
    boundary_plot()
    rospy.spin()