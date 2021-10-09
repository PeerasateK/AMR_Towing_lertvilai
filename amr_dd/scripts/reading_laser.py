import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import *

pub = None

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    target = max(regions, key=regions.get)
    state_description = ''

    if regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 1 - Backward'
        linear_x = -0.6
        angular_z = 0
    elif regions['right'] < 0.6:
        state_description = 'case 7 - Emer Turn Left'
        linear_x = 0.4
        angular_z = 0.35
    elif regions['left'] < 0.6:
        state_description = 'case 7 - Emer Turn Right'
        linear_x = 0.4
        angular_z = -0.35
    elif target == "front":
        state_description = 'case 2 - Forward'
        linear_x = 0.7
        angular_z = 0
    elif target == "sfleft":
        state_description = 'case 3 - Slide SLeft'
        linear_x = 0.6
        angular_z = 0.1
    elif target == "sfright":
        state_description = 'case 4 - Slide SRight'
        linear_x = 0.6
        angular_z = -0.1
    elif target == "fleft":
        state_description = 'case 3 - Slide Left'
        linear_x = 0.4
        angular_z = 0.15
    elif target == "fright":
        state_description = 'case 4 - Slide Right'
        linear_x = 0.4
        angular_z = -0.15
    elif target == "left":
        state_description = 'case 5 - Turn Left'
        linear_x = 0.2
        angular_z = 0.3
    elif target == "right":
        state_description = 'case 6 - Turn Right'
        linear_x = 0.2
        angular_z = -0.3
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    print("{}: {}".format(state_description,regions))
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def clbk_laser(msg):
    # 720 /7 = 102 
    regions = {
        'right':  min(min(msg.ranges[0:101]), 10),
        'fright': min(min(msg.ranges[102:203]), 10),
        'sfright':  min(min(msg.ranges[204:305]), 10),
        'front':  min(min(msg.ranges[306:414]), 10),
        'sfleft':  min(min(msg.ranges[415:516]), 10),
        'fleft':  min(min(msg.ranges[517:618]), 10),
        'left':   min(min(msg.ranges[619:720]), 10),
    }
    take_action(regions)

def main():
    global pub
    rospy.init_node('reading_laser')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/amr_dd/laser/scanF', LaserScan, clbk_laser)
    rospy.spin()

if __name__ == '__main__':
    main()