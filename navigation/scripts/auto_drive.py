#!/usr/bin/env python
# license removed for brevity

from sensor_msgs.msg import Joy, NavSatFix
from nav_msgs.msg import Odometry
import actionlib
import rospy
import tf2_ros, tf

from geometry_msgs.msg import TransformStamped, PoseStamped
from rospy import ROSException
import tf2_geometry_msgs
import geonav_transform.geonav_conversions as gc

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


goal_tobe_stored = None; previous_stored_goal = None

previous_l2_state = 0
route_wayPoints = []

tfBuffer = None
tf2_listener = None
first_gpsMsg = True
olat = None; olon = None

def timer_callback():
    if goal_tobe_stored is not None and previous_stored_goal != goal_tobe_stored:
        print "\n##############\nRecord new way point... \n"
        print goal_tobe_stored
        print("\n#######################\n")
        route_wayPoints.append(goal_tobe_stored)
        previous_stored_goal = goal_tobe_stored

timer = None

def odomCallBack(odom_msg):
    global tfBuffer
    odom_pose = odom_msg.pose
    transform = tfBuffer.lookup_transform('map',
        'odom', #source frame
        rospy.Time(0), #get the tf at first available time
        rospy.Duration(1.0)) #wait for 1 second
    map_msg = tf2_geometry_msgs.do_transform_pose(odom_pose, transform)
    # print "Pose in Map Frame is:\n"
    # print map_msg
    # return    
    goal_tobe_stored = MoveBaseGoal()     
    try:
        goal_tobe_stored.target_pose.header.frame_id = 'map'
        goal_tobe_stored.target_pose.header.stamp = rospy.Time(0)
        goal_tobe_stored.target_pose.pose = map_msg.pose
    except:
        pass

def gpsCallBack(gps_msg):
    global first_gpsMsg, olat, olon
    
    if first_gpsMsg == True:
        first_gpsMsg = False
        olat = gps_msg.latitude
        olon = gps_msg.longitude

    lat = gps_msg.latitude
    lon = gps_msg.longitude
    xg, yg = gc.ll2xy(lat,lon,olat,olon)
    gpsGoal = MoveBaseGoal()
    gpsGoal.target_pose.pose.position.x = xg
    gpsGoal.target_pose.pose.position.y = yg
    gpsGoal.target_pose.pose.orientation.w = 1
    goal_tobe_stored = MoveBaseGoal()
    try:
        goal_tobe_stored.target_pose.header.frame_id = 'map'
        goal_tobe_stored.target_pose.header.stamp = rospy.Time(0)
        goal_tobe_stored.target_pose.pose = gpsGoal.target_pose.pose
        # print goal_tobe_stored
    except:
        pass

def joy_callback(data):
    global previous_l2_state, timer, first_gpsMsg
    buttons = data.buttons
    try:
        if previous_l2_state == 0 and buttons[7] == 1: ## rising edge --> means start now recording the route
            print "\n\n#########################\nNow we start recording waypoints every 5 seconds\n###############\n"        
            route_wayPoints = []
            goal_tobe_stored = None
            previous_stored_goal = None
            first_gpsMsg = True 
            timer = rospy.Timer(rospy.Duration(10), timer_callback)
            previous_l2_state = 1

        elif previous_l2_state == 1 and buttons[7] == 0: ## faling edge --> means stop now recording the route
            previous_l2_state = 0
            timer.shutdown()
            print "\n\n#########################\nNow we execute those waypoints\n###############\n"
            print route_wayPoints
            print "\n##########################\n"
            # execute_stored_route()

    except:
        rospy.logerr("error while parsing joystick input")
    # old_buttons = data.buttons


def movebase_client(desired_goal):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Sends the goal to the action server.
    client.send_goal(desired_goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

def execute_stored_route():
    for waygoal in route_wayPoints:
        result = movebase_client(waygoal)
        if result:
            rospy.loginfo("Goal execution done!")
        else:
            rospy.logerr("Goal execution failed")

if __name__ == '__main__':
    rospy.init_node('autopilot_node')
    try:
        global tf2_listener, tfBuffer
        tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        tf2_listener = tf2_ros.TransformListener(tfBuffer)
        # print("\n\n####################### Before#########################\n\n")
        rospy.Subscriber('joy', Joy, joy_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, odomCallBack)
        # rospy.Subscriber('/gps/filtered', NavSatFix, gpsCallBack)
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")