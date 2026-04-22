#!/usr/bin/env python3
import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped
import tf.transformations

def set_initial_pose(x, y, z=0.0): #set the initial:same as clicking 2D Pose Estimate in RViz manually
    #/initialpose is a built-in ROS topic AMCL automatically subscribes to;same topic that RViz's 2D Pose Estimate button 
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(1.0)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    #Converts degrees → quaternion for the robot's facing direction
    q = tf.transformations.quaternion_from_euler(0, 0, math.radians(z))
    #Sets which direction the robot is facing.
    msg.pose.pose.orientation = Quaternion(*q)
    #Standard default values.tells AMCL how confident we are about the initial pose. 
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.07
    pub.publish(msg)
    rospy.loginfo("Initial pose set!")
    rospy.sleep(2.0)

def make_goal(x, y, z=0.0):
    # MoveBaseGoal(): prebuild structure: Header: frame and timestamp; pose position: x,y,z,w
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    degree = math.radians(z)
    # use quaternions for computer instead of euler (human way)
    # z = sin(theta/2) ; w = cos(theta/2)
    q = tf.transformations.quaternion_from_euler(0, 0, degree)
    # unpack q list by *
    goal.target_pose.pose.orientation = Quaternion(*q)
    return goal

def navigate_to(client, x, y, label, z=0.0):
    rospy.loginfo(f"Navigating to {label}: ({x}, {y})")
    goal = make_goal(x, y, z)
    client.send_goal(goal)
    client.wait_for_result()
    state = client.get_state()
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"Reached {label}!")
        return True
    else:
        rospy.logwarn(f"Failed to reach {label}, state: {state}")
        return False

def main():
    rospy.init_node('p2a_navi')
    # 'move_base' is the ROS node, MoveBaseAction is the type of message
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base!")

    # ---- DEFINE L1, L2, L3 HERE ----
    L1 = (0.106, -0.044)
    L2 = (4.194, -0.214)
    L3 = (3.011, -3.746)
    # ---------------------------------

    set_initial_pose(L1[0], L1[1])

    success = True

    if not navigate_to(client, L1[0], L1[1], "L1 (start)"):
        success = False
    rospy.sleep(1.0)

    if not navigate_to(client, L2[0], L2[1], "L2"):
        success = False
    rospy.sleep(1.0)

    if not navigate_to(client, L3[0], L3[1], "L3"):
        success = False
    rospy.sleep(1.0)

    if not navigate_to(client, L1[0], L1[1], "L1 (return)"):
        success = False

    if success:
        rospy.loginfo("Mission complete: L1 → L2 → L3 → L1")
    else:
        rospy.logerr("Mission failed! Some waypoints not reached.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
