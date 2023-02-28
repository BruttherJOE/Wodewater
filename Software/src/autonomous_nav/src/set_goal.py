#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sys

## places of interest (for buttons)

# use rostopic echo /odom to find out pose at current position.


#button1
button1_position_x = 0.0
button1_position_y = 0.0
button1_position_z = 0.0

button1_orientation_x = 0.0
button1_orientation_y = 0.0
button1_orientation_z = 0.0
button1_orientation_w = 0.0

#button2
button2_position_x = 0.0
button2_position_y = 0.0
button2_position_z = 0.0

button2_orientation_x = 0.0
button2_orientation_y = 0.0
button2_orientation_z = 0.0
button2_orientation_w = 0.0

# callbacks definition

def active_cb(extra):
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    rospy.loginfo("goal executing. Current location: "+str(feedback))

def done_cb(status, result): #callback activated when goal reached, cancelled or aborted, during execution of goal
    if status == 3:
        rospy.loginfo("Goal reached")

    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")

    if status == 4:
        rospy.loginfo("Goal aborted")



if __name__ == "__main__":

    rospy.init_node('send_goal')
    #import sys, then can use arguments in the terminal, filtered by myargv with ros. cmd below does this
    args = rospy.myargv(argv=sys.argv)
    
    if len(args) != 2: 
        #when you pass arg its +1 so its 2
        #this makes it so that cmdline will only accept 1 arg
        print("ERR : wrong length of args. set where u want to go!")
        sys.exit(1)
    
    the_arg = args[1] 
    #this stores the argument from user in cmdline in this var. arg[0] is the cmd itself.
    #this line is put here so that it executes after the !=2 check

    if the_arg == 'bed1':
        pos_x = button1_position_x
        pos_y = button1_position_y
        pos_z = button1_position_z

        ori_x = button1_orientation_x
        ori_y = button1_orientation_y
        ori_z = button1_orientation_z
        ori_w = button1_orientation_w
        rospy.sleep(0.1337)

    elif the_arg == 'bed2':
        pos_x = button1_position_x
        pos_y = button1_position_y
        pos_z = button1_position_z

        ori_x = button2_orientation_x
        ori_y = button2_orientation_y
        ori_z = button2_orientation_z
        ori_w = button2_orientation_w
        rospy.sleep(0.1337)

    else:
        print("ERR : wrong argument")
        sys.exit(1)


    navclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    navclient.wait_for_server() #check and wait till connection is made

    # eg of nav goal
    goal = MoveBaseGoal() #create movebase goal, send to action server
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = pos_x
    goal.target_pose.pose.position.y = pos_y
    goal.target_pose.pose.position.z = pos_z

    goal.target_pose.pose.orientation.x = ori_x
    goal.target_pose.pose.orientation.y = ori_y
    goal.target_pose.pose.orientation.z = ori_z
    goal.target_pose.pose.orientation.w = ori_w

    navclient.send_goal(goal, done_cb, active_cb, feedback_cb) #callback fn are not mandatory
    finished = navclient.wait_for_result()

    if not finished:
        rospy.logerr("Action server not available")
    else:
        rospy.loginfo (navclient.get_result())






