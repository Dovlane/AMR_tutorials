#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from move_robot.srv import move_robot_service, move_robot_serviceResponse
import math
import copy

#First msg receive flag
first_msg_received = False

#Make global variable for storing robot position
robotPose = Pose() 

# Collect data from /odom topic
def odometryCallback(dataOdom): 
    global robotPose 
    global first_msg_received
    robotPose = dataOdom.pose.pose
    first_msg_received = True

# Move robot when requested
def move_robot_go(dataSrv):
    global robotPose
    
    #Load data from service request
    x_speed = dataSrv.x_speed
    angular_speed = dataSrv.angular_speed

    #Creat init robot position from curent robot position
    init_pose = Pose()
    init_pose.position.x = copy.copy(robotPose.position.x)
    init_pose.position.y = copy.copy(robotPose.position.y)

    #Make 10Hz timer
    r = rospy.Rate(10)

    #Run robot while travel distvance is less then 1m #This was fixed
    while (math.sqrt(math.pow(robotPose.position.x - init_pose.position.x,2)+math.pow(robotPose.position.y - init_pose.position.y,2))<=1.0):
        rospy.loginfo(robotPose)
        #Create msg to be publish
        pub_cmd = Twist()
        pub_cmd.linear.x = x_speed
        pub_cmd.linear.y = 0
        pub_cmd.linear.z = 0
        pub_cmd.angular.x = 0
        pub_cmd.angular.y = 0
        pub_cmd.angular.z = angular_speed

        #Publish data
        pub.publish(pub_cmd)

        #Sleep for some time (10Hz loop)
        r.sleep()
    
    #Create blank msg
    pub_cmd = Twist()

    #Send it to stop the robot
    pub.publish(pub_cmd)
    
    #Return response to service caller
    return move_robot_serviceResponse(True)

if __name__ == '__main__':
    #Init node
    rospy.init_node("move_robot_service")

    #Create publisher to /cmd_vel topic
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    rospy.loginfo("Publisher is ready!")

    #Create subscriber to /odom
    rospy.Subscriber("odom",Odometry,odometryCallback)
    rospy.loginfo("Subscriber is ready!")

    #Create service for moveing robot
    service = rospy.Service("move_robot_service",move_robot_service,move_robot_go)
    rospy.loginfo("Service is ready!")

    #Wait for te first msg
    while not first_msg_received and not rospy.is_shutdown():
        rospy.sleep(0.01)

    rospy.spin()
