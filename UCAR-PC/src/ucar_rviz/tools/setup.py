#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped

def clear_costmaps():
    """Call the move_base clear_costmaps service."""
    try:
        rospy.wait_for_service('/move_base/clear_costmaps', timeout=5.0)
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps_service()
        rospy.loginfo("Successfully cleared costmaps")
        return True
    except rospy.ROSException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return False

def set_initial_pose():
    """Publish initial pose estimate to /initialpose topic."""
    # Create the publisher for initial pose
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    # Wait a moment for publisher to register
    rospy.sleep(0.5)
    
    # Create PoseWithCovarianceStamped message
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"  # Set appropriate frame
    pose.header.stamp = rospy.Time.now()
    
    # Set position
    pose.pose.pose.position.x = 0.25
    pose.pose.pose.position.y = 0.25
    pose.pose.pose.position.z = 0.0
    
    # Set orientation (as quaternion, assuming yaw=0)
    pose.pose.pose.orientation.x = 0.0
    pose.pose.pose.orientation.y = 0.0
    pose.pose.pose.orientation.z = 0.0
    pose.pose.pose.orientation.w = 1.0
    
    # Set covariance
    # Diagonal values for x, y, z, roll, pitch, yaw
    pose.pose.covariance[0] = 0.0025  # x
    pose.pose.covariance[7] = 0.0025  # y
    pose.pose.covariance[35] = 0.0171  # yaw
    
    # Publish the message
    pub.publish(pose)
    rospy.loginfo("Initial pose has been published")
    
    # Give some time for the message to be processed
    rospy.sleep(0.5)

if __name__ == "__main__":
    try:
        # Initialize ROS node
        rospy.init_node('setup_robot_pose', anonymous=True)
        
        # FIRST: Set and publish initial pose
        set_initial_pose()
        
        # SECOND: Clear the costmaps
        if clear_costmaps():
            rospy.loginfo("Costmaps cleared successfully")
        else:
            rospy.logwarn("Failed to clear costmaps")
        
        rospy.loginfo("Setup complete!")
    
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupted")
        pass
    except Exception as e:
        rospy.logerr("Error occurred: {}".format(e))