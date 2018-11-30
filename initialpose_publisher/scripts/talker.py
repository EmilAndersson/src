#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from multi_level_map_msgs.msg import LevelMetaData, MultiLevelMapData
from multi_level_map_msgs.srv import ChangeCurrentLevel, ChangeCurrentLevelRequest

def change_map(level):
    if req.new_level_id ==   


	

def talker():
    pub = rospy.Publisher('initialpose',PoseWithCovarianceStamped , queue_size=10)
    change_floor  = rospy.Publisher('level_changer',ChangeCurrentLevelRequest , queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    
    current_pose = PoseWithCovarianceStamped()

    goalx = 0
    goaly = 0

    origin_pose1 = PoseWithCovarianceStamped()
    origin_pose1.header.frame_id = "" #"level_mux_map" # frameIdFromLevelId(source.text())
    origin_pose1.pose.pose.orientation.w = 0.896705824116    # Makes the origin quaternion valid.
    origin_pose1.pose.pose.orientation.z = 0.44262700437
    origin_pose1.pose.pose.position.x = 1.05531990528
    origin_pose1.pose.pose.position.y = -0.843991756439
    origin_pose1.pose.pose.position.z = 0.0
    origin_pose1.pose.covariance[0] = 1.0
    origin_pose1.pose.covariance[7] = 1.0
    origin_pose1.pose.covariance[14] = 1.0
    origin_pose1.pose.covariance[21] = 1.0
    origin_pose1.pose.covariance[28] = 1.0
    origin_pose1.pose.covariance[35] = 1.0 

    origin_pose2 = PoseWithCovarianceStamped()
    origin_pose2.header.frame_id = "map" #"level_mux_map" # frameIdFromLevelId(source.text())
    origin_pose2.pose.pose.orientation.w = 0.896705824116    # Makes the origin quaternion valid.
    origin_pose2.pose.pose.orientation.z = 0.44262700437
    origin_pose2.pose.pose.position.x = 1.05531990528
    origin_pose2.pose.pose.position.y = -0.843991756439
    origin_pose2.pose.pose.position.z = 0.0
    origin_pose2.pose.covariance[0] = 1.0
    origin_pose2.pose.covariance[7] = 1.0
    origin_pose2.pose.covariance[14] = 1.0
    origin_pose2.pose.covariance[21] = 1.0
    origin_pose2.pose.covariance[28] = 1.0
    origin_pose2.pose.covariance[35] = 1.0 

    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,current_pose_saver)


    
   

    # Create an object of the type ChangeCurrentRequest
    req = ChangeCurrentLevelRequest()
    req.new_level_id = 'level1'
    req.publish_initial_pose = True

    while not rospy.is_shutdown():
	# ToDo:
	# Add if statement and execute the following when the robot is at the right position (elevator)
	if current_pose.pose.pose.position.x <= goalx + 2  and current_pose.pose.pose.position.x > goalx - 2 and current_pose.pose.pose.position.y < goaly + 2 and current_pose.pose.pose.position.y > goaly - 2:
		req.new_level_id = 'level2'	
		req.initial_pose = origin_pose2
		rospy.loginfo(req)
		change_floor.publish(req)
		rate.sleep()


		

 	#origin_pose.header.stamp = rospy.get_rostime()
        #rospy.loginfo(origin_pose)
        # pub.publish(origin_pose) # Currently not in use since the pose estimate is updated via the req
	# Publishing a request on the topic 'level_changer', leading to a change of the map in the level_mux node
	#req.initial_pose = origin_pose
	#rospy.loginfo(req)
	#change_floor.publish(req)
        #rate.sleep()


    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
