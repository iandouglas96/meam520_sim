#!/usr/bin/python2

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node('insert_object',log_level=rospy.INFO)



f = open('../models/obs2/model.sdf', 'r')
sdff = f.read()
f.close()


rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("obs2", sdff, "~/SpawnedObjects", initial_pose, "world")
