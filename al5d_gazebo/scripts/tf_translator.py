#!/usr/bin/python2

import rospy
import tf2_ros
import geometry_msgs.msg
from al5d_gazebo.msg import TransformStampedList

JOINT_NAMES = ["upper_base", "upper_arm", "lower_arm", "wrist", "gripper_base", "end"]

#This is a somewhat hacky buffer to publish joint positions from tf

class TfTranslator:
    def __init__(self):
        rospy.init_node('tf_translator', anonymous=True)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.tf_cb, oneshot=False)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.trans_pub = rospy.Publisher("joint_poses", TransformStampedList, queue_size=1)

    def tf_cb(self, timer):
        joint_poses = TransformStampedList()
        rospy.sleep(1)
        try:
            for joint in JOINT_NAMES:
                trans = self.tf_buffer.lookup_transform("base", joint, rospy.Time())
                joint_poses.transforms.append(trans)
        except:
            pass

        if len(joint_poses.transforms) == len(JOINT_NAMES):
            self.trans_pub.publish(joint_poses)

if __name__=='__main__':
    trans = TfTranslator()
    rospy.spin()
