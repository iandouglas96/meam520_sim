#!/usr/bin/env python

import rospy
from al5d_gazebo.msg import TransformStampedList
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
from Queue import Queue
import numpy as np
from tf.transformations import quaternion_matrix
from copy import deepcopy
from time import sleep

np.set_printoptions(suppress=True)

from std_msgs.msg import Empty, Bool, Header
from sensor_msgs.msg import JointState
# from std_srvs.srv import Empty, EmptyResponse
# from std_srvs.srv import Trigger, TriggerResponse
# from al5d_gazebo.srv import ArmCommand, ArmCommandResponse

#class managing separate ROS loop
class ROSInterface:
    def __init__(self, cmd_q, status_q, pose_q, num_joints):
        #load up queues for async comm
        self.cmd_q = cmd_q
        self.status_q = status_q
        self.pose_q = pose_q
        self.num_joints = num_joints
        self.pos = None
        self.vel = None
        self.move_ind = 0
        self.move_seq = 0
        self.vel_target = None

    def state_cb(self, state):
        #add state to queue
        #note in original message joints are in alphabetical order
        #here we reorder to agree with command format
        self.pos = np.array([state.position[0],
                             state.position[4],
                             state.position[1],
                             state.position[6],
                             state.position[5],
                             state.position[2]])
        self.vel = np.array([state.velocity[0],
                             state.velocity[4],
                             state.velocity[1],
                             state.velocity[6],
                             state.velocity[5],
                             state.velocity[2]])

        #clear queue
        while not self.status_q.empty():
            self.status_q.get()
        self.status_q.put((self.pos, self.vel))

    def pose_cb(self, pose):
        #we know the transforms are already in the order we want
        transforms = []
        for trans in pose.transforms:
            translation = 1000*np.array([trans.transform.translation.x,
                                         trans.transform.translation.y,
                                         trans.transform.translation.z])



            matrix = quaternion_matrix([trans.transform.rotation.x,
                                           trans.transform.rotation.y,
                                           trans.transform.rotation.z,
                                           trans.transform.rotation.w])
            matrix[:3,3] = translation
            transforms.append(matrix)

        #clear queue
        while not self.pose_q.empty():
            self.pose_q.get()
        self.pose_q.put(transforms)

    def start_command(self, state):
        if state[0] == "move":
            self.vel_target = None
            if self.pos is not None:
                #interpolate states between current state and goal
                diff = state[1] - self.pos
                dist = np.max(np.abs(diff))
                interp = np.linspace(0, 1, max(np.ceil(80*dist), 2))
                self.move_seq = self.pos + interp[:,None]*diff[None,:]
                self.move_ind = 0
        elif state[0] == "velocity":
            self.move_seq = 0
            self.vel_target = deepcopy(state[1])
            self.pos_target = deepcopy(self.pos)
            rospy.loginfo(self.vel_target)

    def update_move(self):
        if self.move_seq is not 0:
            #if we are interpolating, move to next state
            if self.move_ind < self.move_seq.shape[0]:
                self.set_state(self.move_seq[self.move_ind,:])
                self.move_ind += 1
            else:
                self.move_ind = 0
                self.move_seq = 0
        elif self.vel_target is not None:
            dt = self.rate.sleep_dur.to_sec()
            self.pos_target = self.pos_target + dt * np.array(self.vel_target)
            self.set_state(self.pos_target)


    def set_state(self, state):
        for ind, s in enumerate(state[:-1]):
            msg = Float64(s)
            self.pos_pubs[ind].publish(msg)

        self.gripper_pubs[0].publish(Float64(-state[-1]))
        self.gripper_pubs[1].publish(Float64(state[-1]))

    def loop(self):
        self.node = rospy.init_node('arm_controller', disable_signals=True)
        self.rate = rospy.Rate(50)
        self.pos_pubs = []
        self.gripper_pubs = []
        for i in range(self.num_joints):
            pub = rospy.Publisher("/al5d_arm_position_controller"+str(i+1)+"/command",
                                  Float64, queue_size=1, latch=True)
            self.pos_pubs.append(pub)
        for i in range(2):
            pub = rospy.Publisher("/al5d_gripper_controller"+str(i+1)+"/command",
                                  Float64, queue_size=1, latch=True)
            self.gripper_pubs.append(pub)

        state_sub = rospy.Subscriber("/joint_states", JointState, self.state_cb)
        pose_sub = rospy.Subscriber("/joint_poses", TransformStampedList, self.pose_cb)

        #poll at 50Hz

        while not rospy.is_shutdown():
            try:
                if not self.status_q.empty() and not self.pose_q.empty():
                    if not self.cmd_q.empty():
                        cmd = self.cmd_q.get()
                        if cmd == "stop":
                            break
                        self.start_command(cmd)
                    self.update_move()
                else:
                    pass
                self.rate.sleep()
            except KeyboardInterrupt:
                break

class ArmController:
    def __init__(self, num_joints=5):
        self.num_joints = num_joints
        self.cur_state = ()
        self.cur_pose = ()
        self.cmd_q = Queue()
        self.status_q = Queue()
        self.pose_q = Queue()

        self.joint_limits = np.array([[-1.4, 1.4],
                                      [-1.2, 1.4],
                                      [-1.8, 1.7],
                                      [-1.9, 1.7],
                                      [-2, 1.5],
                                      [-15, 30]]).T

        #ROS runs in a separate thread because it needs to always
        #check the message buffers
        rospy.loginfo("Starting ros thread...")
        self.ros = ROSInterface(self.cmd_q, self.status_q, self.pose_q, num_joints)
        self.spin_t = threading.Thread(target=self.ros.loop)
        self.spin_t.start()
        rospy.loginfo("Ros thread started")

    def set_vel(self, state):
        scaled_state = deepcopy(state)
        scaled_state[-1] = (-scaled_state[-1])/45.*0.03
        self.cmd_q.put(("velocity", scaled_state))

    def set_tau(self, state):
        rospy.logwarn("Torque control not yet implemented")

    def set_pos(self, state):
        self.set_state(state)

    def set_state(self, state):
        #num joints + gripper
        if len(state) == self.num_joints + 1:
            scaled_state = deepcopy(state)
            #check limits
            if np.any(scaled_state < self.joint_limits[0]):
                bad_joints = np.where(scaled_state < self.joint_limits[0])[0]
                for bad_joint in bad_joints:
                    rospy.logwarn("Joint " + str(bad_joint) + " is below the limit " +
                          str(self.joint_limits[0, bad_joint]))
                scaled_state = np.maximum(scaled_state, self.joint_limits[0])
            if np.any(scaled_state > self.joint_limits[1]):
                bad_joints = np.where(scaled_state > self.joint_limits[1])[0]
                for bad_joint in bad_joints:
                    rospy.logwarn("Joint " + str(bad_joint) + " is above the limit " +
                          str(self.joint_limits[1, bad_joint]))
                scaled_state = np.minimum(scaled_state, self.joint_limits[1])


            scaled_state[-1] = (-scaled_state[-1]+30.)/45.*0.03
            self.cmd_q.put(("move", scaled_state))

        else:
            raise Exception("Invalid state command")

    def get_state(self):

        # output: the joint value and its velocity
        if not self.status_q.empty():
            self.cur_state = self.status_q.queue[0]


        scaled_state = []
        scaled_state.append(deepcopy(self.cur_state[0]))
        scaled_state.append(deepcopy(self.cur_state[1]))
        scaled_state[0][-1] = -scaled_state[0][-1]*45./0.03 + 30.
        scaled_state[1][-1] = -scaled_state[1][-1]*45./0.03

        pos = np.around(scaled_state[0], decimals=3).tolist()
        vel = np.around(scaled_state[1], decimals=3).tolist()

        return pos, vel

    def get_poses(self):

        # output: the T matrix of each joint
        if not self.pose_q.empty():
            self.cur_pose = self.pose_q.queue[0]

        return np.around(self.cur_pose, decimals=3)

    def stop(self):
        self.cmd_q.put("stop")
        self.spin_t.join()

    def is_stopped(self, tolerance=1e-2):
        pos, vel = self.get_state()
        vel = np.array(vel)
        norm = np.linalg.norm(vel * (2*3.14159/30), np.inf) # fairly weight the gripper

        return norm < tolerance and self.ros.move_seq is 0 and self.ros.cmd_q.empty()
        # if we are interpolating, we are not done
        # if we have a target velocity, we may be up against joint limits,
        # so we may still be stopped

    def is_collided(self):
        # TODO: implement
        return False

if __name__ == '__main__':
    try:

        lynx = ArmController()

        def position(msg):
            pos = np.array(msg.position)
            lynx.set_pos(pos)

        def velocity(msg):
            vel = np.array(msg.velocity)
            lynx.set_vel(vel)

        def torque(msg):
            tau = np.array(msg.effort)
            lynx.set_tau(tau)

        def stop(msg):
            lynx.stop()


        # Wait for ROS
        rospy.sleep(rospy.Duration.from_sec(3))

        pos_sub = rospy.Subscriber("/arm_interface/position", JointState, position)
        vel_sub = rospy.Subscriber("/arm_interface/velocity", JointState, velocity)
        torque_sub = rospy.Subscriber("/arm_interface/effort", JointState, torque)
        stop_sub = rospy.Subscriber("/arm_interface/stop", Empty, stop)

        collision_pub = rospy.Publisher("/arm_interface/collided", Bool, queue_size=1, latch=True)
        stopped_pub = rospy.Publisher("/arm_interface/stopped", Bool, queue_size=1, latch=True)
        joint_pub = rospy.Publisher("/arm_interface/state", JointState, queue_size=1, latch=True)

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            try:
                collision_pub.publish(Bool(lynx.is_collided()))
                stopped_pub.publish(Bool(lynx.is_stopped()))

                # we republish joint data due to reordering and scaling
                msg = JointState();
                pos, vel = lynx.get_state()
                msg.position = pos
                msg.velocity = vel
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'world'
                msg.name = ["upper_base", "upper_arm", "lower_arm", "wrist", "gripper_base", "end"];
                joint_pub.publish(msg)

                rate.sleep()
            except KeyboardInterrupt:
                break


    except rospy.ROSInterruptException:
        lynx.stop()
        pass
