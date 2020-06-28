import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
from Queue import Queue
import numpy as np

#class managing separate ROS loop
class ROSInterface:
    def __init__(self, cmd_q, status_q, num_joints):
        #load up queues for async comm
        self.cmd_q = cmd_q
        self.status_q = status_q
        self.num_joints = num_joints
        
    def state_cb(self, state):
        #add state to queue
        #note in original message joints are in alphabetical order
        #here we reorder to agree with command format
        pos = np.array([state.position[0],
                        state.position[4],
                        state.position[1],
                        state.position[6],
                        state.position[5]])
        vel = np.array([state.velocity[0],
                        state.velocity[4],
                        state.velocity[1],
                        state.velocity[6],
                        state.velocity[5]])

        #clear queue
        while not self.status_q.empty():
            self.status_q.get()
        self.status_q.put((pos, vel))

    def send_command(self, state):
        if state[0] == "arm":
            for ind, s in enumerate(state[1]):
                msg = Float64(s)
                self.pos_pubs[ind].publish(msg)
        elif state[0] == "gripper":
            if state[1] == True:
                #open
                self.gripper_pubs[0].publish(Float64(-1))
                self.gripper_pubs[1].publish(Float64(1))
            else:
                #close
                self.gripper_pubs[0].publish(Float64(1))
                self.gripper_pubs[1].publish(Float64(-1))

    def loop(self):
        node = rospy.init_node('arm_controller', disable_signals=True)
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

        #poll at 100Hz
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                if not self.cmd_q.empty():
                    cmd = self.cmd_q.get()
                    if cmd == "stop":
                        break
                    self.send_command(cmd)
                rate.sleep()
            except KeyboardInterrupt:
                break

class ArmController:
    def __init__(self, num_joints=5):
        self.num_joints = num_joints
        self.cur_state = ()
        self.cmd_q = Queue()
        self.status_q = Queue()

        #ROS runs in a separate thread because it needs to always
        #check the message buffers
        print("starting ros thread...")
        self.ros = ROSInterface(self.cmd_q, self.status_q, num_joints)
        self.spin_t = threading.Thread(target=self.ros.loop)
        self.spin_t.start()
        print("ros thread started")

    def set_gripper(self, do_open):
        self.cmd_q.put(("gripper", do_open))

    def set_state(self, state):
        if len(state) == self.num_joints+1:
            self.cmd_q.put(("arm", state))
        else:
            raise Exception("Invalid state command")

    def get_state(self):
        if not self.status_q.empty():
            self.cur_state = self.status_q.get()

        return self.cur_state
    
    def stop(self):
        self.cmd_q.put("stop")
        self.spin_t.join()
