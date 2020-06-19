import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
import threading
from queue import Queue
import numpy as np
from scipy.spatial.transform import Rotation as R

#class managing separate ROS loop
class ROSInterface:
    def __init__(self, cmd_q, scan_q, state_q):
        #load up queues for async comm
        self.cmd_q = cmd_q
        self.scan_q = scan_q
        self.state_q = state_q
        self.have_scan = False
        self.have_state = False
        
    def scan_cb(self, scan):
        #clear queue
        while not self.scan_q.empty():
            self.scan_q.get()
        self.scan_q.put(scan.ranges)
        self.have_scan = True
        
    def state_cb(self, msg):
        #clear queue
        while not self.state_q.empty():
            self.state_q.get()
        self.state_q.put({'pose':msg.pose[-1], 'twist':msg.twist[-1]})  
        self.have_state = True

    def send_command(self):
        twist = Twist()
        twist.linear.x = self.last_cmd[0]
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.last_cmd[1]

        self.twist_pub.publish(twist)

    def loop(self):
        rospy.init_node('mobile_controller', disable_signals=True)
        self.twist_pub = rospy.Publisher("/mobile_base/commands/velocity", 
                                         Twist, queue_size=1, latch=True)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.state_sub = rospy.Subscriber("/gazebo/model_states", 
                                          ModelStates, self.state_cb)
        self.last_cmd = [0, 0, 0] 

        #poll at 100Hz
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                if self.have_scan and self.have_state:
                    if not self.cmd_q.empty():
                        self.last_cmd = self.cmd_q.get()
                        if self.last_cmd == "stop":
                            break
                    self.send_command()
                else:
                    print("Waiting for Gazebo...")
                rate.sleep()
            except KeyboardInterrupt:
                break

class MobileRobot():
    def __init__(self):
        self.cur_scan = ()
        self.cur_state = []
        self.cmd_q = Queue()
        self.scan_q = Queue()
        self.state_q = Queue()
        
        # intrinsic properties
        self.r = 0.035
        self.d = 0.23

        #ROS runs in a separate thread because it needs to always
        #check the message buffers
        print("starting ros thread...")
        self.ros = ROSInterface(self.cmd_q, self.scan_q, self.state_q)
        self.spin_t = threading.Thread(target=self.ros.loop)
        self.spin_t.start()
        print("ros thread started")
        while self.scan_q.empty() or self.state_q.empty(): pass

    def set_wheel_vel(self, vel):
        if len(vel) == 2:
            v_f = -self.r * np.mean(vel)
            yaw_dot = self.r * (vel[0] - vel[1]) / self.d
            self.cmd_q.put([v_f, yaw_dot])
        else:
            raise Exception("Invalid velocity command")

    def get_scan(self):
        if not self.scan_q.empty():
            self.cur_scan = np.array(self.scan_q.get())
        return self.cur_scan
    
    def get_state(self):
        if not self.state_q.empty():
            state = self.state_q.get()
            quaternion = lambda o: np.array([o.x, o.y, o.z, o.w])
            rot_mat = R.from_quat(quaternion(state['pose'].orientation))
            x = np.array([state['pose'].position.x, state['pose'].position.y,
                          R.as_euler(rot_mat, 'zyx')[0]])
            xdot = np.array([state['twist'].linear.x, state['twist'].linear.y,
                             state['twist'].angular.z])
            self.cur_state = [x, xdot]
        return self.cur_state[0], self.cur_state[1]
    
    def get_params(self):
        return {'wheel-radius': self.r, 'axle-width': self.d}
    
    def stop(self):
        self.cmd_q.put("stop")
        self.spin_t.join()
