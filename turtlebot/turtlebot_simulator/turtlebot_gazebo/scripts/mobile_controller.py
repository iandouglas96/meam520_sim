import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading
from queue import Queue
import numpy as np

#class managing separate ROS loop
class ROSInterface:
    def __init__(self, cmd_q, scan_q):
        #load up queues for async comm
        self.cmd_q = cmd_q
        self.scan_q = scan_q
        self.have_scan = False
        
    def scan_cb(self, scan):
        #clear queue
        while not self.scan_q.empty():
            self.scan_q.get()
        self.scan_q.put(scan.ranges)
        self.have_scan = True

    def send_command(self):
        twist = Twist()
        twist.linear.x = self.last_cmd[0]
        twist.linear.y = self.last_cmd[1]
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.last_cmd[2]

        self.twist_pub.publish(twist)

    def loop(self):
        node = rospy.init_node('arm_controller', disable_signals=True)
        self.twist_pub = rospy.Publisher("/mobile_base/commands/velocity", 
                                         Twist, queue_size=1, latch=True)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.last_cmd = [0, 0, 0] 

        #poll at 100Hz
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                if self.have_scan:
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

class MobileController:
    def __init__(self):
        self.cur_scan = ()
        self.cmd_q = Queue()
        self.scan_q = Queue()

        #ROS runs in a separate thread because it needs to always
        #check the message buffers
        print("starting ros thread...")
        self.ros = ROSInterface(self.cmd_q, self.scan_q)
        self.spin_t = threading.Thread(target=self.ros.loop)
        self.spin_t.start()
        print("ros thread started")

    def set_vel(self, vel):
        if len(vel) == 3:
            self.cmd_q.put(vel)
        else:
            raise Exception("Invalid velocity command")

    def get_scan(self):
        if not self.scan_q.empty():
            self.cur_scan = np.array(self.scan_q.get())

        return self.cur_scan
    
    def stop(self):
        self.cmd_q.put("stop")
        self.spin_t.join()
