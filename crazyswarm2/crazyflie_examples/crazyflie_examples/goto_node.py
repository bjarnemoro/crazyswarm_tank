import sys
import signal
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool

from crazyflie_interfaces.srv import Arm, GoTo, Land, Takeoff
from crazyflie_py.crazyflie import TimeHelper
from std_srvs.srv import Empty

TAKEOFF_DURATION = 3.0
GOTO_DURATION = 5.0

def arrayToGeometryPoint(a):
    result = Point()
    result.x = a[0]
    result.y = a[1]
    result.z = a[2]
    return result

class Goto(Node):
    def __init__(self):
        super().__init__("goto_node")
        self.time_helper = TimeHelper(self)
        self.landing_pad = np.array([0.0, 0.0, 1.0])
        self.start_cf_pos = None

        self.aborted = False

        self.state = 0

        prefix = '/cf06'

        self.sub = self.create_subscription(PoseStamped, '/landing_pad/pose', self.landing_pad_pose, 10)
        self.odom_sub = self.create_subscription(PoseStamped, prefix + '/pose', self.cf_pose, 10)
        self.sub2 = self.create_subscription(Bool, '/abort_goto', self.abort, 10)

        
        self.get_logger().info(f"here")
        self.emergencyService = self.create_client(Empty, prefix + '/emergency')
        while not self.emergencyService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('1 service not available, waiting again...')
        self.takeoffService = self.create_client(Takeoff, prefix + '/takeoff')
        while not self.takeoffService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('2 service not available, waiting again...')
        self.landService = self.create_client(Land, prefix + '/land')
        while not self.landService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('3 service not available, waiting again...')
        self.goToService = self.create_client(GoTo, prefix + '/go_to')
        while not self.goToService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('4 service not available, waiting again...')
        self.get_logger().info(f"here2")


        self.main_action()

    def abort(self, msg):
        
        if not self.aborted:
            self.get_logger().info(f"aborting!!!")
            self.aborted = True

            if self.start_cf_pos is not None:
                self.goTo(self.start_cf_pos, 0.0, GOTO_DURATION)
                self.time_helper.sleep(GOTO_DURATION)
                self.get_logger().info(f"aborting with cf pos!!!")
            if self.start_cf_pos is None:
                self.goTo([0.0, 0.0, 1.0], 0.0, GOTO_DURATION)
                self.time_helper.sleep(GOTO_DURATION)
                self.get_logger().info(f"aborting without cf pos!!!")

            self.land(targetHeight=0.04, duration=TAKEOFF_DURATION)
            self.time_helper.sleep(TAKEOFF_DURATION)

    def main_action(self):
        self.takeoff(1.0, TAKEOFF_DURATION)
        self.time_helper.sleep(TAKEOFF_DURATION)

        self.state = 1

        self.goTo(self.landing_pad, 0.0, GOTO_DURATION)
        self.time_helper.sleep(GOTO_DURATION)

        self.state = 0
        if not self.aborted:
            self.land(targetHeight=0.04, duration=TAKEOFF_DURATION)
            self.time_helper.sleep(TAKEOFF_DURATION)

    def landing_pad_pose(self, msg):
        self.landing_pad[0] = msg.pose.position.x
        self.landing_pad[1] = msg.pose.position.y
        #self.landing_pad[2] = msg.pose.position.z

    def cf_pose(self, msg):
        if self.start_cf_pos is None:
            self.start_cf_pos = np.array([msg.pose.position.x, msg.pose.position.y, 1.0])
            self.get_logger().info(f"added initial pos {self.start_cf_pos}")

    def takeoff(self, targetHeight, duration, groupMask=0):
        req = Takeoff.Request()
        req.group_mask = groupMask
        req.height = targetHeight
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.takeoffService.call_async(req)

    def land(self, targetHeight, duration, groupMask=0):
        req = Land.Request()
        req.group_mask = groupMask
        req.height = targetHeight
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.landService.call_async(req)

    def goTo(self, goal, yaw, duration, relative=False, groupMask=0):
        req = GoTo.Request()
        req.group_mask = groupMask
        req.relative = relative
        req.goal = arrayToGeometryPoint(goal)
        req.yaw = float(yaw)
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        self.goToService.call_async(req)

def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    agents = Goto()

    executor = MultiThreadedExecutor()
    executor.add_node(agents)

    executor.spin()

    agents.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()