#!/usr/bin/env python3
# Plays the MATLAB-planned weld pass. Modes: trajectory (to arm_controller), jointstate (RViz replay), marker (seam only)
import argparse, bisect, csv
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
TRAJ = "/home/rajesh/implementation_of_kuka/custom_kuka_description/data/weld_trajectory.csv"
PATH = "/home/rajesh/implementation_of_kuka/custom_kuka_description/data/weld_path.csv"

def load(path):
    with open(path) as f:
        rows = list(csv.reader(f))[1:]
    return [[float(v) for v in r] for r in rows]

def dur(t):
    s = int(t)
    return Duration(sec=s, nanosec=int((t - s) * 1e9))

class Player(Node):
    def __init__(self, mode, loop):
        super().__init__("weld_player")
        self.loop = loop
        self.traj = load(TRAJ)
        self.pts = load(PATH)
        self.times = [r[0] for r in self.traj]
        if mode in ("jointstate", "marker"):
            self.mpub = self.create_publisher(Marker, "/weld_seam", 10)
            self.create_timer(1.0, self.pub_marker)
        if mode == "jointstate":
            self.jpub = self.create_publisher(JointState, "/joint_states", 10)
            self.t0 = self.get_clock().now()
            self.create_timer(0.02, self.pub_state)
        if mode == "trajectory":
            self.send()

    def pub_marker(self):
        m = Marker()
        m.header.frame_id = "world"
        m.ns = "weld"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.005
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.2
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        for x, y, z in self.pts:
            m.points.append(Point(x=x, y=y, z=z))
        self.mpub.publish(m)

    def pub_state(self):
        el = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        end = self.times[-1]
        if self.loop:
            el = el % end
        el = min(el, end)
        i = max(1, min(bisect.bisect_left(self.times, el), len(self.times) - 1))
        a, b = self.traj[i - 1], self.traj[i]
        f = 0.0 if b[0] == a[0] else (el - a[0]) / (b[0] - a[0])
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINTS
        js.position = [a[k] + f * (b[k] - a[k]) for k in range(1, 7)]
        self.jpub.publish(js)

    def send(self):
        pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.get_logger().info("waiting for arm_controller ...")
        while pub.get_subscription_count() == 0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        for r in self.traj:
            if r[0] <= 0.0:
                continue
            p = JointTrajectoryPoint()
            p.positions = r[1:7]
            p.time_from_start = dur(r[0])
            msg.points.append(p)
        pub.publish(msg)
        self.get_logger().info("weld pass sent: %d points, %.1f s" % (len(msg.points), self.traj[-1][0]))
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        raise SystemExit

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["trajectory", "jointstate", "marker"], default="trajectory")
    ap.add_argument("--loop", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    try:
        rclpy.spin(Player(a.mode, a.loop))
    except (SystemExit, KeyboardInterrupt):
        pass
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()
