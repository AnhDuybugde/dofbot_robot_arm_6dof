"""Stick-figure arm overlay for weak GPUs (e.g. VMware SVGA3D).

RobotModel mesh rendering can stay invisible on virtualized GL stacks while
Marker rendering works fine. This node draws the arm as red bones + green
joint spheres from live TF, published to /chess/visual so the stock RViz
config shows it next to the board without any change.
"""
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

LINKS = ["base_link", "arm1_Link", "arm2_Link", "arm3_Link",
         "arm4_Link", "arm5_Link", "Gripping_point_Link"]


class ArmSkeleton(Node):
    def __init__(self):
        super().__init__("simplify_arm_skeleton")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, "/chess/visual", qos)
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        points = []
        for link in LINKS:
            try:
                transform = self.tf.lookup_transform("base_link", link, Time())
            except Exception:
                return
            position = transform.transform.translation
            points.append(Point(x=position.x, y=position.y, z=position.z))
        bones = Marker()
        bones.header.frame_id = "base_link"
        bones.ns = "simplify_arm_bones"
        bones.id = 100
        bones.type = Marker.LINE_STRIP
        bones.action = Marker.ADD
        bones.scale.x = 0.012
        bones.color.r, bones.color.g, bones.color.b, bones.color.a = 1.0, 0.1, 0.1, 1.0
        bones.points = points
        joints = Marker()
        joints.header.frame_id = "base_link"
        joints.ns = "simplify_arm_joints"
        joints.id = 101
        joints.type = Marker.SPHERE_LIST
        joints.action = Marker.ADD
        joints.scale.x = joints.scale.y = joints.scale.z = 0.022
        joints.color.r, joints.color.g, joints.color.b, joints.color.a = 0.1, 1.0, 0.1, 1.0
        joints.points = points
        self.publisher.publish(MarkerArray(markers=[bones, joints]))


def main():
    rclpy.init()
    node = ArmSkeleton()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
