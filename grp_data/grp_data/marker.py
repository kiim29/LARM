import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class MarkerNode(Node):

    def __init__(self):
        super().__init__('marker_node')

        self.create_subscription( Point, 'cola_detect_definite', self.callback, 10)
        self.create_subscription( Point, 'cherry_detect_definite', self.callback, 10)

        self.markerPublisher = self.create_publisher( Marker, 'marker', 10)

    def callback(self, localPoint) :
        globalPoint = Marker()
        globalPoint.pose.position.x = 0.0
        globalPoint.pose.position.y = 0.0
        globalPoint.pose.position.z = 0.0
        globalPoint.scale.x = 0.2
        globalPoint.scale.y = 0.2
        globalPoint.scale.z = 1.0
        globalPoint.type = 1
        globalPoint.lifetime.sec = 60
        globalPoint.color.r = 1.0
        globalPoint.color.a = 1.0
        globalPoint.header.stamp = self.get_clock().now().to_msg()
        globalPoint.header.frame_id = "base_link"
        self.markerPublisher.publish(globalPoint)

def main(args=None):
    rclpy.init(args=args)
    marker = MarkerNode()
    rclpy.spin(marker)
    marker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
