import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from marker import Marker

class MarkerNode(Node):

    def __init__(self):
        super().__init__('marker_node')

        self.create_subscription( Point, 'cola_detect_definite', self.callback, 10)
        self.create_subscription( Point, 'cherry_detect_definite', self.callback, 10)

        self.markerPublisher = self.create_publisher( Marker, 'marker', 10)

    def callback(self, localPoint) :
        globalPoint = Marker()
        globalPoint.pose.position = localPoint
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