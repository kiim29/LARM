import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String

class Detect(Node):

    def __init__(self):
        super().__init__('detect')

        self.create_subscription( Point, 'cola_detect_definite', self.cola_callback, 10)
        self.create_subscription( Point, 'cherry_detect_definite', self.cherry_callback, 10)

        self.bottleDetectPublisher = self.create_publisher( String, 'detection', 10)

    def cola_callback(self, pointMsg) :
        strgMsg = String()
        strgMsg.data = 'Bouteille de Nuka Cola en x = ' + str(round(pointMsg.x, 3)) + ' et y = ' + str(round(pointMsg.y, 3)) + ' détectée à ' + str(round(pointMsg.z, 3)) + 'm'
        self.bottleDetectPublisher.publish(strgMsg)

    def cherry_callback(self, pointMsg) :
        strgMsg = String()
        strgMsg.data = 'Bouteille de Nuka Cherry en x = ' + str(round(pointMsg.x, 3)) + ' et y = ' + str(round(pointMsg.y, 3)) + ' détectée à ' + str(round(pointMsg.z, 3)) + 'm'
        self.bottleDetectPublisher.publish(strgMsg)

def main(args=None):
    rclpy.init(args=args)
    detect = Detect()
    rclpy.spin(detect)
    detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()