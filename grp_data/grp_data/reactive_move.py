import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import random

class ReactiveMove(Node):

    def __init__(self):
        super().__init__('move')
        self.velocity_publisher = self.create_publisher(Twist, '/commands/velocity', 10)
        self.consigne='avance'
        self.create_subscription( PointCloud, 'cloud', self.control, 10)

    def avance(self):
        velo = Twist()
        velo.linear.x = 0.3
        self.velocity_publisher.publish(velo)

    def tourneGauche(self):
        velo = Twist()
        velo.angular.z = 1.0
        self.velocity_publisher.publish(velo)

    def tourneDroite(self):
        velo = Twist()
        velo.angular.z = -1.0
        self.velocity_publisher.publish(velo)

    def activate(self):
        if self.consigne=='avance' :
            self.avance()
        elif self.consigne=='gauche' :
            self.tourneGauche()
        elif self.consigne=='droite' :
            self.tourneDroite()
        elif self.consigne=='stop' :
            self.stop()
        else :
            self.stop()

    def stop(self):
        velo = Twist()
        velo.angular.z = 0.0
        velo.linear.x = 0.0
        self.velocity_publisher.publish(velo)
        
    def control(self, pointcloud):
        obstacleGauche = False
        obstacleDroite = False
        for point in pointcloud.points:
            if 0.05 <= point.x <= 0.2 and 0.2 >= point.y >= 0.05:
                obstacleGauche = True
            elif 0.05 <= point.x <= 0.2 and -0.2 <= point.y <= -0.05:
                obstacleDroite = True
        if obstacleDroite and obstacleGauche:
            if self.consigne == 'avance':
                if random.random() < 0.5:
                    self.consigne = 'gauche'
                else:
                    self.consigne = 'droite'
        elif obstacleDroite: 
            if self.consigne == 'avance':
                self.consigne = 'gauche'
        elif obstacleGauche: 
            if self.consigne == 'avance':
                self.consigne = 'droite'
        elif self.consigne == ' avance' and random.random()<0.05:
            self.consigne = 'droiteLongtemps'
            compteur = random.randint(10,30)
        elif self.consigne == 'droiteLongtemps' and compteur > 0:
            compteur = compteur-1
        else:
            self.consigne = 'avance'
        self.activate()


def main(args=None):
    rclpy.init(args=args)
    move = ReactiveMove()

    # Start the ros infinit loop with the move node.
    rclpy.spin(move)

    # At the end, destroy the node explicitly.
    move.destroy_node()

    # and shut the light down.
    rclpy.shutdown()

if __name__ == '__main__':
    main()