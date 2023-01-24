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
        self.consigne='stop'
        self.consigne_timeout = 0
        self.rand_target = 0.0
        self.create_subscription( PointCloud, 'cloud', self.control, 10)
        self.timer = self.create_timer(0.1, self.activate)
        self.randtimer = self.create_timer(45.0, self.randomise) 
        self.velo = Twist()

    def avance(self):
        if self.velo.linear.x <= 0.35:
            self.velo.linear.x += 0.03

    def tourneGauche(self):
        if self.velo.angular.z <= 1.0:
            self.velo.angular.z += 0.2

    def tourneDroite(self):
        if self.velo.angular.z >= -1.0:
            self.velo.angular.z -= 0.2
    
    def droitDevant(self):
        if self.velo.angular.z >= 0.05:
            self.velo.angular.z -= 0.1
        elif self.velo.angular.z < -0.05:
            self.velo.angular.z += 0.1
        elif self.velo.angular.z != 0.0:
            self.velo.angular.z = 0.0

    def activate(self):
        if self.rand_target > 1.0:
            self.consigne = 'gauche'
            self.rand_target -= 1
        elif self.rand_target < -1.0:
            self.consigne = 'droite'
            self.rand_target += 1
        else:
            self.rand_target = 0.0

        if self.consigne=='avance' :
            self.avance()
            self.droitDevant()
        elif self.consigne=='gauche' :
            self.tourneGauche()
            self.stop()
        elif self.consigne=='droite' :
            self.tourneDroite()
            self.stop()
        elif self.consigne=='courbeGauche':
            self.tourneGauche()
            self.avance()
        elif self.consigne=='courbeDroite':
            self.tourneDroite()
            self.avance()
        elif self.consigne=='stop' :
            self.stop()
        else :
            self.stop()
        if self.consigne_timeout >= 30:
            self.consigne = 'stop'
        else:
            self.consigne_timeout += 1
        self.velocity_publisher.publish(self.velo)

    def stop(self):
        if self.velo.linear.x >= 0.1:
            self.velo.linear.x -= 0.1
        else:
            self.velo.linear.x = 0.0
        
    def control(self, pointcloud):
        obstacleProcheGauche = False
        obstacleProcheDroite = False
        obstacleLoinGauche = False
        obstacleLoinDroite = False
        for point in pointcloud.points:
            if point.x > 0.1:
                x=point.x/0.18
                y=point.y/0.18
                if -0.5*y*y*y*y > x - 2:
                    if y > 0:
                        obstacleProcheGauche = True
                    else:
                        obstacleProcheDroite = True    
                elif -2*y*y*y*y > x - 5:
                    if y > 0:
                        obstacleLoinGauche = True
                    else:
                        obstacleLoinDroite = True    
        if obstacleProcheGauche and obstacleProcheDroite:
            if self.consigne != 'gauche' and self.consigne != 'droite':
                if random.random() < 0.5:
                    self.consigne = 'gauche'
                else:
                    self.consigne = 'droite'
        elif obstacleProcheDroite: 
            if self.consigne != 'droite':
                self.consigne = 'gauche'
        elif obstacleProcheGauche: 
            if self.consigne != 'gauche':
                self.consigne = 'droite'
        elif obstacleLoinDroite and obstacleLoinGauche: 
            if self.consigne != 'courbeGauche' and self.consigne != 'courbeDroite':
                if random.random() < 0.5:
                    self.consigne = 'courbeGauche'
                else:
                    self.consigne = 'courbeDroite'
        elif obstacleLoinDroite: 
            if self.consigne == 'avance':
                self.consigne = 'courbeGauche'
        elif obstacleLoinGauche: 
            if self.consigne == 'avance':
                self.consigne = 'courbeDroite'
        else:
            self.consigne = 'avance'
        self.consigne_timeout = 0
        # self.get_logger().info( f"\n\nobstacleLoinGauche:{obstacleLoinGauche}" )
        # self.get_logger().info( f"\nobstacleLoinDroite:{obstacleLoinDroite}" )
        # self.get_logger().info( f"\nobstacleProcheGauche:{obstacleProcheGauche}" )
        # self.get_logger().info( f"\nobstacleProcheDroite:{obstacleProcheDroite}" )
        # self.get_logger().info( f"\nConsigne:{self.consigne}" )

    def randomise(self):
        self.rand_target = random.randint(-20, 20)

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