import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
import random
import statistics

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

        self.phaseDetectCola = False
        self.phaseDetectCherry = False

        self.cola_detected = []
        self.cherry_detected = []
        self.create_subscription( Point, 'cola_detect', self.cola_react, 10)
        self.create_subscription( Point, 'cherry_detect', self.cherry_react, 10)
        self.cola_detectDefinitPublisher = self.create_publisher(Point, 'cola_detect_definite', 10)
        self.cherry_detectDefinitPublisher = self.create_publisher(Point, 'cherry_detect_definite', 10)
        self.cola_analyse_timer = self.create_timer(0.5, self.cola_analyse)
        self.cherry_analyse_timer = self.create_timer(0.5, self.cherry_analyse)

    def avance(self):
        if self.velo.linear.x <= 0.4:
            self.velo.linear.x += 0.03
        if self.phaseDetectCola or self.phaseDetectCherry:
            self.velo.linear.x = 0.1

    def tourneGauche(self):
        if self.velo.angular.z <= 0.8:
            self.velo.angular.z += 0.2
        if self.phaseDetectCola or self.phaseDetectCherry:
            self.velo.angular.z = 0.2

    def tourneDroite(self):
        if self.velo.angular.z >= -0.8:
            self.velo.angular.z -= 0.2
        if self.phaseDetectCola or self.phaseDetectCherry:
            self.velo.angular.z = -0.2
    
    def droitDevant(self):
        if self.velo.angular.z >= 0.05:
            self.velo.angular.z -= 0.1
        elif self.velo.angular.z < -0.05:
            self.velo.angular.z += 0.1
        elif self.velo.angular.z != 0.0:
            self.velo.angular.z = 0.0
        if self.phaseDetectCola or self.phaseDetectCherry:
            self.velo.angular.z = 0.0

    def activate(self):
        if not self.phaseDetectCola and not self.phaseDetectCherry:
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
        self.consigne_timeout = 0
        if self.phaseDetectCola or self.phaseDetectCherry:
            return
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
        # self.get_logger().info( f"\n\nobstacleLoinGauche:{obstacleLoinGauche}" )
        # self.get_logger().info( f"\nobstacleLoinDroite:{obstacleLoinDroite}" )
        # self.get_logger().info( f"\nobstacleProcheGauche:{obstacleProcheGauche}" )
        # self.get_logger().info( f"\nobstacleProcheDroite:{obstacleProcheDroite}" )
        # self.get_logger().info( f"\nConsigne:{self.consigne}" )

    def randomise(self):
        self.rand_target = random.randint(-20, 20)

    def cola_react(self, ptMsg):
        self.cola_detected.append(ptMsg)

    def cherry_react(self, ptMsg):
        self.cherry_detected.append(ptMsg)

    def cola_analyse(self):
        if len(self.cola_detected) >= 10 :
            list_x = []
            list_y = []
            list_z = []
            for p in self.cola_detected:
                list_x.append(p.x)
                list_y.append(p.y)
                list_z.append(p.z)
            etypeX = statistics.stdev(list_x)
            etypeY = statistics.stdev(list_y)
            mx = statistics.mean(list_x)
            my = statistics.mean(list_y)
            mz = statistics.mean(list_z)
            if etypeX < 200 and etypeY < 200 :
                if not self.phaseDetectCola :
                    self.phaseDetectCola = True
                    self.compteur_stop_cola = 0
                if mz > 0.6:
                    if mx > 498:
                        self.consigne = 'droite'
                    elif mx < 350:
                        self.consigne = 'gauche'
                    else:
                        self.consigne = 'avance'
                else:
                    self.consigne = 'stop'
                    self.compteur_stop_cola += 1
                    p = Point()
                    p.x = mx
                    p.y = my
                    p.z = mz
                    self.cola_detectDefinitPublisher.publish( p )
                if self.compteur_stop_cola >= 10:
                    self.consigne = 'droite'
                    

                self.get_logger().info( f"\npoint:{p}" )
                self.get_logger().info( f"\nConsigne:{self.consigne}" )
                self.rand_target = 0
            else:
                self.phaseDetectCola = False
        else:
            self.phaseDetectCola=False
        self.cola_detected = []


    def cherry_analyse(self):
        if len(self.cherry_detected) >= 10 :
            list_x = []
            list_y = []
            list_z = []
            for p in self.cherry_detected:
                list_x.append(p.x)
                list_y.append(p.y)
                list_z.append(p.z)
            etypeX = statistics.stdev(list_x)
            etypeY = statistics.stdev(list_y)
            mx = statistics.mean(list_x)
            my = statistics.mean(list_y)
            mz = statistics.mean(list_z)
            if etypeX < 200 and etypeY < 200 :
                if not self.phaseDetectCherry :
                    self.phaseDetectCherry = True
                    self.compteur_stop_cherry = 0
                if mz > 0.6:
                    if mx > 498:
                        self.consigne = 'droite'
                    elif mx < 350:
                        self.consigne = 'gauche'
                    else:
                        self.consigne = 'avance'
                else:
                    self.consigne = 'stop'
                    self.compteur_stop_cherry += 1
                    p = Point()
                    p.x = mx
                    p.y = my
                    p.z = mz
                    self.cherry_detectDefinitPublisher.publish( p )
                if self.compteur_stop_cherry >= 10:
                    self.consigne = 'droite'
                    

                self.get_logger().info( f"\npoint:{p}" )
                self.get_logger().info( f"\nConsigne:{self.consigne}" )
                self.rand_target = 0
            else:
                self.phaseDetectCherry = False
        else:
            self.phaseDetectCherry=False
        self.cherry_detected = []

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
