import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
import numpy as np

class TurtlebotControl(Node):

    def __init__(self):
        super().__init__("turtlebot3_control")

        self.pose             = None            # Guarda a posição e velocidade do robô
        self.graus            = None            # Possui a orientação do robô
        self.msg              = Twist()         # movimenta o robô
        self.yaw              = None

        self.x = 2
        self.y = -1
        self.angulo = math.pi/6
        self.loop = True
        self.terminei_de_virar = False
        self.log = True
        self.aux = True
        self.erro_angular_anterior = 0
        self.rotation_p_gain = 5.0
        self.rotation_d_gain = 1.0

        self.laser_const = 1

        self.forward_gain  = 0.7

        # Publishing frame rate
        self.frames_sec = 30

        # Creating motion control publisher
        self.motion_publisher_ = self.create_publisher(Twist, "cmd_vel", 10) # Publishes in the cmd_vel topic from turtlebot3

        self.pose_subscription = self.create_subscription(Odometry, "odom", self.Turtlebot3_pose, 10)
        
        self.timer_ = self.create_timer(1/self.frames_sec, self.control_loop) # Run control_loop 30 times per sec
        
        #self.get_logger().info("Turtlebot3 Node has been started!!")
    
    def Turtlebot3_pose(self, msg):
        '''Retorna a Pose do Turtlebot3'''
        self.pose = msg
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.orientation = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        self.aux = False
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orientation)

    def go_to(self, x_target, y_target):
        if self.aux:
            return
        
        aux = 0
        x_position = self.position.x
        y_position = self.position.y

        x = x_target - x_position
        y = y_target - y_position

        if y >=0 and x>=0:
            obj = math.atan(y/x)
        elif y>=0 and x<0:
            obj = np.pi + math.atan(y/x)
        elif y<0 and x>=0:
            obj = math.atan(y/x)
        else:
            obj = -np.pi + math.atan(y/x)

        if self.yaw is None:
            return

        if obj>=0:
            f_obj = True
        else:
            f_obj = False

        if self.yaw>=0:
            f_yaw = True
        else:
            f_yaw = False

        ########################

        if f_obj==f_yaw:
            erro = obj - self.yaw 
        else:
            if f_obj:
                aux = -np.pi+obj
            else:
                aux = np.pi+obj
            
            if f_yaw:
                if self.yaw>=aux:
                    erro = self.yaw - obj 
                else:
                    erro = obj - self.yaw 
            else:
                if self.yaw>=aux:
                    erro = obj - self.yaw 
                else:
                    erro = self.yaw - obj 
        
        position_error = np.sqrt((x_position-x_target)**2 + (y_position-y_target)**2)
        if abs(erro)>0.05:
            self.msg.angular.z = erro # Positivo = anti-horário
        else:
            self.msg.angular.z = 0.0
        
        if self.msg.angular.z > 5.0:
            self.msg.angular.z = 5.0

        if position_error>0.3:
            self.msg.linear.x = self.forward_gain
            self.motion_publisher_.publish(self.msg)
            return False
        else:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.motion_publisher_.publish(self.msg)
            return True

    def angle_ajust(self, angulo):

        if self.yaw == None:
            return

        diff_angle = self.yaw - angulo
        if self.erro_angular_anterior == 0:
            self.erro_angular_anterior = diff_angle
        
        self.erro_d = self.erro_angular_anterior - diff_angle
        self.erro_angular_anterior = diff_angle
        
        if abs(diff_angle) > 0.05: 
            self.msg.angular.z = -self.rotation_p_gain * diff_angle + self.rotation_d_gain * self.erro_d
            self.msg.linear.x = 0.0

        else:
            self.terminei_de_virar = True
            self.msg.angular.z = 0.0
            self.msg.linear.x = 0.0

        
        self.motion_publisher_.publish(self.msg)
        
    def control_loop(self):
        if self.aux:
            return

        if self.log:
            self.get_logger().info('Posição atual x: ' + str(round(self.position.x, 2)) + ', Posição atual y: ' + str(round(self.position.y, 2)))
            self.get_logger().info('Posição final x: ' + str(round(self.x, 2)) + ', Posição final y: ' + str(round(self.y, 2)))
            self.log = False
        
        if self.loop:
            a = self.go_to(self.x, self.y)
            if a:
                self.loop = False
            else: return


        self.angle_ajust(self.angulo)
        if self.terminei_de_virar:
            self.x = self.x +1
            self.y = self.y +1
            self.angulo = self.angulo + math.pi/3
            self.terminei_de_virar = False
            self.loop = True
            self.log = True

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
