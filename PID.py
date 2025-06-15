#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import pandas as pd  # data
import matplotlib.pyplot as plt


class MyRobot(Node):

    def __init__(self):
        super().__init__('myrobot_node')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        ## This QOS_POLICY needs to go before the laser subscription in your code. ##
        self.sub1 = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback, qos_profile=qos_policy)
        ## The QOS_POLICY needs to be added to the call back. ##
        self.sub2 = self.create_subscription(Odometry, 'odom',
                                             self.odom_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # timer_period = 0.1 # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.my_x = 0
        self.my_y = 0
        self.my_th = 0
        self.my_ls = LaserScan()
        self.goals = [
        (1000, 0, 0), #0
        (1000, 0,math.pi / 4),#1
        (1000, 0,math.pi / 2),#2
        (1000, 250,math.pi / 2),#3
        (1000, 500,math.pi / 2),#4
        (1000, 750,math.pi / 2),#5
        (1000, 1000, math.pi / 2),#6
        (1000, 1000, math.pi),#7
        (0, 1000, math.pi),#8
        (0, 1000, -math.pi/2),#9
        (0, 750, -math.pi/2),#10
        (0, 500, -math.pi/2),#11
        (0, 250, -math.pi/2),#12
        (0, 0, -math.pi/2),#13
        (0, 0, -math.pi/2)]#14

        self.goal_index = 0
        self.goal_tolerance = 60
        self.max_linear_speed = 0.6
        self.max_angular_speed = 0.5 # rad/s
        self.Kp = 0.1
        self.Ki = 0.01
        self.Kd = 0.05
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.prev_error_th = 0
        self.timer = self.create_timer(0.1, self.control_loop)  # control loop at 10 Hz
        self.points = []

        self.square_drawn = False #Flag

    def scan_callback(self, msg):
        self.my_ls.ranges = msg.ranges
        # print ('r = {0:5.2f}, f = {1:5.2f}, l = {2:5.2f}'.format(self.my_ls.ranges[90], self.my_ls.ranges[0], self.my_ls.ranges[270]))

    def odom_callback(self, msg):
        if not self.square_drawn:
            self.my_x = msg.pose.pose.position.x * 1000  # m to mm
            self.my_y = msg.pose.pose.position.y * 1000
            # convert quaternian to Euler angles
            q0 = msg.pose.pose.orientation.w
            q1 = msg.pose.pose.orientation.x  # aw error
            q2 = msg.pose.pose.orientation.y
            q3 = msg.pose.pose.orientation.z
            self.my_th = math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2 * q2+ q3 * q3)))  # rad
            print ('x = {0:5.2f}, y = {1:5.2f}, th = {2:5.2f}'.format(self.my_x, self.my_y, self.my_th))
            self.points.append((self.my_x, self.my_y))

    def control_loop(self):
        if not self.square_drawn:
            goal_x, goal_y, goal_yaw = self.goals[self.goal_index]  # current goal point

            #PID： proportional
            error_x = abs(goal_x - self.my_x)
            error_y = abs(goal_y - self.my_y)
            error_yaw = goal_yaw - self.my_th

            print(f"Goal:({goal_x},{goal_y}, Current:({self.my_x},{self.my_y}))")

            # PID: integral
            integral_error_x = self.prev_error_x + error_x
            integral_error_y = self.prev_error_y + error_y
            integral_error_th = self.prev_error_th + error_yaw

            # PID: derivative
            derivative_error_x = error_x - self.prev_error_x
            derivative_error_y = error_y - self.prev_error_y
            derivative_error_th = error_yaw - self.prev_error_th

            linear_speed = (self.Kp * error_x + self.Ki * integral_error_x + self.Kd * derivative_error_x
                            +self.Kp * error_y + self.Ki * integral_error_y + self.Kd * derivative_error_y)
            angular_speed = self.Kp * error_yaw + self.Ki * integral_error_th + self.Kd * derivative_error_th

            distance = math.sqrt(error_x ** 2 + error_y ** 2)
            angle = math.atan2(error_y, error_x)
            error_angle = angle - self.my_th
            cmd_msg = Twist()

            if distance > self.goal_tolerance:
                cmd_msg.linear.x = min(linear_speed, self.max_linear_speed)
                cmd_msg.angular.z = min(angular_speed, self.max_angular_speed)
            else:
                cmd_msg.linear.x = 0.0  # stop linear motion
                if abs(error_yaw) > self.goal_tolerance / 1000:
                    cmd_msg.angular.z = min(angular_speed, self.max_angular_speed)
                else:
                    cmd_msg.linear.x = 0.0
                    self.goal_index = (self.goal_index + 1) % len(self.goals)  #IMPORTANT: next goal point
                    
                    if self.goal_index == 0:
                        self.square_drawn = True
                        self.get_logger().info("Shutting down the node...")
            self.pub.publish(cmd_msg)

            # update
            self.prev_error_x = error_x
            self.prev_error_y = error_y
            self.prev_error_th = error_yaw

            print(f"Distance:{distance}, Yaw_error: {error_yaw}, Goal Index:{self.goal_index}") 
        else:
            self.Moving()
            self.timer.cancel()


    def Moving(self):
        if self.square_drawn:
            # open a data file in the M-Drive, you need it to change to yours
            myfile = open("myodom.csv", "w+")
            # TO DO:
            writer =csv.writer(myfile)
            writer.writerows(map(list, self.points))

            myfile.close()
            print('path completed!')
            self.save_to_excel()
            print('Data saved')

    def save_to_excel(self):
        df = pd.read_csv('myodom.csv', names= ['X','Y'])
        
        print(df)

        x = df['X']
        y = df['Y']
        # Plot the trajectory
        plt.figure(figsize=(8, 6))
        plt.scatter(x, y, label='Trial 1', marker='o', color='blue')
        plt.title('Robot Trajectory - Trial 2')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.show()

        


def main(args=None):
    rclpy.init(args=args)
    myrobot_node = MyRobot()
    rclpy.spin(myrobot_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # myrobot_node.Moving()
    myrobot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
