import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import csv
import pandas as pd  # Data handling


class MyRobot(Node):
    def __init__(self):
        super().__init__('myrobot_node')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.sub_laser = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_policy)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.odom_pos = (-1.9, -0.5, 0.0)  # Initial pose (x, y, theta)
        self.laser_ranges = None
        waypoints = [(-1.5, -0,5), (-1.5, 1.6), (-0.7, 1.5), (-0.5, -1.5),
                      (0.5, -1.5), (0.5, 1.5), (1.5, 1.5), (1.5, -1.5)]  # Updated waypoints
        self.goals = self.calculate_waypoint_orientations(waypoints)
        self.current_goal_index = 0
        self.points = []


        self.obstacle_detected = False
        self.obstacle_avoidance_turn_angle = math.radians(10)  # degree obstacle detection

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.8
        self.linear_acceleration = 0.1
        self.angular_acceleration = 0.1

        self.Kp_linear = 0.2
        self.Kp_angular = 0.5

        #threshold
        self.clear_path_threshold = 0.1
        self.orientation_threshold = math.radians(5)

    def calculate_waypoint_orientations(self, waypoints):
        goals = []
        for i in range(len(waypoints)):
            current_wp = waypoints[i]
            next_wp = waypoints[(i + 1) % len(waypoints)]
            delta_x = next_wp[0] - current_wp[0]
            delta_y = next_wp[1] - current_wp[1]
            theta = math.atan2(delta_y, delta_x)
            goals.append((current_wp[0], current_wp[1], theta))
        return goals

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        if min(msg.ranges) < 0.25:  #obstacle detection threshold
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pos = (x, y, theta)
        print(f"X_=:{x}, Y_=:{y}")
        self.points.append((x,y))

    def control_loop(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info("All waypoints reached. Stopping robot.")
            self.stop_robot()
            return

        goal_x, goal_y,_ = self.goals[self.current_goal_index]
        distance_to_goal, angle_to_goal = self.calculate_distance_and_angle_to_goal(goal_x, goal_y)

        if self.obstacle_detected:
            self.handle_obstacle()
            if self.path_is_clear():
                self.obstacle_detected = False 
            return  # Skip moving towards the goal until the obstacle is cleared
        
        # Check if orientation towards the goal is needed
        if abs(angle_to_goal) > self.orientation_threshold:  # Define self.orientation_threshold in __init__
            # Correct orientation with zero linear speed
            self.correct_orientation(angle_to_goal)
        else:
            # Move towards the goal once properly oriented
            self.move_towards_goal(distance_to_goal, angle_to_goal)

        print(f"Distance:{distance_to_goal}, Goal Index:{self.current_goal_index}") 
        self.move_towards_goal(distance_to_goal, angle_to_goal)

        if distance_to_goal < 0.1 :
            self.current_goal_index += 1
            self.get_logger().info(f"Waypoint {self.current_goal_index} reached. Moving to next waypoint.")

    def correct_orientation(self, angle_to_goal):
        angular_speed = self.Kp_angular * angle_to_goal
        self.publish_velocity(0, angular_speed)  # Zero linear speed

    def move_towards_goal(self, distance, angle):
        linear_speed = self.Kp_linear * (distance/2)
        angular_speed = self.Kp_angular * angle

        self.linear_velocity = min(linear_speed, self.max_linear_speed)
        self.angular_velocity = min(angular_speed, self.max_angular_speed)

        self.publish_velocity(self.linear_velocity, self.angular_velocity)

    def handle_obstacle(self):
        front_ranges = self.laser_ranges[len(self.laser_ranges)//3:len(self.laser_ranges)*2//3]
        left_ranges = self.laser_ranges[:len(self.laser_ranges)//3]
        right_ranges = self.laser_ranges[len(self.laser_ranges)*2//3:]

        # Calculate average distance to obstacles in each direction
        avg_dist_front = sum(front_ranges) / len(front_ranges)
        avg_dist_left = sum(left_ranges) / len(left_ranges)
        avg_dist_right = sum(right_ranges) / len(right_ranges)

        turn_direction = 1  # 1 for right, -1 for left
        if avg_dist_left > avg_dist_right:
            turn_direction = -1  # More space on the left

        #Turning
        turn_angle = self.obstacle_avoidance_turn_angle * turn_direction
        self.publish_velocity(0, turn_angle)
        if self.path_is_clear():
            self.obstacle_detected = False  # Off this flag when the path is clear

    def path_is_clear(self):
        return sum(self.laser_ranges[len(self.laser_ranges)//3:len(self.laser_ranges)*2//3]) / len(self.laser_ranges[len(self.laser_ranges)//3:len(self.laser_ranges)*2//3]) > self.clear_path_threshold

    def calculate_distance_and_angle_to_goal(self, goal_x, goal_y):
        dx = goal_x - self.odom_pos[0]
        dy = goal_y - self.odom_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx) - self.odom_pos[2]
        return distance, angle
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_velocity(self, linear, angular):
        cmd_vel = Twist()
        cmd_vel.linear.x = float(linear)
        cmd_vel.angular.z = float(angular)
        self.pub_cmd_vel.publish(cmd_vel)

    def stop_robot(self):
        #stop the robot
        self.publish_velocity(0, 0)
        self.square_drawn = True
        self.Moving()

    def Moving(self):
        if self.square_drawn:
            # open a data file in the M-Drive
            myfile = open("myodom_mapping.csv", "w+")
            # TO DO:
            writer =csv.writer(myfile)
            writer.writerows(map(list, self.points))
                
            myfile.close()

            self.save_to_excel()
            print('Data saved')

    def save_to_excel(self):
        df = pd.read_csv('myodom.csv', names= ['X','Y'])
        print(df)
        


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
