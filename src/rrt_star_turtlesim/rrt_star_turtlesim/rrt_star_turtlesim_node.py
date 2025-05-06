#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from turtlesim.msg import Pose as TurtlePose
from rrt_star_turtlesim.rrt_star import RRTStar
import numpy as np
import math

class RRTStarTurtlesimNode(Node):
    def __init__(self):
        super().__init__('rrt_star_turtlesim_node')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(TurtlePose, '/turtle1/pose', self.pose_callback, 10)
        
        # Parameters
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('obstacle_radius', 0.5)
        
        # State variables
        self.current_pose = None
        self.path = None
        self.current_waypoint_idx = 0
        
        # Create timer for path following
        self.create_timer(0.1, self.follow_path)
        
        self.get_logger().info('RRT* Turtlesim Node has been started')
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
        # If we don't have a path yet, plan one
        if self.path is None and self.current_pose is not None:
            self.plan_path()
    
    def plan_path(self):
        # Get parameters
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        obstacle_radius = self.get_parameter('obstacle_radius').value
        
        # Create RRT* planner
        start = (self.current_pose.x, self.current_pose.y)
        goal = (goal_x, goal_y)
        bounds = ((0.0, 11.0), (0.0, 11.0))  # Turtlesim bounds
        
        # Add some random obstacles
        obstacles = []
        for _ in range(5):
            x = np.random.uniform(1.0, 10.0)
            y = np.random.uniform(1.0, 10.0)
            obstacles.append((x, y, obstacle_radius))
        
        planner = RRTStar(
            start=start,
            goal=goal,
            bounds=bounds,
            obstacles=obstacles,
            max_iter=1000,
            step_size=0.5,
            goal_sample_rate=0.1,
            search_radius=1.0
        )
        
        # Plan path
        self.path = planner.plan()
        if self.path:
            self.get_logger().info(f'Path found with {len(self.path)} waypoints')
            self.current_waypoint_idx = 0
        else:
            self.get_logger().warn('No path found')
    
    def follow_path(self):
        if self.path is None or self.current_pose is None:
            return
            
        if self.current_waypoint_idx >= len(self.path):
            return
            
        # Get current waypoint
        waypoint = self.path[self.current_waypoint_idx]
        
        # Calculate distance and angle to waypoint
        dx = waypoint[0] - self.current_pose.x
        dy = waypoint[1] - self.current_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - self.current_pose.theta
        while angle_diff > math.pi:
            angle_diff -= 2*math.pi
        while angle_diff < -math.pi:
            angle_diff += 2*math.pi
            
        # Create velocity command
        cmd = Twist()
        
        # If we're close enough to the waypoint, move to next one
        if distance < 0.1:
            self.current_waypoint_idx += 1
            return
            
        # If we're not facing the right direction, rotate
        if abs(angle_diff) > 0.1:
            cmd.angular.z = 0.5 * angle_diff
        else:
            # Move towards waypoint
            cmd.linear.x = 0.5 * distance
            
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RRTStarTurtlesimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 