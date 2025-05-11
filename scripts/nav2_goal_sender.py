#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.wait_for_action_server()
        
    def wait_for_action_server(self):
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('Nav2 action server available!')
        
    def send_goal(self, position_x, position_y, orientation_z=0.0, orientation_w=1.0, frame_id='map'):
        """Send a goal to Nav2 with the specified pose."""
        goal_msg = NavigateToPose.Goal()
        
        # Create the pose message
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = position_x
        goal_msg.pose.pose.position.y = position_y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (as quaternion)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = orientation_z
        goal_msg.pose.pose.orientation.w = orientation_w
        
        self.get_logger().info(f'Sending goal: x={position_x}, y={position_y}')
        
        # Send the goal and get the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
            
        self.get_logger().info('Goal accepted!')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        return get_result_future
    
    def follow_waypoints(self, waypoints):
        """Follow a list of waypoints in sequence.
        
        Each waypoint is a tuple of (x, y, z, w) where z and w are optional 
        orientation quaternion components.
        """
        for i, waypoint in enumerate(waypoints):
            # Unpack the waypoint
            if len(waypoint) == 2:
                x, y = waypoint
                z, w = 0.0, 1.0
            else:
                x, y, z, w = waypoint
                
            self.get_logger().info(f'Navigating to waypoint {i+1}/{len(waypoints)}: ({x}, {y})')
            
            # Send the goal and wait for completion
            result_future = self.send_goal(x, y, z, w)
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result()
            if result.status != 4:  # 4 is SUCCESS in action msgs
                self.get_logger().warn(f'Goal failed with status: {result.status}')
            else:
                self.get_logger().info(f'Reached waypoint {i+1}!')
            
            # Optional: add delay between waypoints
            time.sleep(1.0)


def main():
    rclpy.init()
    
    navigator = Nav2GoalSender()
    
    # Example 1: Send a single goal
    # result_future = navigator.send_goal(1.0, 2.0)
    # rclpy.spin_until_future_complete(navigator, result_future)
    
    # Example 2: Follow a sequence of waypoints
    waypoints = [
        (1.0, 0.0),                  # (x, y) only - 1m forward
        (2.0, 1.0, 0.7071, 0.7071),  # (x, y, qz, qw) - diagonal with 90 degree rotation
        (0.0, 1.0),                  # Back to y-axis, 1m to the left
        (0.0, 0.0, 0.0, 1.0)         # Return to start, facing forward
    ]
    
    navigator.follow_waypoints(waypoints)
    
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 