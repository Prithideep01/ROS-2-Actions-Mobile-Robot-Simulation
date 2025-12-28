#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from mobile_robot_interfaces.action import MoveRobot  
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class RobotActionServer(Node):
    def __init__(self):
        super().__init__("robot_action_server") # node name
        self.goal_handle: ServerGoalHandle = None
        self.goal_lock = threading.Lock()
        self.current_position = 50
        self.action_server = ActionServer(self, MoveRobot, "move_robot",
                                          execute_callback=self.execute_callback,
                                          goal_callback=self.goal_callback,
                                          cancel_callback=self.cancel_callback,
                                          callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f"Action server started. Robot Initial Position: {self.current_position}m")
    
    def goal_callback(self, goal_request: MoveRobot.Goal):

        self.get_logger().info("Recieved a Goal request for the robot")

        # validate the goal request
        if goal_request.goal_position <= 0 and goal_request.desired_velocity<=0:
            self.get_logger().info("Rejecting the Goal request......")
            return GoalResponse.REJECT
        
        ### Goal Policy: Preempt the current active goal when the new goal is received
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info("Aborting the current goal")
                self.goal_handle.abort()

        self.get_logger().info("Accepting the Goal request......")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request from the client")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        goal_position = goal_handle.request.goal_position
        desired_velocity = goal_handle.request.desired_velocity

        with self.goal_lock:
            self.goal_handle = goal_handle

        # Execute the action
        self.get_logger().info("Executing the goal")
        feedback = MoveRobot.Feedback()
        result = MoveRobot.Result()

        # Determine direction (+1 for forward, -1 for backward)
        if goal_position > self.current_position:
            direction = 1
        elif goal_position < self.current_position:
            direction = -1
        else:
            direction = 0 # Already at goal

        # Motion Loop
        while self.current_position != goal_position:

            ### Goal Policy: Preempt the current active goal when the new goal is received
            if not goal_handle.is_active:
                self.get_logger().info("Preempting the current goal........")
                result.final_position = self.current_position
                result.message = f"Preempted by a new goal! Reached {self.current_position}m"
                return result
            
            # Check for Cancel Request
            if goal_handle.is_cancel_requested: 
                self.get_logger().info("Cancelling the Goal")
                goal_handle.canceled()  # set the final state
                result.final_position = self.current_position
                result.message = f"Execution Cancelled ! Reached {self.current_position}m"
                return result
            
            distance_diff = abs(goal_position - self.current_position)

            if desired_velocity > distance_diff:
                step = distance_diff
            else:
                step = desired_velocity

            # Update position based on the calculated step and direction
            self.current_position += (step * direction)
            
            # Publish Feedback
            feedback.current_position = self.current_position
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'Current Position: {self.current_position}m')

            # Sleep for 1 second to simulate m/s movement
            time.sleep(1.0)

        # Mark the goal as successful
        goal_handle.succeed()
        
        # Return Result
        result.final_position = self.current_position
        result.message = f"Success! Reached {self.current_position}m"
        
        return result

def main(args=None):
    rclpy.init(args=args) 
    node = RobotActionServer()
    executor = MultiThreadedExecutor
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()