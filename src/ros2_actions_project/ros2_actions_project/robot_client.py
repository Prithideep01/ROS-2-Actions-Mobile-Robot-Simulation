#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from mobile_robot_interfaces.action import MoveRobot   
from std_msgs.msg import Empty

class RobotActionClient(Node):
    def __init__(self):
        super().__init__("robot_action_client") # node name
        self.goal_handle = None
        self.action_client = ActionClient(self, MoveRobot, "move_robot")
        self.subscriber = self.create_subscription(Empty, "cancel_move", self.cancel_goal, 10)
        self.get_logger().info("Action client node started.............")

    def send_goal(self, goal_position, desired_velocity):
        # Waiting for the server to become active
        self.action_client.wait_for_server()

        # Creating a Goal
        goal = MoveRobot.Goal()
        goal.goal_position = goal_position
        goal.desired_velocity = desired_velocity

        # Send the goal
        self.get_logger().info("Sending the goal position to the mobile robot.....")
        # this will execute the goal_response_callback() when the server replies whether the goal is accepted or rejected
        self.action_client.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)  
    
    def cancel_goal(self, msg):
        self.get_logger().info("Cancel message received....")
        # Do we actually have a goal to cancel?
        if self.goal_handle is None:
            self.get_logger().warn("Cannot cancel: No goal is currently active/accepted yet.")
            return

        self.get_logger().info("Sending cancel request to server...")
        
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_response_callback)
    
    def cancel_response_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Server successfully accepted the cancellation.")
        else:
            self.get_logger().warn("Server rejected the cancellation (Goal might have already finished).")

    # this function checks whether the goal was accepted/rejected
    def goal_response_callback(self, future): 
        self.goal_handle: ClientGoalHandle = future.result()
        if self.goal_handle.accepted:   # if the goal was accepted, we will request the result  
            self.get_logger().info("Goal position was Accepted.....") 
            # this line executes the goal_result_callback when the action is finished (succeeded, aborted, or canceled)
            self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal position got Rejected......")
            self.goal_handle = None

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info(f"Robot's final position is: {result.final_position}")
        self.get_logger().info(f"Robot's message: {result.message}")

        self.goal_handle = None
    
    def goal_feedback_callback(self, feedback_msg):
        current_pos = feedback_msg.feedback.current_position
        self.get_logger().info(f"Robot's current position is: {current_pos}")


def main(args=None):
    rclpy.init(args=args) # initializing the ros2 communications
    node = RobotActionClient()
    node.send_goal(100, 2)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()