#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.get_logger().info('Pick and Place node has been started.')

        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPositionIK.Request()

        self.action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self, goal_pose):
        self.req.ik_request.group_name = "ur_manipulator"
        self.req.ik_request.pose_stamped.header.frame_id = "base_link"
        self.req.ik_request.pose_stamped.pose = goal_pose
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            if self.future.result().error_code.val == 1:
                self.get_logger().info('Goal is reachable')
                trajectory = self.future.result().solution.joint_state
                goal_msg = FollowJointTrajectory.Goal()
                goal_msg.trajectory.joint_names = trajectory.name
                point = JointTrajectoryPoint()
                point.positions = trajectory.position
                point.time_from_start.sec = 4
                goal_msg.trajectory.points.append(point)
                self.action_client.wait_for_server()
                self.send_goal_future = self.action_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, self.send_goal_future)
                goal_handle = self.send_goal_future.result()
                if not goal_handle.accepted:
                    self.get_logger().info('Goal rejected :(')
                    return False

                self.get_logger().info('Goal accepted :)')

                self._get_result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, self._get_result_future)
                result = self._get_result_future.result().result
                self.get_logger().info('Result: {0}'.format(result))
                return True
            else:
                self.get_logger().info('Goal is not reachable')
                return False
        else:
            self.get_logger().error('Exception while calling service: %r' % (self.future.exception(),))
            return False

def main(args=None):
    rclpy.init(args=args)
    pick_and_place = PickAndPlace()

    # Wait for Gazebo to open
    time.sleep(20)

    # Define the poses
    home_pose = Pose()
    home_pose.position.x = 0.3
    home_pose.position.y = 0.0
    home_pose.position.z = 0.5
    home_pose.orientation.x = 0.0
    home_pose.orientation.y = 1.0
    home_pose.orientation.z = 0.0
    home_pose.orientation.w = 0.0

    cube_pose = Pose()
    cube_pose.position.x = 0.5
    cube_pose.position.y = 0.0
    cube_pose.position.z = 0.1
    cube_pose.orientation.x = 0.0
    cube_pose.orientation.y = 1.0
    cube_pose.orientation.z = 0.0
    cube_pose.orientation.w = 0.0

    place_pose = Pose()
    place_pose.position.x = -0.5
    place_pose.position.y = 0.0
    place_pose.position.z = 0.1
    place_pose.orientation.x = 0.0
    place_pose.orientation.y = 1.0
    place_pose.orientation.z = 0.0
    place_pose.orientation.w = 0.0

    # Move to home
    pick_and_place.send_goal(home_pose)

    # Move to cube
    pick_and_place.send_goal(cube_pose)

    pick_and_place.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()