import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.timer import Timer
from mercer_interfaces.msg import SimplifiedMarkerArray, SimplifiedMarker

import sys

class FinalNode(Node):
    def __init__(self):
        super().__init__('final_node')

        self.translation_bias = 1.0
        self.extension_bias = 1.0
        self.markers_response_dict = {}
        self.joint_state = JointState()
        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)

        self.visible_markers = set()
        self.marker_listener = self.create_subscription(SimplifiedMarkerArray, '/aruco/visible_markers', self.marker_callback, 1)

        self.action_timer = self.create_timer(2, self.action_timer_callback)

        # This client will send goals to the follor_joint_trajectory action running in stretch_driver node
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, 'stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Action server not available after waiting for 60 seconds. Exiting...')
            sys.exit()

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state
    
    def marker_callback(self, msg):
        self.visible_markers = set([marker.name for marker in msg.markers])
    
    def action_timer_callback(self):
        if self.visible_markers:
            self.get_logger().info('Visible markers: {}'.format(self.visible_markers))

            point = JointTrajectoryPoint()
            goal_msg = FollowJointTrajectory.Goal()
            correct_marker = False
            # Spin in a circle
            if 'spin' in self.visible_markers:
                goal_msg.trajectory.joint_names.append('rotate_mobile_base')
                point.positions.append(6.3)
                point.velocities.append(1.0)
                point.accelerations.append(3.14)
                correct_marker = True

            # Move Forward or Backwards
            if 'translate' in self.visible_markers:
                goal_msg.trajectory.joint_names.append('translate_mobile_base')
                point.positions.append(1.0 * self.translation_bias)
                point.velocities.append(0.5)
                point.accelerations.append(1.0)
                self.translation_bias = -self.translation_bias
                correct_marker = True

            # Move Lift Upwards
            if 'move_up' in self.visible_markers:
                joint_index = self.joint_state.name.index('joint_lift')
                joint_value = self.joint_state.position[joint_index]
                goal_msg.trajectory.joint_names.append('joint_lift')
                point.positions.append(joint_value + 0.2)
                point.velocities.append(0.2)
                point.accelerations.append(1.0)
                correct_marker = True

            # Move Lift Down
            elif 'move_down' in self.visible_markers:
                joint_index = self.joint_state.name.index('joint_lift')
                joint_value = self.joint_state.position[joint_index]
                goal_msg.trajectory.joint_names.append('joint_lift')
                point.positions.append(joint_value - 0.2)
                point.velocities.append(0.2)
                point.accelerations.append(1.0)
                correct_marker = True

            # Extend Arm In/Out
            if 'extend' in self.visible_markers:
                joint_index = self.joint_state.name.index('wrist_extension')
                joint_value = self.joint_state.position[joint_index]
                goal_msg.trajectory.joint_names.append('wrist_extension')
                point.positions.append(joint_value + 0.2 * self.extension_bias)
                point.velocities.append(0.2)
                point.accelerations.append(1.0)
                self.extension_bias = -self.extension_bias
                correct_marker = True

            if correct_marker:
                goal_msg.trajectory.points.append(point)
                self.trajectory_client.send_goal_async(goal_msg)

        else:
            self.get_logger().info('No markers visible')


def main():
    try:
        rclpy.init()
        node = FinalNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')
    
    if node is not None:
        node.destroy_node()

    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()