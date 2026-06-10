#!/usr/bin/env python3
import unittest

import actionlib
import rospy
import tf
from geometry_msgs.msg import WrenchStamped
from tool_gravity_compensation.msg import CollectLoadProfileAction, CollectLoadProfileGoal
from tool_gravity_compensation.srv import (
    ComputePayload,
    ComputePayloadRequest,
    SetSamplingConfig,
    StepControl,
    StepControlRequest,
)


GRAVITY = 9.80665


def transpose_mat_vec(matrix, vector):
    return [
        matrix[0] * vector[0] + matrix[3] * vector[1] + matrix[6] * vector[2],
        matrix[1] * vector[0] + matrix[4] * vector[1] + matrix[7] * vector[2],
        matrix[2] * vector[0] + matrix[5] * vector[1] + matrix[8] * vector[2],
    ]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def quaternion_from_rotation(matrix):
    return tf.transformations.quaternion_from_matrix([
        [matrix[0], matrix[1], matrix[2], 0.0],
        [matrix[3], matrix[4], matrix[5], 0.0],
        [matrix[6], matrix[7], matrix[8], 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


class CollectLoadProfilePositiveTest(unittest.TestCase):

    ROTATIONS = [
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0],
    ]

    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_collect_load_profile_positive", anonymous=True)
        cls.wrench_pub = rospy.Publisher("/sri_ft_sensor/wrench", WrenchStamped, queue_size=20)
        cls.tf_broadcaster = tf.TransformBroadcaster()
        rospy.wait_for_service("/step_control", timeout=5.0)
        rospy.wait_for_service("/compute_payload", timeout=5.0)
        rospy.wait_for_service("/set_sampling_config", timeout=5.0)
        cls.step_control = rospy.ServiceProxy("/step_control", StepControl)
        cls.compute_payload = rospy.ServiceProxy("/compute_payload", ComputePayload)
        cls.set_sampling_config = rospy.ServiceProxy("/set_sampling_config", SetSamplingConfig)
        cls.collect_client = actionlib.SimpleActionClient("/collect_load_profile", CollectLoadProfileAction)
        if not cls.collect_client.wait_for_server(rospy.Duration(5.0)):
            raise RuntimeError("collect_load_profile action server not available")
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while cls.wrench_pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        if cls.wrench_pub.get_num_connections() == 0:
            raise RuntimeError("collect_load_profile_server did not subscribe to wrench topic")

    def publish_static_window(self, rotation, mass, com):
        q = quaternion_from_rotation(rotation)
        gravity_force_base = [0.0, 0.0, -mass * GRAVITY]
        force_sensor = transpose_mat_vec(rotation, gravity_force_base)
        torque_sensor = cross(com, force_sensor)

        for _ in range(15):
            now = rospy.Time.now()
            self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), q, now, "sri_ft_sensor", "base_link")
            msg = WrenchStamped()
            msg.header.stamp = now
            msg.header.frame_id = "sri_ft_sensor"
            msg.wrench.force.x = force_sensor[0]
            msg.wrench.force.y = force_sensor[1]
            msg.wrench.force.z = force_sensor[2]
            msg.wrench.torque.x = torque_sensor[0]
            msg.wrench.torque.y = torque_sensor[1]
            msg.wrench.torque.z = torque_sensor[2]
            self.wrench_pub.publish(msg)
            rospy.sleep(0.01)
        rospy.sleep(0.05)

    def record_sample(self, profile_type, step_index, rotation, mass, com):
        self.publish_static_window(rotation, mass, com)
        request = StepControlRequest()
        request.profile_type = profile_type
        request.step_index = step_index
        request.execute_motion = False
        request.sample_wrench = True
        request.manual_confirmed = True
        request.target_pose_name = "test"
        response = self.step_control(request)
        self.assertTrue(response.accepted, response.message)
        self.assertTrue(response.sample_recorded, response.message)
        return response

    def clear_profiles(self):
        response = self.set_sampling_config(
            apply=True,
            clear_profiles=True,
            filter_window_size=10,
            min_stable_samples=10,
            max_stddev_force=0.001,
            max_stddev_torque=0.001,
            sample_timeout=1.0)
        self.assertTrue(response.success, response.message)

    def test_computes_payload_after_tool_and_total_profiles_are_sampled(self):
        self.clear_profiles()
        tool_mass = 1.5
        tool_com = [-0.02, 0.01, 0.12]
        payload_mass = 2.5
        payload_com = [0.10, -0.04, 0.20]
        total_mass = tool_mass + payload_mass
        total_com = [
            (tool_mass * tool_com[i] + payload_mass * payload_com[i]) / total_mass
            for i in range(3)
        ]

        for index, rotation in enumerate(self.ROTATIONS):
            self.record_sample(StepControlRequest.TOOL_ONLY, index, rotation, tool_mass, tool_com)
        for index, rotation in enumerate(self.ROTATIONS):
            self.record_sample(
                StepControlRequest.TOOL_PLUS_PAYLOAD,
                index + len(self.ROTATIONS),
                rotation,
                total_mass,
                total_com,
            )

        request = ComputePayloadRequest()
        request.compute = True
        request.min_samples = len(self.ROTATIONS)
        response = self.compute_payload(request)

        self.assertTrue(response.success, response.message)
        self.assertTrue(response.result.success, response.result.message)
        self.assertAlmostEqual(response.result.mass_kg, payload_mass, places=6)
        self.assertAlmostEqual(response.result.weight_n, payload_mass * GRAVITY, places=6)
        self.assertAlmostEqual(response.result.tool_mass, tool_mass, places=6)
        self.assertAlmostEqual(response.result.com_sensor.x, payload_com[0], places=6)
        self.assertAlmostEqual(response.result.com_sensor.y, payload_com[1], places=6)
        self.assertAlmostEqual(response.result.com_sensor.z, payload_com[2], places=6)

        self.clear_profiles()

        after_clear = self.compute_payload(request)
        self.assertFalse(after_clear.success)
        self.assertFalse(after_clear.result.success)
        self.assertTrue(after_clear.message)
        self.assertEqual(after_clear.message, after_clear.result.message)

    def test_manual_action_records_requested_samples_without_motion(self):
        self.publish_static_window(self.ROTATIONS[0], 1.5, [-0.02, 0.01, 0.12])

        goal = CollectLoadProfileGoal()
        goal.profile_type = CollectLoadProfileGoal.TOOL_ONLY
        goal.requested_samples = 2
        goal.execute_motion = False
        goal.manual_confirmed = True
        self.collect_client.send_goal(goal)
        self.assertTrue(self.collect_client.wait_for_result(rospy.Duration(5.0)))

        result = self.collect_client.get_result()
        self.assertTrue(result.success, result.message)
        self.assertTrue(result.result.success, result.result.message)

    def test_step_control_reports_counts_split_by_profile_type(self):
        self.clear_profiles()

        first_tool = self.record_sample(
            StepControlRequest.TOOL_ONLY, 0, self.ROTATIONS[0], 1.5, [-0.02, 0.01, 0.12])
        self.assertEqual(first_tool.samples_collected, 1)
        self.assertEqual(first_tool.tool_samples_collected, 1)
        self.assertEqual(first_tool.total_samples_collected, 0)

        second_tool = self.record_sample(
            StepControlRequest.TOOL_ONLY, 1, self.ROTATIONS[1], 1.5, [-0.02, 0.01, 0.12])
        self.assertEqual(second_tool.samples_collected, 2)
        self.assertEqual(second_tool.tool_samples_collected, 2)
        self.assertEqual(second_tool.total_samples_collected, 0)

        first_total = self.record_sample(
            StepControlRequest.TOOL_PLUS_PAYLOAD, 2, self.ROTATIONS[2], 4.0, [0.055, -0.02125, 0.17])
        self.assertEqual(first_total.samples_collected, 1)
        self.assertEqual(first_total.tool_samples_collected, 2)
        self.assertEqual(first_total.total_samples_collected, 1)

        self.clear_profiles()


if __name__ == "__main__":
    import rostest
    rostest.rosrun("tool_gravity_compensation", "test_collect_load_profile_positive",
                   CollectLoadProfilePositiveTest)
