#!/usr/bin/env python3
import unittest

import actionlib
import rospy

from tool_gravity_compensation.msg import CollectLoadProfileAction, CollectLoadProfileGoal
from tool_gravity_compensation.srv import ComputePayload, SetSamplingConfig, StepControl


class CollectLoadProfileFailClosedTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_collect_load_profile_fail_closed", anonymous=True)
        rospy.wait_for_service("/step_control", timeout=5.0)
        rospy.wait_for_service("/compute_payload", timeout=5.0)
        rospy.wait_for_service("/set_sampling_config", timeout=5.0)
        cls.step_control = rospy.ServiceProxy("/step_control", StepControl)
        cls.compute_payload = rospy.ServiceProxy("/compute_payload", ComputePayload)
        cls.set_sampling_config = rospy.ServiceProxy("/set_sampling_config", SetSamplingConfig)
        cls.client = actionlib.SimpleActionClient("/collect_load_profile", CollectLoadProfileAction)
        if not cls.client.wait_for_server(rospy.Duration(5.0)):
            raise RuntimeError("collect_load_profile action server not available")

    def test_step_control_refuses_missing_data_or_motion_requests(self):
        rejected = self.step_control(
            profile_type=StepControl._request_class.TOOL_ONLY,
            step_index=0,
            execute_motion=False,
            sample_wrench=False,
            target_pose_name="")
        self.assertFalse(rejected.accepted)
        self.assertFalse(rejected.sample_recorded)
        self.assertEqual(rejected.samples_collected, 0)

        motion_rejected = self.step_control(
            profile_type=StepControl._request_class.TOOL_ONLY,
            step_index=1,
            execute_motion=True,
            sample_wrench=True,
            target_pose_name="")
        self.assertFalse(motion_rejected.accepted)
        self.assertFalse(motion_rejected.sample_recorded)
        self.assertEqual(motion_rejected.samples_collected, 0)

        no_wrench = self.step_control(
            profile_type=StepControl._request_class.TOOL_ONLY,
            step_index=2,
            execute_motion=False,
            sample_wrench=True,
            target_pose_name="")
        self.assertFalse(no_wrench.accepted)
        self.assertFalse(no_wrench.sample_recorded)
        self.assertEqual(no_wrench.samples_collected, 0)

    def test_compute_payload_refuses_until_both_profiles_have_samples(self):
        response = self.compute_payload(compute=True, min_samples=1)
        self.assertFalse(response.success)
        self.assertFalse(response.result.success)
        self.assertIn("Need at least 1 samples", response.message)
        self.assertIn("TOOL_ONLY and TOOL_PLUS_PAYLOAD", response.message)

    def test_sampling_config_rejects_unsafe_values_and_accepts_valid_runtime_update(self):
        invalid = self.set_sampling_config(
            apply=True,
            clear_profiles=False,
            filter_window_size=0,
            min_stable_samples=10,
            max_stddev_force=2.0,
            max_stddev_torque=0.2,
            sample_timeout=1.0)
        self.assertFalse(invalid.success)

        valid = self.set_sampling_config(
            apply=True,
            clear_profiles=False,
            filter_window_size=8,
            min_stable_samples=8,
            max_stddev_force=1.5,
            max_stddev_torque=0.15,
            sample_timeout=0.5)
        self.assertTrue(valid.success, valid.message)

    def test_collect_action_aborts_in_checkpoint_mode(self):
        goal = CollectLoadProfileGoal()
        goal.profile_type = CollectLoadProfileGoal.TOOL_ONLY
        goal.requested_samples = 1
        goal.execute_motion = True
        self.client.send_goal(goal)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)))
        self.assertEqual(self.client.get_state(), actionlib.GoalStatus.ABORTED)
        self.assertFalse(self.client.get_result().success)


if __name__ == "__main__":
    import rostest
    rostest.rosrun("tool_gravity_compensation", "test_collect_load_profile_fail_closed",
                   CollectLoadProfileFailClosedTest)
