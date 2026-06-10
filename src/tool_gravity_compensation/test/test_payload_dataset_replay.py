#!/usr/bin/env python3
import csv
import json
import os
import unittest

import rospy
import tf
from geometry_msgs.msg import WrenchStamped
from tool_gravity_compensation.srv import (
    ComputePayload,
    ComputePayloadRequest,
    SetSamplingConfig,
    StepControl,
    StepControlRequest,
)


COMPARISON_METRICS = [
    ("tool_mass_kg", "tool_mass_kg", "tool_mass_kg", None, 1e-6),
    ("payload_mass_kg", "payload_mass_kg", "mass_kg", None, 1e-6),
    ("payload_weight_n", "payload_weight_n", "weight_n", None, 1e-5),
    ("payload_com_x_m", "payload_com_sensor_m", "com_sensor_m", 0, 1e-6),
    ("payload_com_y_m", "payload_com_sensor_m", "com_sensor_m", 1, 1e-6),
    ("payload_com_z_m", "payload_com_sensor_m", "com_sensor_m", 2, 1e-6),
    ("residual_error", None, "residual_error", None, 1e-6),
]


def build_comparison_rows(case_name, theory, actual):
    rows = []
    for metric, theory_key, actual_key, component, tolerance in COMPARISON_METRICS:
        if component is not None:
            theory_value = float(theory[theory_key][component])
            actual_value = float(actual[actual_key][component])
        elif theory_key is None:
            theory_value = 0.0
            actual_value = float(actual[actual_key])
        else:
            theory_value = float(theory[theory_key])
            actual_value = float(actual[actual_key])

        abs_error = abs(actual_value - theory_value)
        rows.append({
            "case": case_name,
            "metric": metric,
            "theory": theory_value,
            "actual": actual_value,
            "abs_error": abs_error,
            "tolerance": tolerance,
            "passed": abs_error <= tolerance,
        })
    return rows


def format_number(value):
    return "{:.10g}".format(value)


def format_terminal_table(rows):
    headers = ["Case", "Metric", "Theory", "Actual", "Abs Error", "Tolerance", "Result"]
    values = [
        [
            row["case"],
            row["metric"],
            format_number(row["theory"]),
            format_number(row["actual"]),
            format_number(row["abs_error"]),
            format_number(row["tolerance"]),
            "PASS" if row["passed"] else "FAIL",
        ]
        for row in rows
    ]
    widths = [
        max(len(headers[index]), max((len(row[index]) for row in values), default=0))
        for index in range(len(headers))
    ]

    def render(row):
        return " | ".join(value.ljust(widths[index]) for index, value in enumerate(row))

    separator = "-+-".join("-" * width for width in widths)
    return "\n".join([render(headers), separator] + [render(row) for row in values])


def write_comparison_reports(rows, report_dir):
    os.makedirs(report_dir, exist_ok=True)
    markdown_path = os.path.join(report_dir, "payload_dataset_comparison.md")
    csv_path = os.path.join(report_dir, "payload_dataset_comparison.csv")

    passed_count = sum(1 for row in rows if row["passed"])
    failed_count = len(rows) - passed_count
    markdown_lines = [
        "# Payload Identification Dataset Comparison",
        "",
        "- Cases: {}".format(len({row["case"] for row in rows})),
        "- Metrics: {}".format(len(rows)),
        "- Passed: {}".format(passed_count),
        "- Failed: {}".format(failed_count),
        "",
        "| Case | Metric | Theory | Actual | Abs Error | Tolerance | Result |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for row in rows:
        markdown_lines.append(
            "| {case} | {metric} | {theory} | {actual} | {abs_error} | {tolerance} | {result} |".format(
                case=row["case"],
                metric=row["metric"],
                theory=format_number(row["theory"]),
                actual=format_number(row["actual"]),
                abs_error=format_number(row["abs_error"]),
                tolerance=format_number(row["tolerance"]),
                result="PASS" if row["passed"] else "FAIL"))
    with open(markdown_path, "w") as stream:
        stream.write("\n".join(markdown_lines) + "\n")

    with open(csv_path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["case", "metric", "theory", "actual", "abs_error", "tolerance", "result"])
        for row in rows:
            writer.writerow([
                row["case"],
                row["metric"],
                format_number(row["theory"]),
                format_number(row["actual"]),
                format_number(row["abs_error"]),
                format_number(row["tolerance"]),
                "PASS" if row["passed"] else "FAIL",
            ])

    return {"markdown": markdown_path, "csv": csv_path}


class PayloadDatasetReplayTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_payload_dataset_replay", anonymous=True)
        cls.wrench_pub = rospy.Publisher("/sri_ft_sensor/wrench", WrenchStamped, queue_size=20)
        cls.tf_broadcaster = tf.TransformBroadcaster()
        rospy.wait_for_service("/step_control", timeout=5.0)
        rospy.wait_for_service("/compute_payload", timeout=5.0)
        rospy.wait_for_service("/set_sampling_config", timeout=5.0)
        cls.step_control = rospy.ServiceProxy("/step_control", StepControl)
        cls.compute_payload = rospy.ServiceProxy("/compute_payload", ComputePayload)
        cls.set_sampling_config = rospy.ServiceProxy("/set_sampling_config", SetSamplingConfig)
        cls.dataset = cls.load_dataset()
        cls.report_dir = os.path.abspath(rospy.get_param(
            "~report_dir",
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "..",
                "build",
                "test_results",
                "tool_gravity_compensation")))
        cls.comparison_rows = []
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while cls.wrench_pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        if cls.wrench_pub.get_num_connections() == 0:
            raise RuntimeError("collect_load_profile_server did not subscribe to wrench topic")

    @staticmethod
    def load_dataset():
        default_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "datasets",
            "payload_test_dataset.json"))
        dataset_path = rospy.get_param("~dataset_path", default_path)
        with open(dataset_path) as stream:
            return json.load(stream)

    def clear_profiles(self):
        response = self.set_sampling_config(
            apply=True,
            clear_profiles=True,
            filter_window_size=3,
            min_stable_samples=3,
            max_stddev_force=1e-6,
            max_stddev_torque=1e-6,
            sample_timeout=1.0)
        self.assertTrue(response.success, response.message)

    def publish_sample_window(self, sample, wrench):
        pose = sample["sensor_pose_base"]
        position = pose["position_m"]
        quaternion = pose["quat_xyzw"]
        force = wrench["force_n"]
        torque = wrench["torque_nm"]

        for _ in range(10):
            now = rospy.Time.now()
            self.tf_broadcaster.sendTransform(
                position,
                quaternion,
                now,
                "sri_ft_sensor",
                "base_link")

            msg = WrenchStamped()
            msg.header.stamp = now
            msg.header.frame_id = "sri_ft_sensor"
            msg.wrench.force.x = force[0]
            msg.wrench.force.y = force[1]
            msg.wrench.force.z = force[2]
            msg.wrench.torque.x = torque[0]
            msg.wrench.torque.y = torque[1]
            msg.wrench.torque.z = torque[2]
            self.wrench_pub.publish(msg)
            rospy.sleep(0.02)
        rospy.sleep(0.05)

    def record_sample(self, profile_type, step_index, sample, wrench):
        self.publish_sample_window(sample, wrench)

        request = StepControlRequest()
        request.profile_type = profile_type
        request.step_index = step_index
        request.execute_motion = False
        request.sample_wrench = True
        request.manual_confirmed = True
        request.target_pose_name = "{}_{}".format(profile_type, step_index)
        response = self.step_control(request)

        self.assertTrue(response.accepted, response.message)
        self.assertTrue(response.sample_recorded, response.message)

    def replay_case(self, case):
        self.clear_profiles()
        samples = case["samples"]
        for index, sample in enumerate(samples):
            self.record_sample(
                StepControlRequest.TOOL_ONLY,
                index,
                sample,
                sample["tool_wrench_sensor"])
        for index, sample in enumerate(samples):
            self.record_sample(
                StepControlRequest.TOOL_PLUS_PAYLOAD,
                index + len(samples),
                sample,
                sample["total_wrench_sensor"])

        request = ComputePayloadRequest()
        request.compute = True
        request.min_samples = len(samples)
        response = self.compute_payload(request)

        self.assertTrue(response.success, "{}: {}".format(case["name"], response.message))
        self.assertTrue(response.result.success, "{}: {}".format(case["name"], response.result.message))

        theory = case["theory"]
        actual = {
            "tool_mass_kg": response.result.tool_mass,
            "mass_kg": response.result.mass_kg,
            "weight_n": response.result.weight_n,
            "com_sensor_m": [
                response.result.com_sensor.x,
                response.result.com_sensor.y,
                response.result.com_sensor.z,
            ],
            "residual_error": response.result.residual_error,
        }
        rows = build_comparison_rows(case["name"], theory, actual)
        self.comparison_rows.extend(rows)
        rospy.loginfo("\nPayload comparison for %s:\n%s",
                      case["name"], format_terminal_table(rows))

    def test_replays_payload_dataset_through_ros_services(self):
        self.assertEqual(
            [case["name"] for case in self.dataset["cases"]],
            [
                "ideal_nominal",
                "unzeroed_sensor_bias",
                "heavy_payload_normalized_condition",
                "rotated_sensor_frame",
                "farther_com_still_valid",
            ])
        for case in self.dataset["cases"]:
            self.replay_case(case)

        table = format_terminal_table(self.comparison_rows)
        paths = write_comparison_reports(self.comparison_rows, self.report_dir)
        rospy.loginfo("\nComplete payload dataset comparison:\n%s", table)
        rospy.loginfo("Markdown report: %s", paths["markdown"])
        rospy.loginfo("CSV report: %s", paths["csv"])

        failed_rows = [row for row in self.comparison_rows if not row["passed"]]
        self.assertFalse(failed_rows, "\n{}".format(table))


if __name__ == "__main__":
    import rostest
    rostest.rosrun("tool_gravity_compensation", "test_payload_dataset_replay",
                   PayloadDatasetReplayTest)
