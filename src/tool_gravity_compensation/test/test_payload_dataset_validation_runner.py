#!/usr/bin/env python3
import importlib.util
import io
import os
import tempfile
import unittest


def load_runner_module():
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    module_path = os.path.join(
        package_dir, "scripts", "run_payload_dataset_validation.py")
    spec = importlib.util.spec_from_file_location(
        "run_payload_dataset_validation", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PayloadDatasetValidationRunnerTest(unittest.TestCase):

    def setUp(self):
        self.runner = load_runner_module()

    def test_prints_detailed_report_after_successful_rostest(self):
        with tempfile.TemporaryDirectory() as report_dir:
            markdown_path = os.path.join(
                report_dir, "payload_dataset_comparison.md")
            csv_path = os.path.join(
                report_dir, "payload_dataset_comparison.csv")

            commands = []
            observed_test_results_dir = []
            previous_test_results_dir = os.environ.get("ROS_TEST_RESULTS_DIR")

            def successful_runner(command):
                commands.append(command)
                observed_test_results_dir.append(
                    os.environ.get("ROS_TEST_RESULTS_DIR"))
                with open(markdown_path, "w") as stream:
                    stream.write(
                        "# Payload Identification Dataset Comparison\n\n"
                        "| Case | Metric | Theory | Actual | Abs Error | Tolerance | Result |\n"
                        "| ideal_nominal | payload_mass_kg | 2.5 | 2.5 | 0 | 1e-6 | PASS |\n")
                with open(csv_path, "w") as stream:
                    stream.write("case,metric,theory,actual,abs_error,tolerance,result\n")
                return 0

            stdout = io.StringIO()
            stderr = io.StringIO()
            return_code = self.runner.run_validation(
                report_dir,
                command_runner=successful_runner,
                stdout=stdout,
                stderr=stderr)

        self.assertEqual(return_code, 0)
        self.assertEqual(
            commands,
            [["rostest", "tool_gravity_compensation", "payload_dataset_replay.test"]])
        self.assertEqual(observed_test_results_dir, [os.path.dirname(report_dir)])
        self.assertEqual(os.environ.get("ROS_TEST_RESULTS_DIR"),
                         previous_test_results_dir)
        self.assertIn("Theory | Actual | Abs Error", stdout.getvalue())
        self.assertIn("ideal_nominal", stdout.getvalue())
        self.assertIn(markdown_path, stdout.getvalue())
        self.assertIn(csv_path, stdout.getvalue())
        self.assertEqual(stderr.getvalue(), "")

    def test_propagates_rostest_failure_without_printing_stale_report(self):
        with tempfile.TemporaryDirectory() as report_dir:
            markdown_path = os.path.join(
                report_dir, "payload_dataset_comparison.md")
            csv_path = os.path.join(
                report_dir, "payload_dataset_comparison.csv")
            with open(markdown_path, "w") as stream:
                stream.write("stale PASS report\n")
            with open(csv_path, "w") as stream:
                stream.write("stale csv\n")

            stdout = io.StringIO()
            stderr = io.StringIO()

            return_code = self.runner.run_validation(
                report_dir,
                command_runner=lambda _command: 7,
                stdout=stdout,
                stderr=stderr)

            self.assertEqual(return_code, 7)
            self.assertEqual(stdout.getvalue(), "")
            self.assertFalse(os.path.exists(markdown_path))
            self.assertFalse(os.path.exists(csv_path))
            self.assertIn("rostest failed", stderr.getvalue())

    def test_prints_current_report_even_when_rostest_fails(self):
        with tempfile.TemporaryDirectory() as report_dir:
            markdown_path = os.path.join(
                report_dir, "payload_dataset_comparison.md")

            def failing_runner(_command):
                with open(markdown_path, "w") as stream:
                    stream.write(
                        "# Payload Identification Dataset Comparison\n\n"
                        "| ideal_nominal | payload_mass_kg | 2.5 | 2.4 | 0.1 | 1e-6 | FAIL |\n")
                return 3

            stdout = io.StringIO()
            stderr = io.StringIO()
            return_code = self.runner.run_validation(
                report_dir,
                command_runner=failing_runner,
                stdout=stdout,
                stderr=stderr)

        self.assertEqual(return_code, 3)
        self.assertIn("payload_mass_kg", stdout.getvalue())
        self.assertIn("FAIL", stdout.getvalue())
        self.assertIn("rostest failed", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
