#!/usr/bin/env python3
import importlib.util
import os
import tempfile
import unittest


def load_replay_module():
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    module_path = os.path.join(package_dir, "test", "test_payload_dataset_replay.py")
    spec = importlib.util.spec_from_file_location("test_payload_dataset_replay", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PayloadComparisonReportTest(unittest.TestCase):

    def setUp(self):
        self.replay = load_replay_module()
        self.theory = {
            "tool_mass_kg": 1.5,
            "payload_mass_kg": 2.5,
            "payload_weight_n": 24.516625,
            "payload_com_sensor_m": [0.1, -0.04, 0.2],
        }
        self.actual = {
            "tool_mass_kg": 1.5000001,
            "mass_kg": 2.5000002,
            "weight_n": 24.516626,
            "com_sensor_m": [0.1000001, -0.0400001, 0.2000001],
            "residual_error": 2e-8,
        }

    def test_builds_detailed_rows_and_terminal_table(self):
        rows = self.replay.build_comparison_rows("ideal_nominal", self.theory, self.actual)

        self.assertEqual(
            [row["metric"] for row in rows],
            [
                "tool_mass_kg",
                "payload_mass_kg",
                "payload_weight_n",
                "payload_com_x_m",
                "payload_com_y_m",
                "payload_com_z_m",
                "residual_error",
            ])
        self.assertTrue(all(row["passed"] for row in rows))

        table = self.replay.format_terminal_table(rows)
        self.assertIn("Theory", table)
        self.assertIn("Actual", table)
        self.assertIn("Abs Error", table)
        self.assertIn("Tolerance", table)
        self.assertIn("PASS", table)
        self.assertIn("ideal_nominal", table)

    def test_writes_markdown_and_csv_reports(self):
        rows = self.replay.build_comparison_rows("ideal_nominal", self.theory, self.actual)

        with tempfile.TemporaryDirectory() as report_dir:
            paths = self.replay.write_comparison_reports(rows, report_dir)

            self.assertTrue(os.path.isfile(paths["markdown"]))
            self.assertTrue(os.path.isfile(paths["csv"]))
            with open(paths["markdown"]) as stream:
                markdown = stream.read()
            with open(paths["csv"]) as stream:
                csv_text = stream.read()

        self.assertIn("# Payload Identification Dataset Comparison", markdown)
        self.assertIn("| Case | Metric | Theory | Actual | Abs Error | Tolerance | Result |", markdown)
        self.assertIn("ideal_nominal", markdown)
        self.assertIn("PASS", markdown)
        self.assertIn("case,metric,theory,actual,abs_error,tolerance,result", csv_text)


if __name__ == "__main__":
    unittest.main()
