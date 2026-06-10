#!/usr/bin/env python3
import os
import unittest


def read_panel_source():
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(package_dir, "src", "tool_gravity_panel.cpp")
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


class ToolGravityPanelModeGatingTest(unittest.TestCase):

    def test_mode_combo_drives_launch_and_sampling_gates(self):
        source = read_panel_source()

        self.assertIn("onModeChanged", source)
        self.assertIn("modeAllowsSensorLaunch", source)
        self.assertIn("modeAllowsRobotLaunch", source)
        self.assertIn("isModeReadyForSampling", source)
        self.assertIn("currentIndexChanged(int)", source)
        self.assertIn("tool_button_->setEnabled(safe && mode_ready)", source)

    def test_record_sample_refuses_when_selected_mode_is_not_ready(self):
        source = read_panel_source()

        record_sample_start = source.index("void recordSample")
        step_call = source.index("step_client_.call", record_sample_start)
        mode_guard = source.index("isModeReadyForSampling", record_sample_start)
        self.assertLess(mode_guard, step_call)


if __name__ == "__main__":
    unittest.main()
