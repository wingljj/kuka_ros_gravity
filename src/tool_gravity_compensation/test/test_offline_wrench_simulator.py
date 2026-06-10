#!/usr/bin/env python3
import importlib.util
import os
import unittest


def load_simulator_module():
    """Load offline_wrench_simulator module for testing.

    Safety note: exec_module is used with a hardcoded path derived from
    this test file's own location — no external input or user-controlled
    path is involved. This is a standard Python importlib pattern for
    loading modules outside the normal package hierarchy.
    """
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    module_path = os.path.join(package_dir, "scripts", "offline_wrench_simulator.py")
    spec = importlib.util.spec_from_file_location("offline_wrench_simulator", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class OfflineWrenchSimulatorTest(unittest.TestCase):

    def test_default_topic_is_namespaced_for_offline_simulation(self):
        simulator = load_simulator_module()

        self.assertEqual(
            simulator.DEFAULT_OFFLINE_WRENCH_TOPIC,
            "/tool_gravity_compensation/offline_wrench")
        self.assertNotEqual(
            simulator.DEFAULT_OFFLINE_WRENCH_TOPIC,
            "/sri_ft_sensor/wrench")

    def test_computes_gravity_wrench_in_sensor_frame(self):
        simulator = load_simulator_module()
        force, torque = simulator.compute_wrench(
            base_r_sensor=[1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0],
            mass_kg=2.0,
            com_sensor_m=[0.10, -0.20, 0.30],
            force_bias=[1.0, 2.0, 3.0],
            torque_bias=[0.5, -0.5, 0.25],
        )

        self.assertAlmostEqual(force[0], 1.0)
        self.assertAlmostEqual(force[1], 2.0)
        self.assertAlmostEqual(force[2], -2.0 * simulator.STANDARD_GRAVITY + 3.0)
        self.assertAlmostEqual(torque[0], (-0.20 * -2.0 * simulator.STANDARD_GRAVITY) + 0.5)
        self.assertAlmostEqual(torque[1], -(0.10 * -2.0 * simulator.STANDARD_GRAVITY) - 0.5)
        self.assertAlmostEqual(torque[2], 0.25)


if __name__ == "__main__":
    unittest.main()
