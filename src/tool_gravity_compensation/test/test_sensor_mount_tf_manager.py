#!/usr/bin/env python3
import importlib.util
import math
import os
import unittest


def load_manager_module():
    """Load sensor_mount_tf_manager module for testing.

    Safety note: exec_module is used with a hardcoded path derived from
    this test file's own location — no external input or user-controlled
    path is involved. This is a standard Python importlib pattern for
    loading modules outside the normal package hierarchy.
    """
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    module_path = os.path.join(package_dir, "scripts", "sensor_mount_tf_manager.py")
    spec = importlib.util.spec_from_file_location("sensor_mount_tf_manager", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SensorMountTfManagerTest(unittest.TestCase):

    def test_make_transform_uses_requested_frames_and_euler_angles(self):
        manager = load_manager_module()
        transform = manager.make_transform(
            "tool0",
            "sri_ft_sensor",
            [0.1, -0.2, 0.3],
            [0.0, 0.0, math.pi / 2.0],
        )

        self.assertEqual(transform.header.frame_id, "tool0")
        self.assertEqual(transform.child_frame_id, "sri_ft_sensor")
        self.assertAlmostEqual(transform.transform.translation.x, 0.1)
        self.assertAlmostEqual(transform.transform.translation.y, -0.2)
        self.assertAlmostEqual(transform.transform.translation.z, 0.3)
        self.assertAlmostEqual(transform.transform.rotation.z, math.sqrt(0.5), places=6)
        self.assertAlmostEqual(transform.transform.rotation.w, math.sqrt(0.5), places=6)


if __name__ == "__main__":
    unittest.main()
