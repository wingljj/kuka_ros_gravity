#!/usr/bin/env python3
import importlib.util
import math
import os
from types import SimpleNamespace
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

    def test_service_rejects_non_finite_and_out_of_range_mount_updates(self):
        manager_module = load_manager_module()
        manager = object.__new__(manager_module.SensorMountTfManager)
        manager.parent_frame = "tool0"
        manager.child_frame = "sri_ft_sensor"
        manager.xyz = [0.0, 0.0, 0.0]
        manager.rpy = [0.0, 0.0, 0.0]
        manager.transform = manager_module.make_transform(
            manager.parent_frame, manager.child_frame, manager.xyz, manager.rpy)

        cases = [
            SimpleNamespace(parent_frame="tool0", child_frame="sri_ft_sensor",
                            x=math.nan, y=0.0, z=0.0,
                            roll=0.0, pitch=0.0, yaw=0.0, save_to_params=False),
            SimpleNamespace(parent_frame="tool0", child_frame="sri_ft_sensor",
                            x=manager_module.MAX_TRANSLATION_M + 0.01, y=0.0, z=0.0,
                            roll=0.0, pitch=0.0, yaw=0.0, save_to_params=False),
            SimpleNamespace(parent_frame="tool0", child_frame="tool0",
                            x=0.0, y=0.0, z=0.0,
                            roll=0.0, pitch=0.0, yaw=0.0, save_to_params=False),
            SimpleNamespace(parent_frame="tool0", child_frame="sri_ft_sensor",
                            x=0.0, y=0.0, z=0.0,
                            roll=0.0, pitch=manager_module.MAX_ROTATION_RAD + 0.01,
                            yaw=0.0, save_to_params=False),
        ]

        for request in cases:
            with self.subTest(request=request):
                response = manager.handle_set_sensor_mount(request)
                self.assertFalse(response.success, response.message)
                self.assertEqual(manager.parent_frame, "tool0")
                self.assertEqual(manager.child_frame, "sri_ft_sensor")
                self.assertEqual(manager.xyz, [0.0, 0.0, 0.0])
                self.assertEqual(manager.rpy, [0.0, 0.0, 0.0])

    def test_service_accepts_valid_mount_update_after_validation(self):
        manager_module = load_manager_module()
        manager = object.__new__(manager_module.SensorMountTfManager)
        manager.parent_frame = "tool0"
        manager.child_frame = "sri_ft_sensor"
        manager.xyz = [0.0, 0.0, 0.0]
        manager.rpy = [0.0, 0.0, 0.0]
        manager.transform = manager_module.make_transform(
            manager.parent_frame, manager.child_frame, manager.xyz, manager.rpy)

        response = manager.handle_set_sensor_mount(SimpleNamespace(
            parent_frame="tool0",
            child_frame="sri_ft_sensor",
            x=0.1,
            y=-0.2,
            z=0.3,
            roll=0.0,
            pitch=0.0,
            yaw=math.pi / 2.0,
            save_to_params=False,
        ))

        self.assertTrue(response.success, response.message)
        self.assertEqual(manager.xyz, [0.1, -0.2, 0.3])
        self.assertEqual(manager.rpy, [0.0, 0.0, math.pi / 2.0])


if __name__ == "__main__":
    unittest.main()
