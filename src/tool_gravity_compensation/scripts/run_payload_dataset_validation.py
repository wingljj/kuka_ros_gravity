#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys


PACKAGE_NAME = "tool_gravity_compensation"
ROSTEST_FILE = "payload_dataset_replay.test"
MARKDOWN_REPORT = "payload_dataset_comparison.md"
CSV_REPORT = "payload_dataset_comparison.csv"


def default_report_dir():
    test_results_dir = os.environ.get("CATKIN_TEST_RESULTS_DIR")
    if test_results_dir:
        return os.path.join(test_results_dir, PACKAGE_NAME)

    try:
        import rospkg
        package_dir = rospkg.RosPack().get_path(PACKAGE_NAME)
        workspace_dir = os.path.abspath(os.path.join(package_dir, "..", ".."))
    except Exception:
        package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        workspace_dir = os.path.abspath(os.path.join(package_dir, "..", ".."))

    return os.path.join(
        workspace_dir, "build", "test_results", PACKAGE_NAME)


def _return_code(result):
    return result.returncode if hasattr(result, "returncode") else int(result)


def _report_paths(report_dir):
    return {
        "markdown": os.path.join(report_dir, MARKDOWN_REPORT),
        "csv": os.path.join(report_dir, CSV_REPORT),
    }


def _remove_stale_reports(report_dir):
    for path in _report_paths(report_dir).values():
        if os.path.exists(path):
            os.remove(path)


def _print_markdown_report(markdown_path, stdout):
    stdout.write("\n=== Payload Dataset Validation Report ===\n\n")
    with open(markdown_path) as stream:
        stdout.write(stream.read())


def run_validation(report_dir, command_runner=subprocess.call,
                   stdout=sys.stdout, stderr=sys.stderr):
    os.makedirs(report_dir, exist_ok=True)
    paths = _report_paths(report_dir)
    _remove_stale_reports(report_dir)

    command = ["rostest", PACKAGE_NAME, ROSTEST_FILE]
    previous_test_results_dir = os.environ.get("ROS_TEST_RESULTS_DIR")
    os.environ["ROS_TEST_RESULTS_DIR"] = os.path.dirname(report_dir)
    try:
        return_code = _return_code(command_runner(command))
    finally:
        if previous_test_results_dir is None:
            os.environ.pop("ROS_TEST_RESULTS_DIR", None)
        else:
            os.environ["ROS_TEST_RESULTS_DIR"] = previous_test_results_dir

    if return_code != 0:
        if os.path.isfile(paths["markdown"]):
            _print_markdown_report(paths["markdown"], stdout)
        stderr.write(
            "payload dataset rostest failed with exit code {}\n".format(
                return_code))
        return return_code

    missing_paths = [
        path for path in paths.values() if not os.path.isfile(path)
    ]
    if missing_paths:
        stderr.write("payload dataset report missing after successful rostest:\n")
        for path in missing_paths:
            stderr.write("- {}\n".format(path))
        return 2

    _print_markdown_report(paths["markdown"], stdout)
    stdout.write("\nReport files:\n")
    stdout.write("- Markdown: {}\n".format(paths["markdown"]))
    stdout.write("- CSV: {}\n".format(paths["csv"]))
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Run the payload dataset ROS replay and print its detailed report.")
    parser.add_argument(
        "--report-dir",
        default=default_report_dir(),
        help="Directory containing payload_dataset_comparison.md/csv.")
    args = parser.parse_args()
    return run_validation(os.path.abspath(args.report_dir))


if __name__ == "__main__":
    sys.exit(main())
