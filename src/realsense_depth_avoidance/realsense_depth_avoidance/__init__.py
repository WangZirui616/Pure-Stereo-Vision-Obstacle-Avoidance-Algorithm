from setuptools import setup

package_name = "realsense_depth_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    entry_points={
        "console_scripts": [
            "depth_gap_avoidance = realsense_depth_avoidance.depth_gap_avoidance:main",
        ],
    },
)