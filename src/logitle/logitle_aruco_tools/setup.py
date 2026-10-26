from glob import glob
from setuptools import find_packages, setup

package_name = "logitle_aruco_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="heeyeon",
    maintainer_email="heeyeon@example.com",
    description="TurtleBot3 ArUco marker pose viewing, auto alignment, and TF correction tools.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "logitle_aruco_pose_viewer = logitle_aruco_tools.logitle_aruco_pose_viewer:main",
            "logitle_aruco_auto_align = logitle_aruco_tools.logitle_aruco_auto_align:main",
            "logitle_align_and_correct_action_server = logitle_aruco_tools.logitle_align_and_correct_action_server:main",
            "logitle_aruco_pose_corrector_action_server = logitle_aruco_tools.logitle_aruco_pose_corrector_action_server:main",
            "logitle_aruco_tf_corrector = logitle_aruco_tools.logitle_aruco_tf_corrector:main",
            "logitle_tf_continuity_monitor = logitle_aruco_tools.logitle_tf_continuity_monitor:main",
            "logitle_marker_map = logitle_aruco_tools.logitle_marker_map:main",
            "logitle_marker_localization_demo = logitle_aruco_tools.logitle_marker_localization:main",
        ],
    },
)
