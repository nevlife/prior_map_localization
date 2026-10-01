from glob import glob

from setuptools import setup

package_name = "prior_map_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("lib/" + package_name, ["scripts/prepare_map.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "wheel_twist_relay = prior_map_localization.wheel_twist_relay:main",
            "gps_cov_floor = prior_map_localization.gps_cov_floor:main",
            "graph_initialpose = prior_map_localization.graph_initialpose:main",
            "odom_twist_relay = prior_map_localization.odom_twist_relay:main",
            "scan_start_relay = prior_map_localization.scan_start_relay:main",
            "lio_twist_relay = prior_map_localization.lio_twist_relay:main",
        ],
    },
)
