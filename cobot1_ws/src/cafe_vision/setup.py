from setuptools import setup, find_packages

package_name = "cafe_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/calibration", ["calibration/board.yaml"]),
        ("share/" + package_name + "/training", ["training/cup.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "vision_node = cafe_vision.vision_node:main",
            "calibrate_node = cafe_vision.calibrate_node:main",
            "capture_node = cafe_vision.capture_node:main",
        ],
    },
)
