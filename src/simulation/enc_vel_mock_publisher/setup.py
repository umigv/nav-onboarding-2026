from setuptools import find_packages, setup

package_name = "enc_vel_mock_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    package_data={"": ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryan",
    maintainer_email="ryan.liao0305@gmail.com",
    description="Publishes a simulated encoder velocity driving a square trajectory, with configurable drift/noise",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "enc_vel_mock_publisher = enc_vel_mock_publisher.enc_vel_mock_publisher:main",
        ],
    },
)
