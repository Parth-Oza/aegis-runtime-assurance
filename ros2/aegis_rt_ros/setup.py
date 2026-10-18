from setuptools import setup

package_name = "aegis_rt_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, "aegis_rt"],
    package_dir={
        package_name: package_name,
        "aegis_rt": "../../src/aegis_rt",
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/command_guard.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Parth Oza",
    maintainer_email="ozaparth055@gmail.com",
    description="ROS 2 adapters for Aegis Runtime Assurance.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "command_guard = aegis_rt_ros.node:main",
        ],
    },
)
