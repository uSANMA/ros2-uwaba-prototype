import os
from setuptools import find_packages, setup
from glob import glob

package_name = "uwaba_prototype_testes"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="frantches",
    maintainer_email="frantches@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "uros_std_msg_teste = uwaba_prototype_testes.uros_std_msg_teste:main",
            "uros_cmd_vel_teste = uwaba_prototype_testes.uros_cmd_vel_teste:main",
            "state_publisher = uwaba_prototype_testes.state_publisher:main",
            "uros_joint_state_teste = uwaba_prototype_testes.uros_joint_state_teste:main",
            "timing_testes = uwaba_prototype_testes.timing_testes:main"
        ],
    },
)
