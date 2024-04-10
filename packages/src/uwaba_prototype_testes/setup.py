from setuptools import find_packages, setup

package_name = 'uwaba_prototype_testes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='frantches',
    maintainer_email='frantches@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "uros_std_msg_teste = uwaba_prototype_testes.uros_std_msg_teste:main",
            "uros_cmd_vel_teste = uwaba_prototype_testes.uros_cmd_vel_teste:main"
        ],
    },
)
