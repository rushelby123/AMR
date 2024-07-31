from setuptools import find_packages, setup
from glob import glob

package_name = 'launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task1.launch.py']),
        ('share/' + package_name + '/launch', ['launch/task2.launch.py']),
        ('share/' + package_name + '/launch', ['launch/task3.launch.py']),
        ('share/' + package_name + '/launch', ['launch/task4.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriele',
    maintainer_email='gabriele@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_explorer = launch_pkg.my_explorer:main",
        ],
    },
)
