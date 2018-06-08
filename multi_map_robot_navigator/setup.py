from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['multi_map_robot_navigator'],
    package_dir={'multi_map_robot_navigator': 'ros/src/multi_map_robot_navigator'}
)

setup(**d)
