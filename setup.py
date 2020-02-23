from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['life_detection_plugin', 'arm_control_plugin', 'current_sensor_plugin'],
    package_dir={'': 'src'},
)

# Unfortunately ignored by ROS, but kept here a documentation
d['requires'] = ['pyqtgraph']

setup(**d)