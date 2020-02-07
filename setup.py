from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['life_detection_plugin', 'arm_control_plugin'],
    package_dir={'': 'src'},
)

setup(**d)