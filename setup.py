from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['manipulation_planning_suite'],
    package_dir={'': 'python_src'},
    requires=['rospy', 'numpy', 'yaml', 'rtree',
              'rospkg']
)

setup(**setup_args)