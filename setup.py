from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['yumi_demos'],
    package_dir={'': 'src'}
   # requires=['roscpp', 'rospy', 'tf']
)

setup(**setup_args)
