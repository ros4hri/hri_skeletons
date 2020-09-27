from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	packages=['hri_skeletons','hri_skeletons.modules'],
	package_dir={'': 'src'}
	)

setup(**d)
