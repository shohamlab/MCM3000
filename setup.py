from setuptools import setup, find_packages
setup(
name='MCM3000',
version='0.2',
author='sstucker',
author_email='sstucker@nyu.edu',
homepage='https://github.com/shohamlab/MCM3000',
description='Python serial interface for Thorlabs MCM3000 Series 3-Axis Controller originally forked from amsikking/thorlabs_MCM3000.',
packages=['MCM3000'],
classifiers=[
'Programming Language :: Python :: 3',
'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
'Operating System :: OS Independent',
],
python_requires='>=3.6',
)
