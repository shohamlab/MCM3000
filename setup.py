from setuptools import setup, find_packages
setup(
name='MCM3000',
version='0.1.0',
author='sstucker',
author_email='sstucker@nyu.edu',
description='Python serial interface for Thorlabs MCM3000 Series 3-Axis Controller originally forked from amsikking/thorlabs_MCM3000.',
packages=find_packages(),
classifiers=[
'Programming Language :: Python :: 3',
'License :: OSI Approved :: MIT License',
'Operating System :: OS Independent',
],
python_requires='>=3.6',
)