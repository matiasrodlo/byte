# Always prefer setuptools over distutils
from setuptools import setup, find_packages
# To use a consistent encoding
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

import sys
sys.path.append("./byte")
from version import __version__

# Get the long description from README.md
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='byte',

    # Versions should comply with PEP440.  For a discussion on single-sourcing
    # the version across setup.py and the project code, see
    # https://packaging.python.org/en/latest/single_source_version.html
    version=__version__,

    description='A comprehensive quadruped robot control system for Raspberry Pi',
    long_description=long_description,
    long_description_content_type='text/markdown',

    # The project's main homepage.
    url='https://github.com/matiasrodlo/byte',
    # Choose your license
    license='MIT',
    zip_safe=False,
    
    # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        # How mature is this project? Common values are
        #   3 - Alpha
        #   4 - Beta
        #   5 - Production/Stable
        'Development Status :: 4 - Beta',

        # Indicate who your project is intended for
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Scientific/Engineering :: Robotics',
        'Topic :: Software Development :: Libraries :: Python Modules',

        # Pick your license as you wish (should match "license" above)
        'License :: OSI Approved :: MIT License',

        # Specify the Python versions you support here. In particular, ensure
        # that you indicate whether you support Python 2, Python 3 or both.
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12',
        
        # Operating System
        'Operating System :: POSIX :: Linux',
        
        # Hardware
        'Topic :: System :: Hardware',
    ],

    # What does your project relate to?
    keywords='robotics quadruped robot raspberry pi GPIO servo control IMU sensor',

    # You can just specify the packages manually here if your project is
    # simple. Or you can use find_packages().
    packages=find_packages(exclude=['doc', 'tests*', 'examples']),

    # List run-time dependencies here.  These will be installed by pip when
    # your project is installed. For an analysis of "install_requires" vs pip's
    # requirements files see:
    # https://packaging.python.org/en/latest/requirements.html
    install_requires=[
        'robot_hat>=2.0.0',
        'numpy>=1.19.0',
        'readchar>=2.0.0',
        'spidev>=3.5',
        'gpiozero>=1.6.0',
        'smbus2>=0.4.0',
    ],

    # Optional dependencies
    extras_require={
        'dev': [
            'pytest>=6.0.0',
            'pytest-cov>=2.10.0',
            'black>=21.0.0',
            'flake8>=3.8.0',
        ],
        'docs': [
            'sphinx>=3.0.0',
            'sphinx-rtd-theme>=0.5.0',
        ],
    },

    # To provide executable scripts, use entry points in preference to the
    # "scripts" keyword. Entry points provide cross-platform support and allow
    # pip to create the appropriate form of executable for the target platform.
    entry_points={
        'console_scripts': [
            'robotdog=byte:__main__',
        ],
    },
    
    # Include package data
    include_package_data=True,
    package_data={
        'byte': ['*.conf', 'sounds/*'],
    },
    
    # Project URLs
    project_urls={
        'Bug Reports': 'https://github.com/matiasrodlo/byte/issues',
        'Source': 'https://github.com/matiasrodlo/byte',
        'Documentation': 'https://github.com/matiasrodlo/byte#readme',
    },
)
