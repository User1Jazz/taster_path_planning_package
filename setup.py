import os
from glob import glob
from setuptools import setup

package_name = 'path_planning_package'
components = 'path_planning_package/components'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, components],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob(os.path.join('launch', '*_launch.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FormulaStudentTeamNapier',
    maintainer_email='FormulaStudent@napier.ac.uk',
    description='''This package is responsible for planning the trajectory for the vehicle through the track. 
                    It uses data from the Perception package, which includes coordinates of differently coloured cones discovered on the track, 
                    and calculates a reliable path within the boundaries.''',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning=path_planning_package.path_planning:main'
        ],
    },
)
