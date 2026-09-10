from setuptools import setup
from glob import glob

package_name = 'a5_student'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='MFE A5 student template.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = a5_student.planner_node:main',
            'controller_node = a5_student.controller_node:main',
        ],
    },
)
