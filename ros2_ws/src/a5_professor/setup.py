from setuptools import setup
from glob import glob

package_name = 'a5_professor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='Professor',
    maintainer_email='professor@example.com',
    description='MFE A5 professor: cone-map sim + grader.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = a5_professor.sim_node:main',
            'grader = a5_professor.grader:main',
        ],
    },
)
