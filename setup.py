from glob import glob

from setuptools import find_packages, setup

package_name = 'guppy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panitan103',
    maintainer_email='panitan103@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guppy_driver_control = guppy.guppy_driver_control:main',
            'guppy_transmitted_example = guppy.guppy_transmitted_example:main',
        ],
    },
)
