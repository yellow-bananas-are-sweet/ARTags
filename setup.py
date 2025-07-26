from setuptools import setup

package_name = 'artag'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ArUco tag detection with PiCamera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'camera_node = artag.camera_node:main',
        'detector_node = artag.detector_node:main'
        ],
    },
)