from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/ring_detector.yaml',
            'config/hand_detector.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/ring_detector.launch.py',
            'launch/target_viewer.launch.py',
            'launch/hand_viewer.launch.py',
            'launch/dual_viewer.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='argen1234-code',
    maintainer_email='3106214334@qq.com',
    description='A4 two-ring target detection and depth localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ring_detector_demo = perception.ring_detector:main',
            'ring_detector_node = perception.ring_detector_node:main',
            'hand_detector_node = perception.hand_detector_node:main',
            'target_viewer = perception.target_viewer:main',
            'dual_viewer = perception.dual_viewer:main',
        ],
    },
)
