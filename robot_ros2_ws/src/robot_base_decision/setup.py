from setuptools import find_packages, setup

package_name = 'robot_base_decision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/decision.launch.py']),
    ],
    package_data={package_name: ['person_approach.html']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='argen',
    maintainer_email='dev@example.com',
    description='Hierarchical decision layer for navigation, voice and vision.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_coordinator = robot_base_decision.task_coordinator:main',
            'nav_decision = robot_base_decision.nav_decision:main',
            'voice_decision = robot_base_decision.voice_decision:main',
            'vision_decision = robot_base_decision.vision_decision:main',
        ],
    },
)
