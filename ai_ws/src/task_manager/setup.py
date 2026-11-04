from setuptools import find_packages, setup

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/task_manager.yaml']),
        ('share/' + package_name + '/launch', ['launch/task_manager.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='argen1234-code',
    maintainer_email='3106214334@qq.com',
    description='Target search state machine and velocity arbitration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_mux = task_manager.velocity_mux:main',
            'target_searcher = task_manager.target_searcher:main',
        ],
    },
)
