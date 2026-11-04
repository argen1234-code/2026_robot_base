from setuptools import find_packages, setup

package_name = 'text_chat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyreadline3; platform_system=="Windows"'],
    zip_safe=True,
    maintainer='argen1234-code',
    maintainer_email='3106214334@qq.com',
    description='文本对话入口节点',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_chat = text_chat.text_chat:main'
        ],
    },
)
