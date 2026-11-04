from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'largemodel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 只安装配置模板；不要把备份/本地密钥文件带进 install
        (os.path.join('share', package_name, 'config'), [
            'config/large_model_interface.yaml',
            'config/yahboom.yaml',
        ]),
        # 只安装静态资源；运行产物（tts_output、截图、视频）不入 install
        (os.path.join('share', package_name, 'resources_file'), [
            'resources_file/usb_test.jpg',
        ]),
        (os.path.join('share', package_name, 'resources_file', 'scan_table'), [
            'resources_file/scan_table/test_table.jpg',
        ]),
        (os.path.join('share', package_name, 'resources_file', 'visual_positioning'), [
            'resources_file/visual_positioning/test_image.jpg',
        ]),
        (os.path.join('share', package_name, 'resources_file', 'system_vioce', 'zh'), [
            'resources_file/system_vioce/zh/longwan-women-1.mp3',
            'resources_file/system_vioce/zh/longwan-women-2.mp3',
        ]),
        (os.path.join('share', package_name, 'resources_file', 'system_vioce', 'en'), [
            'resources_file/system_vioce/en/longxiaochun-women-1.mp3',
            'resources_file/system_vioce/en/longxiaochun-women-2.mp3',
        ]),
    ],
    install_requires=['setuptools','ollama'],
    zip_safe=True,
    maintainer='argen1234-code',
    maintainer_email='3106214334@qq.com',
    description='智能垃圾桶的语音识别/合成、云端大模型与视觉节点',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asr = largemodel.asr:main',
            'model_service = largemodel.model_service:main',
            'tts_only = largemodel.tts_only:main',
            'nav_task = largemodel.nav_task:main',
            'motion_gate = largemodel.motion_gate:main',
        ],
    },
)
