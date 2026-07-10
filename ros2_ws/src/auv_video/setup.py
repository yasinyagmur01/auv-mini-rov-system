from setuptools import setup

package_name = 'auv_video'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AUV Takimi',
    maintainer_email='thelicbd@yeniaycerez.com',
    description='Video akisi',
    license='MIT',
    entry_points={
        'console_scripts': [
            'video_stream_node = auv_video.video_stream_node:main',
        ],
    },
)
