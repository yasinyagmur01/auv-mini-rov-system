from setuptools import setup

package_name = 'auv_lane_follow'

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
    description='Serit takibi',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_follow_node = auv_lane_follow.lane_follow_node:main',
        ],
    },
)
