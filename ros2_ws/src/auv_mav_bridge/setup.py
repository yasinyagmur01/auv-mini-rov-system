from setuptools import setup

package_name = 'auv_mav_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='AUV Takimi',
    maintainer_email='thelicbd@yeniaycerez.com',
    description='pymavlink tabanli ArduSub - ROS 2 koprusu',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = auv_mav_bridge.bridge_node:main',
        ],
    },
)
