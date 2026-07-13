from setuptools import setup

package_name = 'auv_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets', 'PyYAML'],
    zip_safe=True,
    maintainer='AUV Takimi',
    maintainer_email='thelicbd@yeniaycerez.com',
    description='Gorev durum makinesi ve kontrol primitifleri',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_node = auv_mission.mission_node:main',
        ],
    },
)
