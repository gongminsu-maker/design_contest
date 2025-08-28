from setuptools import find_packages, setup

package_name = 'rs485'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gongminsu',
    maintainer_email='gongminsu@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "receiver = rs485.receiver:main",
            "transceiver = rs485.transceiver:main",
            "id_baud = rs485.id_baud:main",
            "cmd_vel = rs485.cmd_vel:main",
            "testcode = rs485.testcode:main",
            "base_broad = rs485.base_broad:main",
            "co = rs485.co",
            "node = rs485.node:main",
            "zmp_test_imu = rs485.zmp_test_imu:main",
            "TSS_test = rs485.TSS_test:main",
            "base_broad_imu = rs485.base_broad_imu:main",

            
        ],
    },
)
