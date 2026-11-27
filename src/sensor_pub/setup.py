from setuptools import find_packages, setup

package_name = 'sensor_pub'

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
    maintainer='zouyixu',
    maintainer_email='2025117964@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'csv_sensor_pub = sensor_pub.csv_sensor_pub:main',
            'moss_sensor_pub = sensor_pub.moss_sensor_pub:main',
            'sonar_sim = sensor_pub.sonar_sim:main',
        ],
    },
)
