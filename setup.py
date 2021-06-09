from setuptools import setup

package_name = 'k3_thermometer_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alph',
    maintainer_email='alphonsustay@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_reader_script = k3_thermometer_ros2.temperature_reader_script:main',
            'temperature_sql_connector = sql_connector.sql_connector_node:main'
        ],
    },
)
