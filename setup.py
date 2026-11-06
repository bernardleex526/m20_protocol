from setuptools import setup

package_name = 'm20_protocol'

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
    description='DEEP Robotics M20 basic_server protocol: framing, commands, error codes',
    license='MIT',
    tests_require=['pytest'],
    python_requires='>=3.8',
)
