from setuptools import setup

package_name = 'ourbotmanager'

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
    maintainer='ubuntu',
    maintainer_email='davidsomerville@comcast.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "base_motor_controller=ourbotmanager.base_motor_controller:main",
            "base_monitor=ourbotmanager.base_monitor:main"
        ], 
    },
)
