from setuptools import find_packages, setup

package_name = 'turtlesim_demo'

# This calls the setup() function to specify package details and configuration
setup(
    name=package_name,  # Name of the package
    version='0.0.0',  # Initial version
    packages=find_packages(exclude=['test']),  # Automatically find all packages except 'test'
    data_files=[
        # Install resource file for ament package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml in the share directory
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['turtlesim_demo/demo_launch.py']),
    ],
    install_requires=['setuptools'],  # Dependencies required to install the package
    zip_safe=True,  # Package can be safely installed as a .zip file
    maintainer='ibhu',  # Maintainer name
    maintainer_email='immemon1811@gmail.com',  # Maintainer email
    description='TODO: Package description',  # Short description of the package
    license='TODO: License declaration',  # License information
    tests_require=['pytest'],  # Test dependencies
    entry_points={
        # Define console scripts for command-line executables
        'console_scripts': [
            'circle_driver = turtlesim_demo.circle_driver:main',
            'pose_logger =   turtlesim_demo.pose_logger:main',
            'server_client = turtlesim_demo.server_client:main',
            'action_server = turtlesim_demo.action_server:main',
            'action_client = turtlesim_demo.action_client:main',
        ],
    },
)
