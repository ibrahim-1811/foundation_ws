from setuptools import find_packages, setup

package_name = 'advance_comm_demo'

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
    maintainer='ibhu',
    maintainer_email='immemon1811@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qos_talker = advance_comm_demo.talker_qos:main',
            'qos_listener = advance_comm_demo.listner_qos:main',
            'lifecycle_talker = advance_comm_demo.lifecycle_talker:main',

        ],
    },
)
