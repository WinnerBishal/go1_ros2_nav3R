from setuptools import find_packages, setup

package_name = 'go1_comms'

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
    maintainer='macwinner',
    maintainer_email='macwinner@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go1_twist_publisher = go1_comms.go1_twist_publisher:main',
            'execute_twist = go1_comms.execute_twist:main',
            'keyboard_cmd_publisher = go1_comms.keyboard_cmd_publihser:main'
        ],
    },
)
