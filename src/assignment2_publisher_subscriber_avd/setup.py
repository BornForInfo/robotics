from setuptools import find_packages, setup

package_name = 'assignment2_publisher_subscriber_avd'

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
    maintainer='lenari',
    maintainer_email='lenari@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = assignment2_publisher_subscriber_avd.publisher:main',
            'subscriber = assignment2_publisher_subscriber_avd.subscriber:main'
        ],
    },
)
