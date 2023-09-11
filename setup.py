from setuptools import find_packages, setup

package_name = 'fws_publisher'

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
    maintainer='nishalangovender',
    maintainer_email='nishalan.govender@gmail.com',
    description='Converts Twist messages to joint position/velocity commands nad publishes them',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = fws_publisher.publisher:main',
        ],
    },
)
