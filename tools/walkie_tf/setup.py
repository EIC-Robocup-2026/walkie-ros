from setuptools import find_packages, setup

package_name = 'walkie_tf'

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
    maintainer='walkie',
    maintainer_email='sutigran2557@gmail.com',
    description='TF transform lookup service node for Walkie robots',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'tf_server = walkie_tf.tf_server_node:main',
        ],
    },
)
