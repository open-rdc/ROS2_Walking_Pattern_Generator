from setuptools import find_packages, setup

package_name = 'robot_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'xml', 'yaml'],
    zip_safe=True,
    maintainer='Yusuke-Yamasaki-555',
    maintainer_email='yuu5364yuu@gmail.com',
    description='Tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
