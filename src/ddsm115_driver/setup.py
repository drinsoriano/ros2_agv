from setuptools import find_packages, setup

package_name = 'ddsm115_driver'

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
    maintainer='circuitcoder1101',
    maintainer_email='circuitcoder1101@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ddsm115_node = ddsm115_driver.ddsm115_node:main',
        ],
    },
)
