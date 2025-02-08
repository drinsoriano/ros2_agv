from setuptools import find_packages, setup

package_name = 'ws2812b_controller'

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
    maintainer_email='drin.soriano@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ws2812b_node = ws2812b_controller.ws2812b_node:main',  # ✅ Add this line
        ],
    },
)
