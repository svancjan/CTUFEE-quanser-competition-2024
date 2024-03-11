from setuptools import find_packages, setup

package_name = 'interfaceNode'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('lib/' + package_name, [package_name + '/tcpConfiguration.yaml',package_name + '/sensorConfiguration.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='otakar',
    maintainer_email='kodytota@fel.cvut.cz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'ROSInterface=interfaceNode.ROSInterface:main'
        ],
    },
)
