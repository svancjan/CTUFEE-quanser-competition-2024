from setuptools import find_packages, setup

package_name = 'win_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kodytota',
    maintainer_email='kodytota@todo.todo',
    description='Interface to windows environment running Quanser simulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'winInterface = win_interface.WindowsInterface:main',
		'testNode = win_interface.ROSSubscriber:main'
        ],
    },
)
