from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ctu_quanser_comp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Otakar Kodytek, Ondrej Mikolas, Honza Kohout, Jakub Macar',
    author_email='kodytota@fel.cvut.cz, mikolond@fel.cvut.cz, kohouj31@fel.cvut.cz, macarjak@fel.cvut.cz',
    maintainer='Otakar Kodytek',
    maintainer_email='kodytota@fel.cvut.cz',
    description='CTU quanser student competition project',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulationInterface = ctu_quanser_comp.windowsInterface.WindowsInterface:main',
            'vision = ctu_quanser_comp.vision.vision_node:main',
            'lateralPlanning = ctu_quanser_comp.lateralPlanning.lateral_plan_node:main'
        ],
    },
)
