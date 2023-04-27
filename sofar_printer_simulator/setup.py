import os
from glob import glob
from setuptools import setup

package_name = 'sofar_printer_simulator'
lib = 'sofar_printer_simulator/lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lib],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "resource"), glob("resource/*.png")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simone',
    maintainer_email='simone.maccio@edu.unige.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'printer_sim_node = sofar_printer_simulator.printer_sim_node:main', 
            'mot_x_node = sofar_printer_simulator.motor_x_controller:main', 
            'mot_y_node = sofar_printer_simulator.motor_y_controller:main', 
        ],
    },
)
