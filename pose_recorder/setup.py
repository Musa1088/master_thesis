from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pose_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "resource"), glob("resource/*.png"))
    ],
    install_requires=['setuptools', 'PySide6'],
    zip_safe=True,
    maintainer='neura1',
    maintainer_email='neura1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_window = pose_recorder.main_window:main'
        ],
    },
)
