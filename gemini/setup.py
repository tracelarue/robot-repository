from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gemini'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trace',
    maintainer_email='traceglarue@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_to_text = gemini.text_to_text:main',
            'multimodal = gemini.MultiModal_image_as_tool:main',
            'multimodal_robot = gemini.MultiModal_robot:main'
        ],
    },
)
