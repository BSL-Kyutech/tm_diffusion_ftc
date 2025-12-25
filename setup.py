import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'diffusion_model_fault_tolerance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("shere",package_name, "model"), glob("diffusion_model_fault_tolerance/model/*")),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TaichiMogami',
    maintainer_email='mogami.taichi550@mail.kyutech.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gen_circle = diffusion_model_fault_tolerance.experiment.gen_circle:main',
        ],
    },
)