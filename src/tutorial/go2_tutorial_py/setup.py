from setuptools import find_packages, setup
from glob import glob

package_name = 'go2_tutorial_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob("launch/*.launch.py")),
        ('share/' + package_name + "/params", glob("params/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gcu68',
    maintainer_email='1906550150fcs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'go2_ctrl = go2_tutorial_py.go2_ctrl:main',
            'go2_state = go2_tutorial_py.go2_state:main',
            'go2_cruising_service = go2_tutorial_py.go2_cruising_service:main',
            'go2_cruising_client = go2_tutorial_py.go2_cruising_client:main',
            'go2_nav_server = go2_tutorial_py.go2_nav_server:main',
            'go2_nav_client = go2_tutorial_py.go2_nav_client:main',
        ],
    },
)
