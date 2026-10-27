import glob
from setuptools import setup

package_name = 'f1tenth_websim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['web/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob.glob('maps/*.yaml') + glob.glob('maps/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Adrián',
    maintainer_email='molinadrian11@gmail.com',
    description='Simulador web F1TENTH compatible con f1tenth_gym_ros',
    license='MIT',
    entry_points={
        'console_scripts': [
            'websim = f1tenth_websim.main:main',
        ],
    },
)
