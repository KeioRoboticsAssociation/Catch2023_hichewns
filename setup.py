from setuptools import setup
import os
from glob import glob
package_name = 'cal_rtheta'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.py')),
        (os.path.join('share',package_name,'urdf'),glob('urdf/*')),
        (os.path.join('share',package_name,'csv'),glob('csv/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moyuboo',
    maintainer_email='moyuboo1@keio.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joysub_cal = cal_rtheta.joysub_cal:main',
            'joint_publisher = cal_rtheta.joint_publisher:main',
            'joy_operation = cal_rtheta.joy_operation:main',
            'index = cal_rtheta.index:main',
            'shooting_index = cal_rtheta.shooting_index:main',
            'input = cal_rtheta.input:main',
            'convert = cal_rtheta.convert:main',
            'state = cal_rtheta.state:main',
            'state_index = cal_rtheta.state_index:main',
            'cmd_state = cal_rtheta.cmd_state:main',
            'xy_to_rtheta_index = cal_rtheta.xy_to_rtheta_index:main',
            'xy_to_rtheta_index_copy = cal_rtheta.xy_to_rtheta_index_copy:main',
            'xy_to_rtheta = cal_rtheta.xy_to_rtheta:main'
        ],
    },
)
