from setuptools import find_packages, setup

package_name = 'bumperbot_py_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albert',
    maintainer_email='nkhanh@humaxdigital.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_tf_kinematicss = bumperbot_py_examples.simple_tf_kinematics:main',
        ],
    },
)
