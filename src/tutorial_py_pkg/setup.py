from setuptools import find_packages, setup

package_name = 'tutorial_py_pkg'

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
    maintainer_email='albert@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "topic_publisher = tutorial_py_pkg.topic_publisher:main",
            "topic_subscription = tutorial_py_pkg.topic_subscription:main",
            "service_server = tutorial_py_pkg.service_server:main",
            "service_client = tutorial_py_pkg.service_client:main",
        ],
    },
)
