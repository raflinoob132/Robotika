from setuptools import setup

package_name = 'my_python_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafli',
    maintainer_email='rafli@todo.todo',
    description='A simple ROS 2 Python package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = my_python_package.simple_node:main'
        ],
    },
)

