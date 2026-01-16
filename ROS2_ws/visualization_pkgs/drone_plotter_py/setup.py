from setuptools import setup

package_name = 'drone_plotter_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'matplotlib',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Newton Campbell',
    maintainer_email='newton.h.campbell@nasa.gov',
    description='ROS2 package that subscribes to /pub_pose and plots drone trajectory in 3D',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'drone_plotter = drone_plotter_py.drone_plotter:main'
        ],
    },
)
