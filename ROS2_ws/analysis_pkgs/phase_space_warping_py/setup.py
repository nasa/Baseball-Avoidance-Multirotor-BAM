from setuptools import setup

package_name = 'phase_space_warping_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'scipy',
                      'scikit-learn'],
    zip_safe=True,
    maintainer='Newton Campbell',
    maintainer_email='newton.h.campbell@nasa.gov',
    description='Phase Space Warping (PSW) analysis for eVTOL flight dynamics',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'psw_node = phase_space_warping_py.phase_space_warping:main',
        ],
    },
)
