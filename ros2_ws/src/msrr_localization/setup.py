from setuptools import find_packages, setup

package_name = 'msrr_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='jaipipatt',
    maintainer_email='j.jaijai108@gmail.com',
    description='AprilTag localization tools for MSRR modules (ESP32-CAM, detection, Kalman tracking)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tag_live_view = msrr_localization.tag_live_view:main',
        ],
    },
)
