from setuptools import find_packages, setup

package_name = 'bob_eye_filter'

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
    maintainer='bobh',
    maintainer_email='bobh@zendee.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bob_eye_filter_exe = bob_eye_filter.bob_eye_filter_exe:main'
        ],
    },
)
