from setuptools import find_packages, setup

package_name = 'mini_porject'

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
    maintainer='gjlee',
    maintainer_email='gjlee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'medic_way = mini_project.medic_waypoint:main',
            'detack_depth = mini_project.detack_math:main',
            
            
        ],
    },
)
