from setuptools import find_packages, setup

package_name = 'semantic_store'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Sarthak Mittal',
    maintainer_email='sarthak@polybee.co',
    description='Pure-Python spatial semantic memory: association/consolidation, '
                'retrieval, region clustering, and JSON persistence. No ROS dependencies.',
    license='Apache-2.0',
    # extras_require (not the deprecated tests_require) is how colcon detects the
    # pytest test runner; without it colcon falls back to `python -m unittest`.
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
