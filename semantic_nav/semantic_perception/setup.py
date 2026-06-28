from setuptools import find_packages, setup

package_name = 'semantic_perception'

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
    description='Phase-1 RGB-D perception for semantic_nav: detect -> deproject -> '
                'transform to map -> publish DetectionArray. Pluggable, mockable detectors.',
    license='Apache-2.0',
    # extras_require (not the deprecated tests_require) is how colcon detects the
    # pytest test runner; without it colcon falls back to `python -m unittest`.
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'perception_node = semantic_perception.perception_node:main',
        ],
    },
)
