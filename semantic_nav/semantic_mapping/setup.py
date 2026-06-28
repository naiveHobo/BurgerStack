from setuptools import find_packages, setup

package_name = 'semantic_mapping'

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
    description='ROS shell over semantic_store: builds the spatial semantic map from '
                'DetectionArray (Phase 1, with batch VLM/embedding enrichment) and '
                'serves QuerySemanticMap from a saved map (Phase 2).',
    license='Apache-2.0',
    # extras_require (not the deprecated tests_require) is how colcon detects the
    # pytest test runner; without it colcon falls back to `python -m unittest`.
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mapping_node = semantic_mapping.mapping_node:main',
            'map_server_node = semantic_mapping.map_server_node:main',
            'map_finalizer_node = semantic_mapping.map_finalizer_node:main',
        ],
    },
)
