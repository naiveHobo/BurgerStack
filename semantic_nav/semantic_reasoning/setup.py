from setuptools import find_packages, setup

package_name = 'semantic_reasoning'

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
    description='Phase-2 agentic reasoning for semantic_nav: a shared tool layer + '
                'reasoning loop turning natural-language commands into Nav2 goals, '
                'exposed via the ExecuteTask action (ollama) and an MCP server (Claude).',
    license='Apache-2.0',
    # extras_require (not the deprecated tests_require) is how colcon detects the
    # pytest test runner; without it colcon falls back to `python -m unittest`.
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'execute_task_node = semantic_reasoning.execute_task_node:main',
            'mcp_server = semantic_reasoning.mcp_server:main',
        ],
    },
)
