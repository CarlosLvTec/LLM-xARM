from setuptools import find_packages, setup

package_name = 'llm_chat'

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
    maintainer='kencove',
    maintainer_email='kencove@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'logfile_publisher = llm_chat.logfile_publisher:main',
        'llm_listener = llm_chat.llm_listener:main',
        ],
    },
)
