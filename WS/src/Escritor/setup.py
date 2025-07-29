from setuptools import find_packages, setup

package_name = 'Escritor'

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
    maintainer='ivan',
    maintainer_email='ivanmorladag@gmail.com',
    description='Escritura turtlesim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'NodoEscritor = Escritor.NodoEscritor:main',
        ],
    },
)
