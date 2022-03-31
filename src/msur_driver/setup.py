from setuptools import setup

package_name = 'msur_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Photon94',
    maintainer_email='299792458.photon.94@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver = msur_driver.receiver:main',
            'update_pid_service = msur_driver.update_pid_service:main',
            'update_pid_client = msur_driver.update_pid_client:main',
            'write_pid_service = msur_driver.write_pid_service:main',
            'write_pid_client = msur_driver.write_pid_client:main',
            'reboot_device_service = msur_driver.reboot_device_service:main',
            'reboot_device_client = msur_driver.reboot_device_client:main',
        ],
    },
)
