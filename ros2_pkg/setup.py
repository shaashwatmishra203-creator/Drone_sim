from setuptools import setup

package_name = 'drone_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone team',
    maintainer_email='drone-team@example.invalid',
    description='Flight evaluation and endurance logging for PX4 SITL.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'flight_logger = drone_eval.flight_logger:main',
            'mission_runner = drone_eval.mission_runner:main',
            'camera_bridge = drone_eval.camera_bridge:main',
            'camera_mapper = drone_eval.camera_mapper:main',
            'state_estimator = drone_eval.state_estimator:main',
            'truth_logger = drone_eval.truth_logger:main',
            'hall_mission = drone_eval.hall_mission:main',
            'vision_view = drone_eval.vision_view:main',
            'cloud_detector = drone_eval.cloud_detector:main',
            'cloud_link = drone_eval.cloud_link:main',
        ],
    },
)
