import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription

from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
        SetLaunchConfiguration)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	# declare launch arguments
	namespace_arg = DeclareLaunchArgument(
		'namespace',
		default_value = '',
		description = 'Top-level namespace')
	serial_port_arg = DeclareLaunchArgument(
		'serial_port',
		default_value = '/dev/ttyUSB0',
		description = 'Serial port')
	sonar_enabled_arg = DeclareLaunchArgument(
		'sonar_enabled',
		default_value = 'False',
		description = 'Whether sonars are enabled')
	publish_motors_state_arg = DeclareLaunchArgument(
		'publish_motors_state',
		default_value = 'True',
		description = 'Whether motors state is published')	

	def get_frame_prefix(context):
		prefix = context.launch_configurations['namespace']
		if (prefix != ''):
			prefix = prefix + '/'
		return prefix

	def get_sonar_frame_id(context):
		return [SetLaunchConfiguration('sonar_frame_id', get_frame_prefix(context) + 'sonar')]

	get_sonar_frame_id_fn = OpaqueFunction(function = get_sonar_frame_id)

	def get_base_frame_id(context):
		return [SetLaunchConfiguration('base_frame_id', get_frame_prefix(context) + 'base_link')]

	get_base_frame_id_fn = OpaqueFunction(function = get_base_frame_id)




    # specify the actions
	driver = Node(
		namespace = [LaunchConfiguration('namespace') ],		
		package =		'rosaria2',
		executable =	'rosaria2_node',
		name = 			'pioneer_p3dx_driver',
		parameters = [
			{
			'tf_prefix' :		[LaunchConfiguration('namespace') ],
			'odom_frame_id':	'odom',
			'base_frame_id':	'base_link',
			'bumper_frame_id':	'bumper',
			'sonar_frame_id':	'sonar',
			'serial_port':		[LaunchConfiguration('serial_port') ],
			'sonar_enabled':	[LaunchConfiguration('sonar_enabled') ],
			'publish_sonar':	[LaunchConfiguration('sonar_enabled') ],
			'publish_sonar_pointcloud2': [LaunchConfiguration('sonar_enabled') ],
			'publish_motors_state': [LaunchConfiguration('publish_motors_state') ]
			}
		]#,
		#arguments=['--ros-args', '--log-level', 'DEBUG'] # to increase the logger level from INFO to DEBUG
	)

	launch_tf_broadcaster = Node(
		namespace = [LaunchConfiguration('namespace') ],
		package='tf2_ros',
		executable='static_transform_publisher',
		name='static_tf_sonar_base_tf',
		arguments=['0', '0', '0.215', '0', '0', '0',
				[LaunchConfiguration('base_frame_id') ], [LaunchConfiguration('sonar_frame_id')]
			]
        )

    # create the launch description and populate
	ld = LaunchDescription()

	# arguments
	ld.add_action(namespace_arg)
	ld.add_action(serial_port_arg)
	ld.add_action(sonar_enabled_arg)
	ld.add_action(publish_motors_state_arg)

	# opaque functions
	ld.add_action(get_sonar_frame_id_fn)
	ld.add_action(get_base_frame_id_fn)	

	# actions
	ld.add_action(driver)
	ld.add_action(launch_tf_broadcaster)

	return ld
