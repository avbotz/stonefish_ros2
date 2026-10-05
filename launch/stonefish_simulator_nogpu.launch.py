from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    simulation_data = LaunchConfiguration('simulation_data')
    scenario_desc = LaunchConfiguration('scenario_desc')
    simulation_rate = LaunchConfiguration('simulation_rate')
    publish_clock = LaunchConfiguration('publish_clock')
    realtime_factor = LaunchConfiguration('realtime_factor')
    
    simulation_data_arg = DeclareLaunchArgument(
        'simulation_data',
        default_value = ''
    )

    scenario_desc_arg = DeclareLaunchArgument(
        'scenario_desc',
        default_value = ''
    )

    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate',
        default_value = '100.0'
    )

    publish_clock_arg = DeclareLaunchArgument(
        'publish_clock',
        default_value = 'false'
    )

    realtime_factor_arg = DeclareLaunchArgument(
        'realtime_factor',
        default_value = '1.0'
    )

    stonefish_simulator_nogpu_node = Node(
            package='stonefish_ros2',
            executable='stonefish_simulator_nogpu',
            namespace='stonefish_ros2',
            name='stonefish_simulator_nogpu',
            arguments=[simulation_data, scenario_desc, simulation_rate],
            parameters=[{
                'publish_clock': ParameterValue(publish_clock, value_type=bool),
                'realtime_factor': ParameterValue(realtime_factor, value_type=float)
            }],
            output='screen',
    )

    return LaunchDescription([
        simulation_data_arg,
        scenario_desc_arg,
        simulation_rate_arg,
        publish_clock_arg,
        realtime_factor_arg,
        stonefish_simulator_nogpu_node
    ])