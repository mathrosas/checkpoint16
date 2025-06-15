from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='True',
        description='Use simulation (Gazebo) clock if true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    eight_trajectory =  Node(
            package='eight_trajectory',
            executable='eight_trajectory',
            name='eight_trajectory',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
        ]
    )
    
    kinematic_model_node = Node(
        name="kinematic_model",
        package="kinematic_model",
        executable="kinematic_model",
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        eight_trajectory,
        kinematic_model_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=eight_trajectory,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    ])