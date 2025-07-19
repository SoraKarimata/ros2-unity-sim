
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    # Get the launch directory

    tf_publisher = Node(
        package='tf_publisher',
        executable='tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        name='tf_publisher'
    )


    ld = LaunchDescription()


    ld.add_action(tf_publisher)


    return ld
