from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ---- launch args (override at the CLI) ----
    mode           = LaunchConfiguration('mode')
    source_topic   = LaunchConfiguration('source_topic')
    out1_topic     = LaunchConfiguration('out1_topic')
    out2_topic     = LaunchConfiguration('out2_topic')
    ee1_frame      = LaunchConfiguration('ee1_frame')
    ee2_frame      = LaunchConfiguration('ee2_frame')
    world_frame    = LaunchConfiguration('world_frame')
    tf_timeout_ms  = LaunchConfiguration('tf_timeout_ms')
    neura_1_enabled = LaunchConfiguration('neura_1_enabled')
    neura_2_enabled = LaunchConfiguration('neura_2_enabled')
    surface_normal_x = LaunchConfiguration('surface_normal_x')
    surface_normal_y = LaunchConfiguration('surface_normal_y')
    surface_normal_z = LaunchConfiguration('surface_normal_z')
    master_slave_control = LaunchConfiguration('master_slave_control')

    return LaunchDescription([
        DeclareLaunchArgument('mode',          default_value='mirror',   description="fixed | mirror"),
        DeclareLaunchArgument('source_topic',  default_value='neura/cmd_vel'),
        DeclareLaunchArgument('out1_topic',    default_value='neura_1/cmd_vel'),
        DeclareLaunchArgument('out2_topic',    default_value='neura_2/cmd_vel'),
        DeclareLaunchArgument('ee1_frame',     default_value='neura_1/ee_tcp'),
        DeclareLaunchArgument('ee2_frame',     default_value='neura_2/ee_tcp'),
        DeclareLaunchArgument('world_frame',   default_value='world_link'),
        DeclareLaunchArgument('tf_timeout_ms', default_value='30'),
        DeclareLaunchArgument('neura_1_enabled', default_value='true', description='Enable remapping for neura_1'),
        DeclareLaunchArgument('neura_2_enabled', default_value='true', description='Enable remapping for neura_2'),
        DeclareLaunchArgument('surface_normal_x', default_value='0.0', description='X component of the surface normal in world frame (only for mode=fixed)'),
        DeclareLaunchArgument('surface_normal_y', default_value='0.0', description='Y component of the surface normal in world frame (only for mode=fixed)'),
        DeclareLaunchArgument('surface_normal_z', default_value='0.0', description='Z component of the surface normal in world frame (only for mode=fixed)'),
        DeclareLaunchArgument('master_slave_control', default_value='true', description='If true, neura_2 follows neura_1 displacements'),


        Node(
            package='velocity_remapper',
            executable='velocity_remapper_node',   # your compiled executable
            name='velocity_remapper',
            output='screen',
            parameters=[
                # inline params
                {
                    'mode': mode,
                    'source_topic': source_topic,
                    'out1_topic': out1_topic,
                    'out2_topic': out2_topic,
                    'ee1_frame': ee1_frame,
                    'ee2_frame': ee2_frame,
                    'world_frame': world_frame,
                    'tf_timeout_ms': tf_timeout_ms,
                    'neura_1_enabled': neura_1_enabled,
                    'neura_2_enabled': neura_2_enabled,
                    'surface_normal_x': surface_normal_x,
                    'surface_normal_y': surface_normal_y,
                    'surface_normal_z': surface_normal_z,
                    'master_slave_control': master_slave_control,
                },
            ],
        ),
    ])
