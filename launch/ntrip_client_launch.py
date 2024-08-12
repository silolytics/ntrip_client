from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent
from launch.actions import SetEnvironmentVariable
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.actions import DeclareLaunchArgument
import os
import launch
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():

   return LaunchDescription([
      # Declare arguments with default values
      DeclareLaunchArgument(
          'namespace',             default_value=''),
      DeclareLaunchArgument(
          'node_name',             default_value='ntrip_client'),
      DeclareLaunchArgument('debug',                 default_value='false'),
      DeclareLaunchArgument(
          'host',                  default_value="sapos.geonord-od.de"),
      DeclareLaunchArgument('port',                  default_value='2101'),
      DeclareLaunchArgument(
          'mountpoint',            default_value='RTCM4G'),
      DeclareLaunchArgument('username',              default_value='gast'),
      #This is not used since it does not handle digits correct
      DeclareLaunchArgument('password',              default_value='gast'),
      DeclareLaunchArgument('ssl',                   default_value='False'),
      DeclareLaunchArgument('cert',                  default_value='None'),
      DeclareLaunchArgument('key',                   default_value='None'),
      DeclareLaunchArgument('ca_cert',               default_value='None'),

      SetEnvironmentVariable(name='NTRIP_NODE_NAME',
                           value=LaunchConfiguration('node_name')),
      
      Node(
            name=LaunchConfiguration('node_name'),
            namespace=LaunchConfiguration('namespace'),
            package='ntrip_client',
            executable='ntrip_node',
            respawn=True,
            respawn_delay=4,
            parameters=[
               {
                  # Required parameters used to connect to the NTRIP server
                  'host': LaunchConfiguration('host'),
                  'port': LaunchConfiguration('port'),
                  'mountpoint': LaunchConfiguration('mountpoint'),
                  'username': LaunchConfiguration('username'),
                  'password': LaunchConfiguration('password'),
                  # Whether to connect with SSL. cert, key, and ca_cert options will only take effect if this is true
                  'ssl': LaunchConfiguration('ssl'),
                  # If the NTRIP caster uses cert based authentication, you can specify the cert and keys to use with these options
                  'cert': LaunchConfiguration('cert'),
                  'key':  LaunchConfiguration('key'),
                  # If the NTRIP caster uses self signed certs, or you need to use a different CA chain, specify the path to the file here
                  'ca_cert': LaunchConfiguration('ca_cert'),
                  # Not sure if this will be looked at by other ndoes, but this frame ID will be added to the RTCM messages published by this node
                  'rtcm_frame_id': 'odom',
                  # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                  'nmea_max_length': 82,
                  'nmea_min_length': 3,
                  # Will affect how many times the node will attempt to reconnect before exiting, and how long it will wait in between attempts when a reconnect occurs
                  'reconnect_attempt_max': 10,
                  'reconnect_attempt_wait_seconds': 5,
                  # How many seconds is acceptable in between receiving RTCM. If RTCM is not received for this duration, the node will attempt to reconnect
                  'rtcm_timeout_seconds': 4
               }
            ],
        )
          
   ])
    
   