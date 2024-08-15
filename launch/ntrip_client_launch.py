from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ntrip_node = Node(
        name="ntrip_client",
        namespace="ntrip",
        package="ntrip_client",
        executable="ntrip_node",
        respawn=True,
        respawn_delay=4,
        parameters=[
            {
                # Required parameters used to connect to the NTRIP server
                "host": "sapos.geonord-od.de",
                "port": 2101,
                "mountpoint": "RTCM4G",
                "username": "gast",
                "password": "gast",
                # Whether to connect with SSL. cert, key, and ca_cert options will only take effect if this is true
                "ssl": False,
                # If the NTRIP caster uses cert based authentication, you can specify the cert and keys to use with these options
                "cert": "None",
                "key": "None",
                # If the NTRIP caster uses self signed certs, or you need to use a different CA chain, specify the path to the file here
                "ca_cert": "None",
                # Not sure if this will be needed by other nodes, but this frame ID will be added to the RTCM messages published by this node
                "rtcm_frame_id": "odom",
                # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                "nmea_max_length": 82,
                "nmea_min_length": 3,
                # Will affect how many times the node will attempt to reconnect before exiting, and how long it will wait in between attempts when a reconnect occurs
                "reconnect_attempt_max": 10,
                "reconnect_attempt_wait_seconds": 5,
                # How many seconds is acceptable in between receiving RTCM. If RTCM is not received for this duration, the node will attempt to reconnect
                "rtcm_timeout_seconds": 4,
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(ntrip_node)
    return ld
