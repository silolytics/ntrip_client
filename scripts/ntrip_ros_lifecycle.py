from typing import Optional

import os
import sys

import time
import datetime
from math import floor
import rclpy


# Node, State and Publisher are aliases for LifecycleNode, LifecycleState and LifecyclePublisher
# respectively.
# In case of ambiguity, the more explicit names can be imported.

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.qos import qos_profile_system_default
from rclpy.timer import Timer
from rtcm_msgs.msg import Message as rtcm_msgs_RTCM

from std_msgs.msg import Header
from nmea_msgs.msg import Sentence, Gpgga

from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.nmea_parser import NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH
import std_msgs.msg


class NTRIPRos(Node):

    def __init__(self, node_name, **kwargs):
        """Construct the node."""
        self._count: int = 0
        self._rtcm_pub: Optional[Publisher] = None
        self._rtcm_timer: Optional[Timer] = None
        self._diag_timer: Optional[Timer] = None
        super().__init__(node_name, **kwargs)

    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.
    
        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.
    
        :return: The state machine either invokes a transition to the "inactive" state or stays
            in "unconfigured" depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "inactive".
            TransitionCallbackReturn.FAILURE transitions to "unconfigured".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', '127.0.0.1'),
                ('port', 2101),
                ('mountpoint', 'mount'),
                ('ntrip_version', 'None'),
                ('authenticate', False),
                ('username', ''),
                ('password', 'None'),
                ('ssl', False),
                ('cert', 'None'),
                ('key', 'None'),
                ('ca_cert', 'None'),
                ('rtcm_frame_id', 'odom'),
                ('nmea_max_length', NMEA_DEFAULT_MAX_LENGTH),
                ('nmea_min_length', NMEA_DEFAULT_MIN_LENGTH),
                ('rtcm_message_package', _RTCM_MSGS_NAME),
                ('reconnect_attempt_max', NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_MAX),
                ('reconnect_attempt_wait_seconds',
                 NTRIPClient.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS),
                ('rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS),
                ('base_file_path', '/home'),
            ]
        )
    
        # Read some mandatory config
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        mountpoint = self.get_parameter('mountpoint').value
        # Check if environment variables are set. This overide the default values
        if (os.getenv('HOST') is not None):
            host = os.getenv('HOST')
    
            host_new = rclpy.parameter.Parameter(
                'host',
                rclpy.Parameter.Type.STRING,
                host
            )
            self.set_parameters([host_new])
        if (os.getenv('MOUNTPOINT') is not None):
            mountpoint = os.getenv('MOUNTPOINT')
    
            mountpoint_new = rclpy.parameter.Parameter(
                'mountpoint',
                rclpy.Parameter.Type.STRING,
                mountpoint
            )
            self.set_parameters([mountpoint_new])
        if (os.getenv('PORT') is not None):
            port = os.getenv('PORT')
    
            port_new = rclpy.parameter.Parameter(
                'port',
                rclpy.Parameter.Type.STRING,
                port
            )
            self.set_parameters([port_new])
        # Optionally get the ntrip version from the launch file
        ntrip_version = self.get_parameter('ntrip_version').value
        if ntrip_version == 'None':
            ntrip_version = None
        # Set the log level to debug if debug is true
        if self._debug:
            rclpy.logging.set_logger_level(
                self.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG)
        # If we were asked to authenticate, read the username and password
        username = None
        password = None
    
        if self.get_parameter('authenticate').value:
            username = self.get_parameter('username').value
            password = self.get_parameter('password').value
            if (os.getenv('USERNAME') is not None):
                username = os.getenv('USERNAME')
    
                username_new = rclpy.parameter.Parameter(
                    'username',
                    rclpy.Parameter.Type.STRING,
                    username
                )
                self.set_parameters([username_new])
            
            if (os.getenv('PASSWORD') is not None):
                password = str(os.getenv('PASSWORD'))
    
                password_new = rclpy.parameter.Parameter(
                    'password',
                    rclpy.Parameter.Type.STRING,
                    password
                )
                self.set_parameters([password_new])
            
            if not username:
                self.get_logger().error(
                    'Requested to authenticate, but param "username" was not set')
                sys.exit(1)
            if not password:
                self.get_logger().error(
                    'Requested to authenticate, but param "password" was not set')
                sys.exit(1)
        # Last nmea time
        self.last_nmea_time = self.get_clock().now()
        # Read an optional Frame ID from the config
        self._rtcm_frame_id = self.get_parameter('rtcm_frame_id').value
        self._rtcm_message_type = rtcm_msgs_RTCM
        self._create_rtcm_message = self._create_rtcm_msgs_rtcm_message
        # Setup the RTCM publisher
        ## Application setup starts
        self._rtcm_pub = self.create_lifecycle_publisher(
            self._rtcm_message_type, 'rtcm', 10)
        # Create timer for rtcm
        self._rtcm_timer = self.create_timer(0.5, self.publish_rtcm)
        # Initialize the client
        self._client = NTRIPClient(
            host=host,
            port=port,
            mountpoint=mountpoint,
            ntrip_version=ntrip_version,
            username=username,
            password=password,
            logerr=self.get_logger().error,
            logwarn=self.get_logger().warning,
            loginfo=self.get_logger().info,
            logdebug=self.get_logger().debug
        )
        # Get some SSL parameters for the NTRIP client
        self._client.ssl = self.get_parameter('ssl').value
        self._client.cert = self.get_parameter('cert').value
        self._client.key = self.get_parameter('key').value
        self._client.ca_cert = self.get_parameter('ca_cert').value
        if self._client.cert == 'None':
            self._client.cert = None
        if self._client.key == 'None':
            self._client.key = None
        if self._client.ca_cert == 'None':
            self._client.ca_cert = None
        # Get some timeout parameters for the NTRIP client
        self._client.nmea_parser.nmea_max_length = self.get_parameter(
            'nmea_max_length').value
        self._client.nmea_parser.nmea_min_length = self.get_parameter(
            'nmea_min_length').value
        self._client.reconnect_attempt_max = self.get_parameter(
            'reconnect_attempt_max').value
        self._client.reconnect_attempt_wait_seconds = self.get_parameter(
            'reconnect_attempt_wait_seconds').value
        self._client.rtcm_timeout_seconds = self.get_parameter(
            'rtcm_timeout_seconds').value
    
        
        self._ntrip_connected = False
        self._diag_array = DiagnosticArray()
        self._diag_array.status = [
            # Data available and ok
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/ntrip/connected', message='Ntrip connected: OK', hardware_id='20'),
                             DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/ntrip/rtcm', message='Rtcm data : OK', hardware_id='20')]
        
        if not self._client.connect():
            self.get_logger().error('Unable to connect to NTRIP server')
            self._diag_array.status[0].level = DiagnosticStatus.WARN
            self._diag_array.status[0].message = 'Ntrip not connected'
        else:
            self._diag_array.status[0].level = DiagnosticStatus.OK
            self._diag_array.status[0].message = 'Ntrip connected'
        
        # Create subsriber for gga
        self._gga_sub = self.create_subscription(
            Gpgga, '/gpgga', self.subscribe_gga, 2)
        self._diagnostic_pub = self.create_lifecycle_publisher(DiagnosticArray,
                                                    '/diagnostics',
                                                    qos_profile_system_default)
        
        timer_period = 2.0
        self._diag_timer = self.create_timer(timer_period, self.diagnostic_callback)
        self.get_logger().info('on_configure() is called.')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Differently to rclcpp, a lifecycle publisher transitions automatically between the
        # inactive and enabled state and viceversa.
        # For that reason, we only need to write an on_configure() and on_cleanup() callbacks,
        # and we don't need to write on_activate()/on_deactivate() callbacks.
    
        # Log, only for demo purposes
        self.get_logger().info('on_activate() is called.')
    
        # The default LifecycleNode callback is the one transitioning
        # LifecyclePublisher entities from inactive to enabled.
        # If you override on_activate(), don't forget to call the parent class method as well!!
        return super().on_activate(state)
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info('Ntrip on_deactivate() is called.')
        # Same reasong here that for on_activate().
        # These are the two only cases where you need to call the parent method.
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node, after a cleaning-up transition is requested.
    
        on_cleanup callback is being called when the lifecycle node
        enters the "cleaning up" state.
    
        :return: The state machine either invokes a transition to the "unconfigured" state or stays
            in "inactive" depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
            TransitionCallbackReturn.FAILURE transitions to "inactive".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.stop()
    
        self.get_logger().info('Ntrip on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested.
    
        on_shutdown callback is being called when the lifecycle node
        enters the "shutting down" state.
    
        :return: The state machine either invokes a transition to the "finalized" state or stays
            in the current state depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
            TransitionCallbackReturn.FAILURE transitions to "inactive".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.stop()
    
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS
    
    ## No lifecycle code
    def publish_rtcm(self):
        for raw_rtcm in self._client.recv_rtcm():
            self._rtcm_pub.publish(self._create_rtcm_message(raw_rtcm))

    def _create_rtcm_msgs_rtcm_message(self, rtcm):
        """Wrapper to create rtcm message from rtcm data retreived 
            from client

        Args:
            rtcm (_type_): The rtcm data

        Returns:
            rtcm_message : rtcm_msgs_RTCM 
        """
        return rtcm_msgs_RTCM(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=self._rtcm_frame_id
            ),
        message=rtcm
        )
    
    def subscribe_gga(self, gga):
        """Subscribe to gga and send nmea to ntrip_client

        Args:
            nmea (_type_): _description_
        """
        duration = rclpy.Duration(
            self.get_clock().now().to_sec() - self.last_nmea_time.to_sec())
        if (gga.lat != 0.0 and gga.num_sats > 4 and int(duration.to_sec()) > 1.0):
            time = self.get_clock().now()
            self.last_nmea_time = time
            time = time.to_sec()
            if (gga.station_id == ''):
                gga.station_id = 0
            nmea_sentence = self.gen_gga(time, gga.lat, gga.lat_dir, gga.lon, gga.lon_dir, gga.gps_qual,
                                         gga.num_sats, gga.hdop, gga.alt, gga.undulation, int(gga.diff_age), int(gga.station_id))
            self._client.send_nmea(nmea_sentence)
    
    def stop(self):
        """Stop the Ntrip service
        """
        self.get_logger().info('Stopping RTCM publisher')
        
        self.destroy_timer(self._rtcm_timer)
        self.destroy_timer(self._diag_timer)
        self.destroy_publisher(self._rtcm_pub)
        self.destroy_publisher(self._diagnostic_pub)
        self.get_logger().info('Disconnecting NTRIP client')
        self._client.disconnect()
        self.get_logger().info('Shutting down node')

    def gen_gga(self, time_t, lat, lat_pole, lng, lng_pole, fix_quality, num_sats, hdop, alt_m, geoidal_sep_m, dgps_age_sec, dgps_ref_id):
        """Create GGA suitable for ntrip nmea msgs

        Args:
            time_t (_type_): time 
            lat (_type_): latitude
            lat_pole (_type_): lat pol
            lng (_type_): longitude
            lng_pole (_type_): _description_
            fix_quality (_type_): _description_
            num_sats (_type_): number of satellites in view
            hdop (_type_): _description_
            alt_m (_type_): _description_
            geoidal_sep_m (_type_): _description_
            dgps_age_sec (_type_): _description_
            dgps_ref_id (_type_): _description_

        Returns:
            _type_: _description_
        """
        dt_object = datetime.datetime.fromtimestamp(time_t)
        hhmmssss = '%02d%02d%02d%s' % (
            dt_object.hour, dt_object.minute, dt_object.second, '.%02d' if 0 != 0 else '')
        lat_abs = abs(lat)
        lat_deg = lat_abs
        lat_min = (lat_abs - floor(lat_deg)) * 60
        lat_sec = round((lat_min - floor(lat_min)) * 1000)
        lat_pole_prime = ('S' if lat_pole ==
                          'N' else 'N') if lat < 0 else lat_pole
        lat_format = '%02d%02d.%03d' % (lat_deg, lat_min, lat_sec)

        lng_abs = abs(lng)
        lng_deg = lng_abs
        lng_min = (lng_abs - floor(lng_deg)) * 60
        lng_sec = round((lng_min - floor(lng_min)) * 1000)
        lng_pole_prime = ('W' if lng_pole ==
                          'E' else 'E') if lng < 0 else lng_pole
        lng_format = '%03d%02d.%03d' % (lng_deg, lng_min, lng_sec)

        dgps_format = '%s,%s' % ('%.1f' % dgps_age_sec if dgps_age_sec is not None else '',
                                 '%04d' % dgps_ref_id if dgps_ref_id is not None else '')

        str = 'GPGGA,%s,%s,%s,%s,%s,%d,%02d,%.1f,%.1f,M,%.1f,M,%s' % (
            hhmmssss, lat_format, lat_pole_prime, lng_format, lng_pole_prime, fix_quality, num_sats, hdop, alt_m, geoidal_sep_m, dgps_format)
        crc = 0
        for c in str:
            crc = crc ^ ord(c)
        crc = crc & 0xFF

        return '$%s*%0.2X' % (str, crc)
    
    # A    lifecycle node has the same node API
    # as    a regular node. This means we can spawn a
    # no   de, give it a name and add it to the executor.

def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    ntrip_node = NTRIPRos('ntrip_client')
    executor.add_node(ntrip_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        ntrip_node.destroy_node()


if __name__ == '__main__':
    main()
