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

from ntrip_client.ntrip_client_lib.ntrip_client import NTRIPClient
from ntrip_client.ntrip_client_lib.nmea_parser import NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH
import std_msgs.msg


class NTRIPRos(Node):

    def __init__(self, node_name, **kwargs):
        """Construct the node."""
        self._count: int = 0
        self._rtcm_pub: Optional[Publisher] = None
        self._rtcm_timer: Optional[Timer] = None
        self._diag_timer: Optional[Timer] = None
        super().__init__(node_name, **kwargs)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 2101)
        self.declare_parameter('mountpoint', 'mountpoint')
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('ssl', False)
        self.declare_parameter('cert', 'None')
        self.declare_parameter('key', 'None')
        self.declare_parameter('ca_cert', 'None')
        self.declare_parameter('rtcm_frame_id', 'odom')
        self.declare_parameter('nmea_max_length', NMEA_DEFAULT_MAX_LENGTH)
        self.declare_parameter('nmea_min_length', NMEA_DEFAULT_MIN_LENGTH)
        self.declare_parameter('reconnect_attempt_max',
                               NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_MAX)
        self.declare_parameter('reconnect_attempt_wait_seconds',
                               NTRIPClient.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS)
        self.declare_parameter('rtcm_timeout_seconds',
                               NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS)

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
        self.get_logger().fatal("############ Configure state")
        # Read some mandatory config
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        self.get_logger().error("Der node ist {}".format(host))
        mountpoint = self.get_parameter('mountpoint').value
        self.get_logger().fatal("host:{} , port:{} , mountpoint:{}".format(
        self.get_parameter('host').value,
        self.get_parameter('port').value,
        self.get_parameter('mountpoint').value))


        # We don't use the ntrip version since the node fails here
        # self.get_logger().warn("Value auth {}".format(self.get_parameter('authenticate').value))
        ntrip_version = None
        # Set the log level to debug if debug is true
        # If we were asked to authenticate, read the username and password
        username = None
        password = None

        username = self.get_parameter('username').value
        password = self.get_parameter('password').value

        # Last nmea time
        self.last_nmea_time = self.get_clock().now()
        # Read an optional Frame ID from the config
        self._rtcm_frame_id = self.get_parameter('rtcm_frame_id').value
        self._rtcm_message_type = rtcm_msgs_RTCM
        self._create_rtcm_message = self._create_rtcm_msgs_rtcm_message
        # Setup the RTCM publisher
        # Application setup starts
        self._rtcm_pub = self.create_lifecycle_publisher(
            self._rtcm_message_type, 'rtcm', 10)
        # Create timer for rtcm
        self._rtcm_timer = self.create_timer(1.0, self.publish_rtcm)
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
        self.get_logger().warn("Sind vor gga")
        timer_period = 2.0
        self._diag_timer = self.create_timer(
            timer_period, self.diagnostic_callback)
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
        self.destroy_timer(self._rtcm_timer)
        self.destroy_timer(self._diag_timer)
        self.destroy_publisher(self._rtcm_pub)
        self.destroy_publisher(self._diagnostic_pub)
        self.get_logger().info('Disconnecting NTRIP client')
        self._client.disconnect()

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

    # No lifecycle code
    def diagnostic_callback(self):
        self._diag_array.header.stamp = self.get_clock().now().to_msg()
        self._diagnostic_pub.publish(self._diag_array)

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
        self.undeclare_parameter('host', '127.0.0.1')
        self.undeclare_parameter('port', 2101)
        self.undeclare_parameter('mountpoint', 'mountpoint')
        self.undeclare_parameter('username', '')
        self.undeclare_parameter('password', '')
        self.undeclare_parameter('ssl', False)
        self.undeclare_parameter('cert', 'None')
        self.undeclare_parameter('key', 'None')
        self.undeclare_parameter('ca_cert', 'None')
        self.undeclare_parameter('rtcm_frame_id', 'odom')
        self.undeclare_parameter('nmea_max_length', NMEA_DEFAULT_MAX_LENGTH)
        self.undeclare_parameter('nmea_min_length', NMEA_DEFAULT_MIN_LENGTH)
        self.undeclare_parameter('reconnect_attempt_max',
                              NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_MAX)
        self.undeclare_parameter('reconnect_attempt_wait_seconds',
                              NTRIPClient.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS)
        self.undeclare_parameter('rtcm_timeout_seconds',
                               NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS)
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
    name = 'ntrip_client'
    if (os.getenv('NTRIP_NODE_NAME') is not None):
        name = str(os.getenv('NTRIP_NODE_NAME'))
    executor = rclpy.executors.SingleThreadedExecutor()
    ntrip_node = NTRIPRos(name)
    executor.add_node(ntrip_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        ntrip_node.destroy_node()


if __name__ == '__main__':
    main()
