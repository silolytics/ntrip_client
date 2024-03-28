import os
import sys
import json
import importlib.util

import time
import datetime
from math import floor

import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import Header
from nmea_msgs.msg import Sentence, Gpgga

from ntrip_client.ntrip_client import NTRIPClient
from ntrip_client.nmea_parser import NMEA_DEFAULT_MAX_LENGTH, NMEA_DEFAULT_MIN_LENGTH

# Try to import a couple different types of RTCM messages
_MAVROS_MSGS_NAME = "mavros_msgs"
_RTCM_MSGS_NAME = "rtcm_msgs"
have_mavros_msgs = False
have_rtcm_msgs = False
if importlib.util.find_spec(_MAVROS_MSGS_NAME) is not None:
    have_mavros_msgs = True
    from mavros_msgs.msg import RTCM as mavros_msgs_RTCM
if importlib.util.find_spec(_RTCM_MSGS_NAME) is not None:
    have_rtcm_msgs = True
    from rtcm_msgs.msg import Message as rtcm_msgs_RTCM


class NTRIPRos(Node):
    def __init__(self, node_name, **kwargs):
        # Read a debug flag from the environment that should have been set by the launch file
        try:
            self._debug = json.loads(os.environ["NTRIP_CLIENT_DEBUG"].lower())
        except:
            self._debug = False

        # Init the node and declare params
        super().__init__(node_name, **kwargs)
        #self.declare_parameters(
        #    namespace='',
        #    parameters=[
        #        ('host', '127.0.0.1'),
        #        ('port', 2101),
        #        ('mountpoint', 'mount'),
        #        ('ntrip_version', 'None'),
        #        ('authenticate', False),
        #        ('username', ''),
        #        ('password', 'None'),
        #        ('ssl', False),
        #        ('cert', 'None'),
        #        ('key', 'None'),
        #        ('ca_cert', 'None'),
        #        ('rtcm_frame_id', 'odom'),
        #        ('nmea_max_length', NMEA_DEFAULT_MAX_LENGTH),
        #        ('nmea_min_length', NMEA_DEFAULT_MIN_LENGTH),
        #        ('rtcm_message_package', _RTCM_MSGS_NAME),
        #        ('reconnect_attempt_max', NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_MAX),
        #        ('reconnect_attempt_wait_seconds',
        #         NTRIPClient.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS),
        #        ('rtcm_timeout_seconds', NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS),
        #        ('base_file_path', '/home'),
        #    ]
        #)
#
        ## Read some mandatory config
        #host = self.get_parameter('host').value
        #port = self.get_parameter('port').value
        #mountpoint = self.get_parameter('mountpoint').value
#
#
        ## Optionally get the ntrip version from the launch file
        #ntrip_version = self.get_parameter('ntrip_version').value
        #if ntrip_version == 'None':
        #    ntrip_version = None
#
        ## Set the log level to debug if debug is true
        #if self._debug:
        #    rclpy.logging.set_logger_level(
        #        self.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG)
#
        ## If we were asked to authenticate, read the username and password
        #username = None
        #password = None
        #if self.get_parameter('authenticate').value:
        #    username = self.get_parameter('username').value
        #    password = self.get_parameter('password').value
        #    print(password)
        #    if not username:
        #        self.get_logger().error(
        #            'Requested to authenticate, but param "username" was not set')
        #        sys.exit(1)
        #    if not password:
        #        self.get_logger().error(
        #            'Requested to authenticate, but param "password" was not set')
        #        sys.exit(1)
        ## Last nmea time
        #self.last_nmea_time = self.get_clock().now()
        ## Read an optional Frame ID from the config
        #self._rtcm_frame_id = self.get_parameter('rtcm_frame_id').value
#
        ## Determine the type of RTCM message that will be published
        #rtcm_message_package = self.get_parameter('rtcm_message_package').value
        #if rtcm_message_package == _MAVROS_MSGS_NAME:
        #    if have_mavros_msgs:
        #        self._rtcm_message_type = mavros_msgs_RTCM
        #        self._create_rtcm_message = self._create_mavros_msgs_rtcm_message
        #    else:
        #        self.get_logger().fatal('The requested RTCM package {} is a valid option, but we were unable to import it. Please make sure you have it installed'.format(rtcm_message_package))
        #elif rtcm_message_package == _RTCM_MSGS_NAME:
        #    if have_rtcm_msgs:
        #        self._rtcm_message_type = rtcm_msgs_RTCM
        #        self._create_rtcm_message = self._create_rtcm_msgs_rtcm_message
        #    else:
        #        self.get_logger().fatal('The requested RTCM package {} is a valid option, but we were unable to import it. Please make sure you have it installed'.format(rtcm_message_package))
        #else:
        #    self.get_logger().fatal('The RTCM package {} is not a valid option. Please choose between the following packages {}'.format(
        #        rtcm_message_package, ','.join([_MAVROS_MSGS_NAME, _RTCM_MSGS_NAME])))
#
        ## Setup the RTCM publisher
        #self._rtcm_pub = self.create_publisher(
        #    self._rtcm_message_type, 'rtcm', 10)
#
        ## Initialize the client
        #self._client = NTRIPClient(
        #    host=host,
        #    port=port,
        #    mountpoint=mountpoint,
        #    ntrip_version=ntrip_version,
        #    username=username,
        #    password=password,
        #    logerr=self.get_logger().error,
        #    logwarn=self.get_logger().warning,
        #    loginfo=self.get_logger().info,
        #    logdebug=self.get_logger().debug
        #)
#
        ## Get some SSL parameters for the NTRIP client
        #self._client.ssl = self.get_parameter('ssl').value
        #self._client.cert = self.get_parameter('cert').value
        #self._client.key = self.get_parameter('key').value
        #self._client.ca_cert = self.get_parameter('ca_cert').value
        #if self._client.cert == 'None':
        #    self._client.cert = None
        #if self._client.key == 'None':
        #    self._client.key = None
        #if self._client.ca_cert == 'None':
        #    self._client.ca_cert = None
#
        ## Get some timeout parameters for the NTRIP client
        #self._client.nmea_parser.nmea_max_length = self.get_parameter(
        #    'nmea_max_length').value
        #self._client.nmea_parser.nmea_min_length = self.get_parameter(
        #    'nmea_min_length').value
        #self._client.reconnect_attempt_max = self.get_parameter(
        #    'reconnect_attempt_max').value
        #self._client.reconnect_attempt_wait_seconds = self.get_parameter(
        #    'reconnect_attempt_wait_seconds').value
        #self._client.rtcm_timeout_seconds = self.get_parameter(
        #    'rtcm_timeout_seconds').value
#
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
                print(password)
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

            # Determine the type of RTCM message that will be published
            rtcm_message_package = self.get_parameter('rtcm_message_package').value
            if rtcm_message_package == _MAVROS_MSGS_NAME:
                if have_mavros_msgs:
                    self._rtcm_message_type = mavros_msgs_RTCM
                    self._create_rtcm_message = self._create_mavros_msgs_rtcm_message
                else:
                    self.get_logger().fatal('The requested RTCM package {} is a valid option, but we were unable to import it. Please make sure you have it installed'.format(rtcm_message_package))
            elif rtcm_message_package == _RTCM_MSGS_NAME:
                if have_rtcm_msgs:
                    self._rtcm_message_type = rtcm_msgs_RTCM
                    self._create_rtcm_message = self._create_rtcm_msgs_rtcm_message
                else:
                    self.get_logger().fatal('The requested RTCM package {} is a valid option, but we were unable to import it. Please make sure you have it installed'.format(rtcm_message_package))
            else:
                self.get_logger().fatal('The RTCM package {} is not a valid option. Please choose between the following packages {}'.format(
                    rtcm_message_package, ','.join([_MAVROS_MSGS_NAME, _RTCM_MSGS_NAME])))

            # Setup the RTCM publisher
            self._rtcm_pub = self.create_publisher(
                self._rtcm_message_type, 'rtcm', 10)

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
        
    # Generate gga from msg
    def gen_gga(self, time_t, lat, lat_pole, lng, lng_pole, fix_quality, num_sats, hdop, alt_m, geoidal_sep_m, dgps_age_sec, dgps_ref_id):
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

    def run(self):
        # Connect the client
        if not self._client.connect():
            self.get_logger().error('Unable to connect to NTRIP server')
            return False
        # Setup our subscriber
        self._nmea_sub = self.create_subscription(
            Gpgga, '/gpgga', self.subscribe_nmea, 2)

        # Start the timer that will check for RTCM data
        self._rtcm_timer = self.create_timer(0.5, self.publish_rtcm)
        return True

    def stop(self):
        self.get_logger().info('Stopping RTCM publisher')
        if self._rtcm_timer:
            self._rtcm_timer.cancel()
            self._rtcm_timer.destroy()
        self.get_logger().info('Disconnecting NTRIP client')
        self._client.disconnect()
        self.get_logger().info('Shutting down node')
        self.destroy_node()

    def subscribe_nmea(self, nmea):
        # Just extract the NMEA from the message, and send it right to the server
        duration = rclpy.Duration(
            self.get_clock().now().to_sec() - self.last_nmea_time.to_sec())
        if (nmea.lat != 0.0 and nmea.num_sats > 4 and int(duration.to_sec()) > 1.0):
            time = self.get_clock().now()
            self.last_nmea_time = time
            time = time.to_sec()
            if (nmea.station_id == ''):
                nmea.station_id = 0
            nmea_sentence = self.gen_gga(time, nmea.lat, nmea.lat_dir, nmea.lon, nmea.lon_dir, nmea.gps_qual,
                                         nmea.num_sats, nmea.hdop, nmea.alt, nmea.undulation, int(nmea.diff_age), int(nmea.station_id))
            self._client.send_nmea(nmea_sentence)

    def publish_rtcm(self):
        for raw_rtcm in self._client.recv_rtcm():
            self._rtcm_pub.publish(self._create_rtcm_message(raw_rtcm))

    def _create_mavros_msgs_rtcm_message(self, rtcm):
        return mavros_msgs_RTCM(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=self._rtcm_frame_id
            ),
            data=rtcm
        )

    def _create_rtcm_msgs_rtcm_message(self, rtcm):
        return rtcm_msgs_RTCM(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=self._rtcm_frame_id
            ),
            message=rtcm
        )


if __name__ == '__main__':
    # Start the node
    rclpy.init()
    node = NTRIPRos()
    if not node.run():
        sys.exit(1)
    try:
        # Spin until we are shut down
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException as e:
        raise e
    finally:
        node.stop()

        # Shutdown the node and stop rclpy
        rclpy.shutdown()
