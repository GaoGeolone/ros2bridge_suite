# import rospy
import rclpy
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.util import bson

from twisted.internet.protocol import DatagramProtocol

class RosbridgeUdpFactory(DatagramProtocol):
    def startProtocol(self):
        self.socks = dict()
    def datagramReceived(self, message, source_addr):
        (host, port) = source_addr
        endpoint = host.__str__() + port.__str__()
        if endpoint in self.socks:
            self.socks[endpoint].datagramReceived(message)
        else:
            def writefunc(msg):
                self.transport.write(msg, (host, port))

            self.socks[endpoint] = RosbridgeUdpSocket(writefunc)
            self.socks[endpoint].startProtocol()
            self.socks[endpoint].datagramReceived(message)

class RosbridgeUdpSocket:
    client_id_seed = 0
    clients_connected = 0
    client_count_pub = None

    # The following parameters are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600                  # seconds
    # protocol.py:
    delay_between_messages = 0              # seconds
    max_message_size = None                 # bytes
    unregister_timeout = 10.0               # seconds
    ros_node = None

    def __init__(self,write):
        self.write = write

    def startProtocol(self):
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size,
            "unregister_timeout": cls.unregister_timeout
        }
        try:
            self.protocol = RosbridgeProtocol(cls.client_id_seed, cls.ros_node, parameters=parameters)
            self.protocol.outgoing = self.send_message
            cls.client_id_seed += 1
            cls.clients_connected += 1
            if cls.client_count_pub:
                cls.client_count_pub.publish(cls.clients_connected)
        except Exception as exc:
            # rospy.logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
            cls.ros_node.get_logger().error("Unable to accept incoming connection.  Reason: " + str(exc))
        # rospy.loginfo("Client connected.  %d clients total.", cls.clients_connected)
        cls.ros_node.get_logger().info("Client connected. "+ str(cls.clients_connected) + " clients total.")

    def datagramReceived(self, message):
        self.protocol.incoming(message)

    def stopProtocol(self):
        cls = self.__class__
        cls.clients_connected -= 1
        self.protocol.finish()
        if cls.client_count_pub:
            cls.client_count_pub.publish(cls.clients_connected)
        # rospy.loginfo("Client disconnected. %d clients total.", cls.clients_connected)
        logger.info.loginfo("Client disconnected. %d clients total.", cls.clients_connected)
    def send_message(self, message):
        binary = isinstance(message, bson.BSON)
        self.write(message)
    def check_origin(self, origin):
        return False
