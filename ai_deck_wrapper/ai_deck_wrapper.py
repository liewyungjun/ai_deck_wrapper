import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import time
import socket,os,struct, time


#wrapper node to take images from crazyflie ip and publish to ros2 topic of type sensor_msgs/Image
class AI_Deck_Wrapper(Node):

    def __init__(self):
        super().__init__('ai_deck_wrapper')
        self.declare_parameter("period", 0.5) 
        self.declare_parameter("ip", "192.168.4.1") 
        self.declare_parameter("port", "5000") 

        self.deck_ip = self.get_parameter('ip')
        self.deck_port = self.get_parameter('port')
        self.timer_period = self.get_parameter('period')

        self.publisher_ = self.create_publisher(CompressedImage, '/image_rect/compressed', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Connecting to socket on {}:{}...".format(self.deck_ip, self.deck_port))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.deck_ip, self.deck_port))
        self.get_logger().info("Socket connected")

        self.imgdata = None
        self.data_buffer = bytearray()
        self.count = 0
        self.start = time.time()

    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size-len(data)))
        return data

    def timer_callback(self):
        #maybe this all can be a rcvImage function
        # First get the info
        packetInfoRaw = self.rx_bytes(4)
        #self.get_logger().info(packetInfoRaw)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
        #self.get_logger().info("Length is {}".format(length))
        #self.get_logger().info("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
        #self.get_logger().info("Function is 0x{:02X}".format(function))

        imgHeader = self.rx_bytes(length - 2)
        #self.get_logger().info(imgHeader)
        #self.get_logger().info("Length of data is {}".format(len(imgHeader)))
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        if magic == 0xBC:
            #self.get_logger().info("Magic is good")
            #self.get_logger().info("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
            #self.get_logger().info("Image format is {}".format(format))
            #self.get_logger().info("Image size is {} bytes".format(size))

            # Now we start rx the image, this will be split up in packages of some size
            imgStream = bytearray()

            while len(imgStream) < size:
                packetInfoRaw = self.rx_bytes(4)
                [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                #self.get_logger().info("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = self.rx_bytes(length - 2)
                imgStream.extend(chunk)
            
            
            self.count = self.count + 1
            meanTimePerImage = (time.time()-self.start) / self.count
            self.get_logger().info("{}".format(meanTimePerImage))
            self.get_logger().info("{}".format(1/meanTimePerImage))

            self.imgdata = imgStream

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.data = self.imgdata
        #CompressedImage msg
        msg.format = 'jpeg'

        #Image msg
        # msg.height = height
        # msg.width = width
        # msg.encoding = format
        # msg.is_bigendian = False
        # msg.step = size

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Image: "%s"' % msg.header.stamp)
        


def main(args=None):
    rclpy.init(args=args)

    ai_deck_wrapper = AI_Deck_Wrapper()

    rclpy.spin(ai_deck_wrapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ai_deck_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    