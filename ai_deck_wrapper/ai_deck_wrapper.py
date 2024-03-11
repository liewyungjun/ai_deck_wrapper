import rclpy
import sys
#sys.path.append('/opt/ros/humble/lib/python3/dist-packages')
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
#from image_transport import ImageTransport, CameraPublisher

import time
import socket,os,struct, time
import numpy as np
import cv2
from cv_bridge import CvBridge



#wrapper node to take images from crazyflie ip and publish to ros2 topic of type sensor_msgs/Image
class AI_Deck_Wrapper(Node):

    def __init__(self):
        super().__init__('ai_deck_wrapper')
        self.declare_parameter("period", 2.0) 
        self.declare_parameter("ip", "192.168.1.113") 
        self.declare_parameter("port", 5000) 
        self.declare_parameter("name", "cf13") 

        self.deck_ip = self.get_parameter('ip')
        self.deck_port = self.get_parameter('port')
        self.timer_period = self.get_parameter('period')
        self.name = self.get_parameter('name')

        self.publisher_ = self.create_publisher(Image, '/image_rect', 10)
        #self.publisher = self.image_transport.advertiseCamera('/image_rect', CompressedImage, 'jpeg')
        #self.publisher_info = self.create_publisher(CameraInfo, self.name.value + '/camera_info', 10)
        self.publisher_info = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.timer = self.create_timer(self.timer_period.value, self.timer_callback)

        self.get_logger().info("Connecting to socket on {}:{}...".format(self.deck_ip.value, self.deck_port.value))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.deck_ip.value, self.deck_port.value))
        self.get_logger().info("Socket connected")

        self.imgdata = None
        self.data_buffer = bytearray()
        self.count = 0
        self.start = time.time()
        self.bridge = CvBridge()


        self.cam_info = CameraInfo()
        self.cam_info.width = 324
        self.cam_info.height = 244
        self.cam_info.distortion_model = "plumb_bob"
        cam_mats = [180.049252, 0.000000, 169.270200, 0.000000, 180.528259, 160.598796, 0.000000, 0.000000, 1.000000]
        for i in range(len(cam_mats)):    
            self.cam_info.k[i] = cam_mats[i]
        
        distortion_mats = [-0.077304, -0.006814, 0.000832, 0.000674, 0.000000]
        for i in range(len(distortion_mats)):    
            self.cam_info.d.append(distortion_mats[i])
        
        projection_mats = [164.769073, 0.000000, 170.861434, 0.000000, 0.000000, 169.009918, 163.618060, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
        for i in range(len(projection_mats)):
            self.cam_info.p[i] = projection_mats[i]

        rectification_mats = [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
        for i in range(len(rectification_mats)):
            self.cam_info.r[i] = rectification_mats[i]


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
            self.count = self.count % 100
            #meanTimePerImage = (time.time()-self.start) / self.count
            #self.get_logger().info("{}".format(meanTimePerImage))
            #self.get_logger().info("{}".format(1/meanTimePerImage))

            self.imgdata = imgStream

        '''
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.data = self.imgdata
        #CompressedImage msg
        msg.format = 'jpeg'
        '''

        

        #Image msg
        # msg.height = height
        # msg.width = width
        # msg.encoding = format
        # msg.is_bigendian = False
        # msg.step = size
        self.cam_info.header.stamp = self.get_clock().now().to_msg()

        bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
        bayer_img.shape = (244, 324)
        cv_image = self.bridge.cv2_to_imgmsg(bayer_img, 'mono8')
        cv_image.header.stamp = self.cam_info.header.stamp
        self.publisher_.publish(cv_image)

        self.publisher_info.publish(self.cam_info)
        #self.publisher_.publish(msg)
        #self.publisher_.publish(msg, self.cam_info)
        #self.get_logger().info('Publishing Image: "%s"' % msg.header.stamp)
        #self.get_logger().info('Publishing Camera: "%s"' % self.cam_info.header.stamp)

        #display image
        if format == 0:
            bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
            bayer_img.shape = (244, 324)
            color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
            cv2.putText(bayer_img, "Count: {:.1f}".format(self.count), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(self.name.value, bayer_img)
            #cv2.imshow('Color', color_img)
            cv2.waitKey(1)
        else:
            with open("img.jpeg", "wb") as f:
                f.write(imgStream)
            nparr = np.frombuffer(imgStream, np.uint8)
            decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
            cv2.imshow('JPEG', decoded)
            cv2.waitKey(1)
        


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
    