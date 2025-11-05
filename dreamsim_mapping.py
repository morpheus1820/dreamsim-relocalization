#!/usr/bin/env python
import csv
import cv2
import message_filters
import numpy as np
import rclpy
import sys
import time

from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Mapper(Node):
    def __init__(self):
        super().__init__("mapper_node")

        self.img_count = 0
        self.last_time = time.time()
        self.csvfile = open('map.csv', 'w', newline='')
        self.spamwriter = csv.writer(self.csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)

        self.br = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
                
        self.img_sub = self.create_subscription(Image, "/cer/realsense_repeater/color_image", self.image_callback, 10)

        self.get_logger().info("Mapper node started.")
        
        # self.img_sub = message_filters.Subscriber(self, Image, "/cer/realsense_repeater/color_image")
        # self.amcl_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, "amcl_pose")
        # self.tss = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.amcl_sub], 1, slop=1.0)
        # self.tss.registerCallback(self.combined_callback)

    def image_callback(self, msg):
        print("callback!")
        current_time = time.time()
        if current_time - self.last_time < 1.0:
            print("too early!")
            return
        self.last_time = current_time
        
        ############# RT #################
        T = None
        try:
            T = self.tf_buffer.lookup_transform(
                    "map",
                    "mobile_base_body_link",
                    msg.header.stamp  #rclpy.time.Time()   #depth_msg.header.stamp 
                )
        except TransformException:
            print("Could not transform!")
            return

        q = T.transform.rotation
        x = str(T.transform.translation.x)
        y = str(T.transform.translation.y)
        a = str(np.arctan2(2.0 * (q.z * q.w + q.x * q.y) , - 1.0 + 2.0 * (q.w * q.w + q.x * q.x)))
        
        rgb = self.br.imgmsg_to_cv2(msg).copy()[:,:,::-1]
        filename = "map_image_" + str(self.img_count).zfill(5) + ".jpg"
        cv2.imwrite(filename, rgb)
        self.spamwriter.writerow([str(self.img_count), x, y, a, filename])
        self.img_count += 1
                        
    # def combined_callback(self, img_msg, amcl_msg):
        # print("callback!")
        # rgb = self.br.imgmsg_to_cv2(img_msg).copy()[:,:,::-1]
        # pose = amcl_msg.pose.pose
# 
        # filename = "map_image_" + str(self.img_count).zfill(5) + ".jpg"
        # cv2.imwrite(filename, rgb)
        # x = str(pose.position.x)
        # y = str(pose.position.y)
        # q = pose.orientation
        # a = str(np.atan2(2.0 * (q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z))
        # 
        # self.spamwriter.write([str(self.img_count), x, y, a, filename])
        # self.img_count += 1
        
def main(args=None):
    rclpy.init(args=args)
    node = Mapper()
    rclpy.spin(node)
    node.csvfile.close()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
