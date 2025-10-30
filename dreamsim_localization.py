#!/usr/bin/env python
import csv
import numpy as np
import pandas as pd
import rclpy
import sys
import torch.nn.functional as F

from cv_bridge import CvBridge
from dreamsim import dreamsim
from PIL import Image as PILImage
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from tqdm import tqdm


def pose_to_transformation_matrix(x, y, yaw):
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    T = np.array([
        [cos_yaw, -sin_yaw, x],
        [sin_yaw,  cos_yaw, y],
        [0,        0,       1]
    ])

    return T
    
class Localizer(Node):
    def __init__(self):
        super().__init__("localizer_node")

        # read image-pose map
        self.image_pose_df = pd.read_csv(sys.argv[1] + "/map.csv",
                                         header=None,
                                         names=['index', 'x', 'y', 'a', 'filename']
                                        )
        self.images = []
        for filename in self.image_pose_df['filename']:
            self.images.append(PILImage.open(sys.argv[1] + "/" + filename))

        # init model
        self.device = "cuda"
        self.model, self.preprocess = dreamsim(pretrained=True, device=self.device)

        # compute query embeddings
        self.query_embeddings = []
        for image in self.images:
            self.query_embeddings.append(self.model.embed(self.preprocess(image).to(self.device)))
        print("Done loading model and preprocessing images.")
        self.last_rgb = None
        self.br = CvBridge()
                
        self.img_sub = self.create_subscription(Image, "/cer/realsense_repeater/color_image", self.image_callback, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "pose", 10)
        
    def image_callback(self, msg):
        self.last_rgb = self.br.imgmsg_to_cv2(msg).copy()[:,:,::-1]

    def amcl_callback(self, msg):
        print("received amcl pose")
        if msg.pose.covariance.max() > 0.15 and self.last_rgb is not None:
            print("covariance too high, re-localizing...")

            target_emb = self.model.embed(self.preprocess(PILImage.fromarray(self.last_rgb)).to(self.device))

            similarities = []

            for i, query_emb in tqdm(enumerate(self.query_embeddings), total=len(self.query_embeddings)):
                similarity = F.cosine_similarity(query_emb, target_emb, dim=-1).item()
                similarities.append(similarity)

            est = np.argmax(similarities)

            x = self.image_pose_df['x'][est]
            y = self.image_pose_df['y'][est]
            a = self.image_pose_df['a'][est]
             
            print("localized at: ", x, y, a, "matched to", self.image_pose_df['filename'][est], "with similarity", similarities[est])

            # TODO: send pose to amcl
            if similarities[est] > 0.7:
                print("sending pose to amcl")
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = "map"
                pose_msg.header.stamp = msg.header.stamp
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = 0.0
                self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Localizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
