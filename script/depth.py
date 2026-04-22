#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class BallFollower:
    def __init__(self):
        self.bridge = CvBridge()

        self.cmd_pub   = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1) #output
        self.image_pub = rospy.Publisher('/image_converter/output_video', Image, queue_size=1) #output

        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback) #input

        self.ball_col    = None
        self.ball_row    = None
        self.ball_size   = None
        self.front_dist  = None

        self.target_dist = 1.0
        self.tolerance   = 0.05

        rospy.loginfo("Ball Follower (Depth Only) Started!")

    def depth_callback(self, msg):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            depth = depth_raw.astype(np.float32) / 1000.0
        except: #t Astra firmware versions publish different formats: try 16UC1 first, if fails try 32FC1
            try:
                depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            except Exception as e:
                rospy.logerr("Depth error: %s", str(e))
                return

        # ignore object closer than 0.2m and the background farther than 4m
        valid_mask = ((depth > 0.2) & (depth < 4.0)).astype(np.uint8) * 255

        # Finds all blob outlines in the binary mask using cv2.findContours built-in function from the OpenCV library
        contours, _ = cv2.findContours(
            valid_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #stretches all values to fit 0–255 range to be a viewable grayscale image. Near objects = bright, far objects = dark.
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        #Converts float to 8-bit integer (required for display and drawing).
        depth_vis = np.uint8(depth_vis)
        #green circles for target output visualization.
        vis_image = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)
        #best ball candidate found
        best = None
        # highest circularity score
        best_circularity = 0

        for c in contours: #every blob found in the mask.
            area = cv2.contourArea(c) # pixel area of the blob
            if area < 500: # too small to be a ball
                continue

            ((x, y), radius) = cv2.minEnclosingCircle(c) #Find the min circle that fits around the blob. Returns center (x, y) & radius in pixels.
            if radius < 15: #Skip if circle; too small
                continue

            circularity = area / (3.14159 * radius * radius) if radius > 0 else 0
            #Perfect circle = 1.0 ; Square ≈ 0.6; Random shape < 0.5

            if circularity > 0.6 and circularity > best_circularity:
                best_circularity = circularity
                best = (x, y, radius, c)

        if best is not None:
            x, y, radius, c = best
            self.ball_col = int(x)
            self.ball_row = int(y)
            self.ball_size = radius

            # a small region(4*4) for better reliable distance reading; min max to prevent out of bound
            y1 = max(0, int(y) - 2) # 2 pixels above center, min 0
            y2 = min(depth.shape[0], int(y) + 2) # 2 pixels below center, max image height
            x1 = max(0, int(x) - 2) # 2 pixels left of center, min 0
            x2 = min(depth.shape[1], int(x) + 2) # 2 pixels right of center, max image width
            region = depth[y1:y2, x1:x2]
            valid = region[(region > 0.2) & (region < 4.0) & np.isfinite(region)] #remove NaN and infinity values
            if len(valid) > 0:
                self.front_dist = float(np.mean(valid))
            else:
                self.front_dist = None

            #draws a green circle on the visualization image
            cv2.circle(vis_image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
        else:
            self.ball_col   = None
            self.ball_row   = None
            self.ball_size  = None
            self.front_dist = None

        # text on the visual output
        dist_text = "{:.2f}m".format(self.front_dist) if self.front_dist else "N/A"
        size_text = "{:.1f}px".format(self.ball_size) if self.ball_size else "N/A"

        cv2.putText(vis_image, "dist: " + dist_text,
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(vis_image, "size: " + size_text,
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(vis_image, "target: 1.0m",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(vis_image, "MODE: DEPTH ONLY",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        #Converts OpenCV image → ROS Image message and publishes to /image_converter/output_video
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(vis_image, 'bgr8'))

    def follow(self):
        rate      = rospy.Rate(10)
        img_width = 640

        while not rospy.is_shutdown():
            twist = Twist()

            if self.ball_col is not None:

                diff_ball_center = self.ball_col - img_width // 2 #current - desired
                twist.angular.z = -float(diff_ball_center) / 300.0 #Converts pixel error → turning speed

                if self.front_dist is not None:
                    dist_error = self.front_dist - self.target_dist

                    if dist_error > self.tolerance:
                        twist.linear.x = 0.2 # >1m distance + tolerate → move forward
                    elif dist_error < -self.tolerance:
                        twist.linear.x = -0.2 #<1m-tolerate → move backward
                    else:
                        twist.linear.x = 0.0
                    rospy.loginfo("dist: %.2fm | error: %.2fm | col: %d",
                                  self.front_dist, dist_error, self.ball_col)
                else:
                    twist.linear.x = 0.1  # ball visible but no depth → creep forward

            else:
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
                rospy.loginfo("No ball detected, stopping...")

            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ball_follower')
    follower = BallFollower()
    follower.follow()
