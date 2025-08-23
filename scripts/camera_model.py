#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import math
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

class OmmiUndistortion:
    def __init__(self):
        self.init_params()
        self.init_subscriber()
        self.init_publisher()
    
    def init_params(self):
        import numpy as np
        import cv2
        from cv_bridge import CvBridge

        # ---- Intrinsics (omni/Mei) ----
        self.xi = float(1)
        self.fx = float(1308.1691840205795)
        self.fy = float(1318.683585974909)
        self.cx = float(1221.8790961896898)
        self.cy = float(1052.024852216732)

        # ---- Image size ----
        self.width  = int(2448)
        self.height = int(2048)

        # ---- I/O ----
        self.bridge = CvBridge()

        # ---- Camera matrix & distortion (radtan: k1,k2,p1,p2) ----
        self.K = np.array([[self.fx, 0.0,      self.cx],
                        [0.0,      self.fy,  self.cy],
                        [0.0,      0.0,      1.0]], dtype=np.float64)

        self.D = np.array(
            [-0.13274940732954246, 0.3435168982763474,
            0.007927480590485703, -0.00010939869901162644],
            dtype=np.float64
        ).reshape(-1)  # (4,)

        # ---- Projection matrix (Knew): giữ FOV gốc, nhưng tâm ảnh đặt giữa khung mới ----
        Knew = self.K.copy()
        Knew[0, 2] = self.width  / 2.0  # cx_new
        Knew[1, 2] = self.height / 2.0  # cy_new
        Knew = Knew.astype(np.float64)

        # (Tuỳ chọn) Nếu muốn FOV ~90° thay vì giữ FOV gốc, bỏ comment đoạn dưới:
        # import math
        # f = 0.5 * self.width / math.tan(math.radians(90.0) * 0.5)
        # Knew = np.array([[f, 0, self.width/2.0],
        #                  [0, f, self.height/2.0],
        #                  [0, 0, 1]], dtype=np.float64)

        # ---- Rotation (view direction). Nếu không cần xoay, dùng identity float64 hoặc None ----
        self.R = np.eye(3, dtype=np.float64)   # hoặc: self.R = None

        # ---- Precompute remap (lưu ý: tham số thứ 7 là mltype, không phải m1type) ----
        # Size phải là (width, height)
        xi_scalar = np.array([self.xi], dtype=np.float64)

        self.mapx, self.mapy = cv2.omnidir.initUndistortRectifyMap(
            self.K,            # CV_64F, 3x3
            self.D,            # (4,), CV_64F
            xi_scalar,    # scalar
            self.R,            # None hoặc 3x3 CV_32F/CV_64F
            Knew,              # projection matrix cho ảnh đầu ra
            (self.width, self.height),
            cv2.CV_32FC1,      # <-- mltype (truyền theo vị trí)
            flags=cv2.omnidir.RECTIFY_PERSPECTIVE
        )

    def init_publisher(self):
        self.img_pub = rospy.Publisher("/camera/image_undirtortion", Image, queue_size=1)

    
    def init_subscriber(self):
        self.ommi_sub = rospy.Subscriber("/camera/image_color", Image, self.ommi_handler, queue_size= 1)

    def rectify(self, bgr):
        return cv2.remap(bgr, self.mapx, self.mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


    def ommi_handler(self, msg: Image):
        print("[Ommi Handle] received image.")
        try: 
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rect = self.rectify(bgr)

            out = self.bridge.cv2_to_imgmsg(rect, "bgr8"); out.header = msg.header
            self.img_pub.publish(out)

        except Exception as e:
            rospy.logwarn("cb_image error: %s", e)

        
def main():
    rospy.init_node("omnidir_undistort_node")
    OmmiUndistortion()
    rospy.spin()

if __name__ == "__main__":
    main()