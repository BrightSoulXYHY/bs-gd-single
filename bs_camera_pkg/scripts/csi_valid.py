#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2

def gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
 
 
def show_camera():
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
 
    while cap.isOpened():
        flag, img = cap.read()
        cv2.imshow("CSI Camera", img)
        kk = cv2.waitKey(1)

 
        # do other things
 
        if kk == ord('q'):  # 按下 q 键，退出
            break
        if kk==ord('s'):
            cv2.imwrite('./c01.jpg',img)
 
    cap.release()
    cv2.destroyAllWindows()
    
 
if __name__ == "__main__":
    show_camera()
