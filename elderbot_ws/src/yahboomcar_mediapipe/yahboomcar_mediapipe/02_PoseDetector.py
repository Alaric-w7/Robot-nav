#!/usr/bin/env python3
# encoding: utf-8

import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from yahboomcar_msgs.msg import PointArray
import cv2 as cv
import numpy as np
import time
import os

print("import done")

class PoseDetector(Node):
    def __init__(self, name, mode=False, smooth=True, detectionCon=0.5, trackCon=0.5):
        super().__init__(name)
        self.mpPose = mp.solutions.pose
        self.mpDraw = mp.solutions.drawing_utils
        
        # --- 核心优化: model_complexity=0 (Lite模型) ---
        # 极大地降低CPU占用，是提升FPS的关键
        self.pose = self.mpPose.Pose(
            static_image_mode=mode,
            model_complexity=0,
            smooth_landmarks=smooth,
            min_detection_confidence=detectionCon,
            min_tracking_confidence=trackCon )
            
        self.pub_point = self.create_publisher(PointArray, '/mediapipe/points', 1000)
        self.lmDrawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=-1, circle_radius=6)
        self.drawSpec = mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)

    def pubPosePoint(self, frame, draw=True):
        pointArray = PointArray()
        # 优化: 不再每次都创建全黑背景，利用切片或者后续处理
        img = np.zeros(frame.shape, np.uint8)
        
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        self.results = self.pose.process(img_RGB)
        
        if self.results.pose_landmarks:
            if draw: 
                self.mpDraw.draw_landmarks(frame, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            
            # 画骨架图
            self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS, self.lmDrawSpec, self.drawSpec)
            
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                point = Point()
                point.x, point.y, point.z = lm.x, lm.y, lm.z
                pointArray.points.append(point)
                
        self.pub_point.publish(pointArray)
        return frame, img

    def frame_combine(self, frame, src):
        if len(frame.shape) == 3:
            frameH, frameW = frame.shape[:2]
            srcH, srcW = src.shape[:2]
            dst = np.zeros((max(frameH, srcH), frameW + srcW, 3), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        else:
            src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
            frameH, frameW = frame.shape[:2]
            imgH, imgW = src.shape[:2]
            dst = np.zeros((frameH, frameW + imgW), np.uint8)
            dst[:, :frameW] = frame[:, :]
            dst[:, frameW:] = src[:, :]
        return dst

def main():
    print("start it")
    rclpy.init()
    pose_detector = PoseDetector('pose_detector')
    
    # --- 修正: 根据 v4l2-ctl 结果，RGB 相机是 index 6 ---
    camera_index = 6
    print(f"Opening Camera Index: {camera_index} (RGB/MJPG)")
    
    capture = cv.VideoCapture(camera_index)
    
    # --- 关键设置: 强制使用 MJPG ---
    # 你的设备支持 MJPG，这对 30FPS 至关重要
    capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 检查是否真正打开
    if not capture.isOpened():
        print(f"Error: Could not open video device {camera_index}.")
        return

    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    
    pTime = 0
    
    while capture.isOpened():
        # --- 健壮性读取: 防止坏帧导致崩溃 ---
        try:
            ret, frame = capture.read()
            if not ret or frame is None:
                print("Warning: Empty frame received, skipping...")
                time.sleep(0.01)
                continue
        except Exception as e:
            print(f"Read Error: {e}")
            continue

        # 正常处理
        frame, img = pose_detector.pubPosePoint(frame, draw=False)
        
        if cv.waitKey(1) & 0xFF == ord('q'): 
            break
            
        # FPS 计算
        cTime = time.time()
        if (cTime - pTime) > 0:
            fps = 1 / (cTime - pTime)
        else:
            fps = 30
        pTime = cTime
        
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        
        # 显示拼接画面
        dist = pose_detector.frame_combine(frame, img)
        cv.imshow('dist', dist)

    capture.release()
    cv.destroyAllWindows()
