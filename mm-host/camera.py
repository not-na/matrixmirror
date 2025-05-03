#!/usr/bin/env python
# -*- coding: utf-8 -*-
__all__ = [
    "HAVE_PICAMERA",
    "PiCameraSource",
    "CVCameraSource",
    "autocam",
]

from typing import Dict

import numpy as np

import cv2

try:
    import picamera2
    from libcamera import Transform
    HAVE_PICAMERA = True
except ImportError:
    print("Could not import picamera2, PiCameraSource not available")
    HAVE_PICAMERA = False

import util


def autocam() -> util.FrameSource:
    if not HAVE_PICAMERA:
        return CVCameraSource()
    try:
        return PiCameraSource()
    except Exception:
        return CVCameraSource()


class PiCameraSource(util.FrameSource):
    def __init__(self):
        if not HAVE_PICAMERA:
            print("WARNING: Could not import picamera module, choose a different camera source")

        self.cam = picamera2.Picamera2()
        self.cam.configure(
            self.cam.create_video_configuration(
                transform=Transform()
            )
        )
        self.cam.start()

    def get_frame(self):
        frame: np.ndarray = self.cam.capture_array("main")

        if frame.shape[2] == 4:
            frame = frame[:, :, :3]
        assert frame.dtype == np.uint8

        return frame

    def get_config_params(self) -> Dict[str, Dict]:
        return {}

    def on_param_changed(self, param_name: str, new_value) -> None:
        pass


class CVCameraSource(util.FrameSource):
    def __init__(self, res=(640, 480)):
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
        print(f"Camera resolution: requested {res}, actual ({self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}, {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)})")

    def get_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            raise RuntimeError("Could not read camera frame")

        if frame.shape[2] == 4:
            frame = frame[:, :, :3]
        assert frame.dtype == np.uint8

        return frame

    def get_config_params(self) -> Dict[str, Dict]:
        return {}

    def on_param_changed(self, param_name: str, new_value) -> None:
        pass


