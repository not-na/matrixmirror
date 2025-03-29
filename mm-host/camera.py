#!/usr/bin/env python
# -*- coding: utf-8 -*-
from typing import Dict

import numpy as np
import picamera2
from libcamera import Transform

import util


class CameraSource(util.FrameSource):
    def __init__(self):
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
