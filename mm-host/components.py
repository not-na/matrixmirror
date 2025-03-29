#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  components
#
#  Copyright 2025 notna <notna@apparat.org>
#
#  This file is part of mm-host.
#
#  mm-host is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  mm-host is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with mm-host.  If not, see <http://www.gnu.org/licenses/>.
#

import cv2 as cv

from util import *

class SquareCrop(FrameProcessor):
    def process(self, frame):
        h, w = frame.shape[0], frame.shape[1]
        assert h <= w, "Only landscape images are supported"

        if h == w:
            return frame  # Already square

        # Crop to center
        return frame[0:h, (w-h)//2:(w-h)//2+h]

    def get_config_params(self) -> Dict[str, Dict]:
        return {}

    def on_param_changed(self, param_name: str, new_value) -> None:
        pass

class ScaleDown(FrameProcessor):
    def process(self, frame):
        size = self.get_cfg("scale_targetsize")
        return cv.resize(frame, (size, size), interpolation=cv.INTER_AREA)

    def get_config_params(self) -> Dict[str, Dict]:
        return {
            "scale_targetsize": {
                "default": 64,
            }
        }

    def on_param_changed(self, param_name: str, new_value) -> None:
        pass
