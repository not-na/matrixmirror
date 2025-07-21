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
import cv2
import cv2 as cv

from queue import Queue

import numpy as np

from display import DISPLAY_SIZE
from util import *

class SquareCrop(FrameProcessor):
    def process(self, frame):
        h, w = frame.shape[0], frame.shape[1]
        assert h <= w, "Only landscape images are supported"

        if h == w:
            return frame  # Already square

        # Hack to get the later scale to an integer, halves processing time
        h = DISPLAY_SIZE * (h // DISPLAY_SIZE)

        # Crop to center
        return frame[0:h, (w-h)//2:(w-h)//2+h]

    def __str__(self):
        return "SquareCrop()"


@dataclass
class ScaleDown(FrameProcessor):
    scale_targetsize = DISPLAY_SIZE
    hybrid_factor = 2

    mode = "hybrid"
    def process(self, frame):
        size = self.scale_targetsize
        if self.mode == "hybrid":
            # Scale down to multiple of target res, then use area interpolation
            factor = self.hybrid_factor
            frame = cv.resize(frame, (size*factor, size*factor), interpolation=cv2.INTER_NEAREST)
            return cv.resize(frame, (size, size), interpolation=cv2.INTER_AREA)
        elif self.mode == "nearest":
            return cv.resize(frame, (size, size), interpolation=cv.INTER_NEAREST)
        elif self.mode == "area":
            return cv.resize(frame, (size, size), interpolation=cv.INTER_AREA)
        else:
            raise NotImplementedError

    def __str__(self):
        return f"{self.__class__.__name__}(mode={self.mode}, hybrid_factor={self.hybrid_factor})"


@dataclass
class MotionExtract(FrameProcessor):
    """
    ``weights`` is a list of floats between 0.0 and 1.0 that determine how strongly each old frame should be weighted.
    Index 0 is the current frame, the last index the oldest.

    The sum of all weights should be 1.0.
    """
    weights: List[float]

    def __post_init__(self):
        self.frame_queue = []

    def process(self, frame):
        if len(self.frame_queue) == 0:
            # Fill with current frame
            for i in range(len(self.weights)-1):
                self.frame_queue.append(frame)

        out = frame
        acc = self.weights[0]
        for i, weight in enumerate(self.weights[1:]):
            #print(f"{i=}: {weight=} fq={-(i+1)} {acc=}")
            if weight == 0:
                continue
            out = cv2.addWeighted(out, acc, self.frame_queue[-(i+1)], weight, 0)
            acc += weight

        # Pop one old frame off the list
        self.frame_queue.pop(0)

        # Add inverse of current frame to list
        self.frame_queue.append(cv2.bitwise_not(frame))

        return out


@dataclass
class DifferenceAmplify(FrameProcessor):
    bias: float = 0.5
    gain: float = 2

    def process(self, frame: np.ndarray):
        frame = frame.astype(np.float32) / 256
        frame -= self.bias
        frame *= self.gain
        frame += self.bias
        return (frame*256).astype(np.uint8)

@dataclass
class DifferenceAdd(FrameProcessor):
    base: FrameProcessor

    bias: float = 0.5
    gain: float = 2

    def process(self, frame: np.ndarray):
        frame = frame.astype(np.float32) / 256
        frame -= self.bias  # Remove bias => bias-free difference in range -1 to 1
        frame *= self.gain  # Apply gain
        frame += self.base.last_output.astype(np.float32) / 256
        return (frame*256).astype(np.uint8)