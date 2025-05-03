#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  display
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
import abc
import collections
import queue

import socket
import struct
import time
from pathlib import Path
from typing import Optional, Dict
import traceback
import threading

import cv2
import numpy as np

import serial
import serial.tools.list_ports

import spidev

import util

DISPLAY_SIZE = 64

MAGIC_PREAMBLE = b"\x00\xC0\xFE\xAA"

SERIAL_VID = 0x2e8a
SERIAL_PID = 0x000a
SERIAL_INTERFACE = "Board CDC"

SPI_SPEED_KHZ = 16000

class AbstractDisplay(util.FrameSink):
    def __init__(self, fps_limit: float = 60, show_fps: bool = True):
        self.last_frame = time.perf_counter()
        self.fps_limit = fps_limit
        self.show_fps = show_fps
        self.n_frame = 0

    def push_frame(self, frame):
        assert frame.shape == (DISPLAY_SIZE, DISPLAY_SIZE, 3), f"Frame shape is {frame.shape}"
        assert frame.dtype == np.uint8

        frame = frame.transpose((1, 0, 2))

        buf = frame.tobytes("C")
        assert len(buf) == DISPLAY_SIZE*DISPLAY_SIZE*3
        self.draw_frame(buf)

    def get_config_params(self) -> Dict[str, Dict]:
        return {}

    def on_param_changed(self, param_name: str, new_value) -> None:
        pass

    @abc.abstractmethod
    def send_buffer(self, buf: bytes):
        pass

    @abc.abstractmethod
    def close(self):
        pass

    def draw_frame(self, framebuf: bytes):
        t = time.perf_counter()
        wait_time = 1/self.fps_limit - (t - self.last_frame)
        if wait_time > 0:
            #print(f"Running too fast, sleeping for {wait_time} seconds")
            time.sleep(wait_time)

        st = time.perf_counter()
        buf = MAGIC_PREAMBLE+framebuf
        self.send_buffer(buf)
        et = time.perf_counter()
        if self.n_frame % self.fps_limit == 0:
            print(f"Frame update took {et - st:.4f} seconds ({len(buf) * 8 / (et - st) / 1024:.2f}kiBits/s) => {1 / (et - st):.4f} FPS max")

        if self.show_fps and self.n_frame % self.fps_limit == 0:
            print(f"Running at {1/(st-self.last_frame):.2f} FPS")
        self.last_frame = st
        self.n_frame += 1

    def set_solid_color(self, color):
        #print(f'Setting solid color to {color}')
        out_color = struct.pack("BBB", *color)

        out = bytearray(DISPLAY_SIZE*DISPLAY_SIZE*3)
        for i in range(DISPLAY_SIZE*DISPLAY_SIZE):
            out[i*3:i*3+3] = out_color

        self.draw_frame(bytes(out))


class TCPDisplay(AbstractDisplay):
    def __init__(self, serveraddr: str, port: int = 1234):
        super().__init__()

        self.serveraddr = serveraddr
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.serveraddr, self.port))

    def send_buffer(self, buf: bytes):
        try:
            self.sock.send(buf)
        except OSError as e:
            traceback.print_exc()
            print("Reconnecting...")
            self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.serveraddr, self.port))

    def close(self):
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()


class SerialUnixSocketDisplay(AbstractDisplay):
    def __init__(self, sockpath: Path):
        super().__init__()

        self.sockpath = sockpath.expanduser().resolve()

        if not self.sockpath.exists():
            raise RuntimeError(f"Couldn't find socket {self.sockpath}")
        elif not self.sockpath.is_socket():
            raise RuntimeError(f"{self.sockpath} is not a UNIX socket!")

        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(str(self.sockpath))

        #self.sockfile = self.sock.makefile("wb")

    def send_buffer(self, buf: bytes):
        print(f"Sending {len(buf)} bytes")
        #self.sockfile.write(buf)
        #self.sockfile.flush()
        for i in range(128):
            self.sock.send(bytes(buf[i*96:i*96+96]))
            # self.sock.close()
            #
            time.sleep(0.005)
            # self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            # self.sock.connect(str(self.sockpath))

    def close(self):
        self.sock.close()


class SerialDisplay(AbstractDisplay):
    def __init__(self, port: Optional[str] = None):
        super().__init__()

        if port is None:
            for candidate in serial.tools.list_ports.comports():
                if candidate.vid == SERIAL_VID and candidate.pid == SERIAL_PID and candidate.interface == SERIAL_INTERFACE:
                    port = candidate.device

            if port is None:
                raise RuntimeError(f"Couldn't find any suitable serial port!")

        self.port = port
        print(f"Using port {port}")

        self.serial = serial.Serial(port, timeout=0.1)

    def send_buffer(self, buf: bytes):
        #print(f"Sending {len(buf)} bytes")
        self.serial.write(buf)
        self.serial.flush()
        #print(f"Output: {self.serial.readlines()}")

    def close(self):
        self.serial.close()


class SPIDisplay(AbstractDisplay):
    def __init__(self, show_fps: bool = True):
        super().__init__(show_fps=show_fps)

        self.spi = spidev.SpiDev()
        self.spi.open(0,0)

        self.spi.max_speed_hz = SPI_SPEED_KHZ*1000
        self.spi.mode = 0b11

    def send_buffer(self, buf: bytes):
        self.spi.writebytes2(bytearray(buf))

    def close(self):
        self.spi.close()


class CVDisplay(util.FrameSink):
    def push_frame(self, frame):
        f = self.get_cfg("cv_display_upscale")
        frame = cv2.resize(frame, None, fx=f, fy=f, interpolation=cv2.INTER_NEAREST)

        cv2.imshow("MM Host preview", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt

    def close(self):
        pass

    def get_config_params(self) -> Dict[str, Dict]:
        return {
            "cv_display_upscale": {
                "default": 8,
            }
        }

    def on_param_changed(self, param_name: str, new_value) -> None:
        pass


class AsyncDisplay(AbstractDisplay):
    def __init__(self, display: AbstractDisplay):
        super().__init__()

        self.display = display

        self.queue = queue.Queue(maxsize=1)

        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

    def _run(self):
        while True:
            buf = self.queue.get()
            if buf is None:
                break
            self.display.draw_frame(buf)

    def draw_frame(self, buf: bytes):
        self.queue.put(buf)

    def send_buffer(self, buf: bytes):
        raise NotImplementedError("AsyncDisplay.send_buffer should not be called directly")

    def close(self):
        self.queue.put(None)
        self.thread.join()
        self.display.close()


if __name__ == "__main__":
    #display = SerialUnixSocketDisplay(Path("~/.tio-sock"))
    #display = SerialDisplay()
    #display = TCPDisplay("192.168.178.176")
    display = AsyncDisplay(SPIDisplay())

    colors = [
        (0, 0, 0),
        (255, 0, 0),
        (0, 255, 0),
        (0, 0, 255),
        (0, 255, 255),
        (255, 0, 255),
        (255, 255, 0),
        (255, 255, 255),
    ]

    try:
        while True:
            #for c in colors:
            #    display.set_solid_color(c)
            for i in range(0, 512, 1):
                #time.sleep(1.0)
                #time.sleep(0.5)
                n = i
                if i >= 256:
                    n = 511 - i
                display.set_solid_color((n, 0, 255-n))
    finally:
        display.close()
