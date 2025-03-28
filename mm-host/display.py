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

import socket
import struct
import time
from pathlib import Path
from typing import Optional
import traceback

import serial
import serial.tools.list_ports

DISPLAY_SIZE = 64

MAGIC_PREAMBLE = b"\x00\xC0\xFE\xAA"

SERIAL_VID = 0x2e8a
SERIAL_PID = 0x000a
SERIAL_INTERFACE = "Board CDC"

class AbstractDisplay(abc.ABC):
    def __init__(self, fps_limit: float = 30):
        self.last_frame = time.perf_counter()
        self.fps_limit = fps_limit

    @abc.abstractmethod
    def send_buffer(self, buf: bytes):
        pass

    @abc.abstractmethod
    def close(self):
        pass

    def draw_frame(self, framebuf: bytes):
        wait_time = 1/self.fps_limit - (time.perf_counter() - self.last_frame)
        if wait_time > 0:
            time.sleep(wait_time)
        self.send_buffer(MAGIC_PREAMBLE+framebuf)

        t = time.perf_counter()
        print(f"Running at {1/(t-self.last_frame):.2f} FPS")
        self.last_frame = t

    def set_solid_color(self, color):
        print(f'Setting solid color to {color}')
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
        st = time.perf_counter()
        self.serial.write(buf)
        self.serial.flush()
        et = time.perf_counter()
        print(f"Frame update took {et-st:.4f} seconds ({len(buf)*8/(et-st)/1024:.2f}kiBits/s) => {1/(et-st):.4f} FPS")
        #print(f"Output: {self.serial.readlines()}")

    def close(self):
        self.serial.close()


if __name__ == "__main__":
    #display = SerialUnixSocketDisplay(Path("~/.tio-sock"))
    #display = SerialDisplay()
    display = TCPDisplay("192.168.178.176")

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
        display.set_solid_color((255,255,255))

        while True:
            #for c in colors:
            #    display.set_solid_color(c)
            for i in range(0, 512, 4):
                #time.sleep(1.0)
                #time.sleep(0.1)
                n = i
                if i >= 256:
                    n = 511 - i
                display.set_solid_color((n, 0, 255-n))
    finally:
        display.close()
