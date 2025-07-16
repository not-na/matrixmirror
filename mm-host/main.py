#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  main
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
import pprint

import argparse
import subprocess
from typing import Callable

from util import *
from camera import *
from display import *
from components import *

BUTTON_AUTO_LOCK_TIME = 30

def check_unlock():
    global buttons_locked, buttons_last_active

    if all(map(lambda btn: btn.is_pressed, BUTTON_UNLOCK_COMBO)):
        print("Unlocked!")
        buttons_locked = False
        buttons_last_active = time.monotonic()
        return False  # don't process this press yet

    if time.monotonic() - buttons_last_active > BUTTON_AUTO_LOCK_TIME:
        print("Locked after timeout!")
        buttons_locked = True
        return False

    if not buttons_locked:
        buttons_last_active = time.monotonic()

    print(f"Lock state: {buttons_locked}")
    return not buttons_locked

def update_mode():
    print(f"(Re-)loading pipeline for mode {mode_names[cur_mode]}")
    pipeline.replace_core(modes[mode_names[cur_mode]]())
    print(f"New pipeline for mode {mode_names[cur_mode]}:\n{pipeline}")

try:
    import gpiozero

    buttons_locked = True
    buttons_last_active = time.monotonic()

    button_reset = gpiozero.Button(24, bounce_time=0.1)
    button_next = gpiozero.Button(23, bounce_time=0.1)
    button_prev = gpiozero.Button(22, bounce_time=0.1)
    button_shutdown = gpiozero.Button(27, hold_time=1, bounce_time=0.1)

    BUTTON_UNLOCK_COMBO = [button_reset, button_prev]

    def on_button_reset():
        print("Raw press: reset")
        if not check_unlock():
            return
        print("Reset button pressed")

        update_mode()
    button_reset.when_pressed = on_button_reset

    def on_button_prev():
        print("Raw press: prev")
        if not check_unlock():
            return
        print("Prev button pressed")

        global cur_mode
        cur_mode -= 1
        if cur_mode < 0:
            cur_mode = len(modes) - 1

        update_mode()
    button_prev.when_pressed = on_button_prev

    def on_button_next():
        print("Raw press: next")
        if not check_unlock():
            return
        print("Next button pressed")

        global cur_mode
        cur_mode += 1
        if cur_mode >= len(modes):
            cur_mode = 0

        update_mode()
    button_next.when_pressed = on_button_next

    def on_button_shutdown():
        print("Raw press: shutdown")
        if not check_unlock():
            return

        print("Shutting down")
        subprocess.call(["sudo", "poweroff"])

    button_shutdown.when_held = on_button_shutdown

    print("Buttons enabled")
    has_buttons = True
except ImportError:
    has_buttons = False
    print("Warning: gpiozero is not installed")

modes: Dict[str, Callable[[], List[FrameProcessor]]] = {
    "basic": lambda: [
        SquareCrop(),
        ScaleDown(),
        #MotionExtract([0.5, 0.5]),
    ],
    "motion_basic": lambda: [
        SquareCrop(),
        ScaleDown(),
        MotionExtract([0.5, 0.5]),
    ]
}

mode_names = list(modes.keys())
print(f"Available modes ({len(mode_names)}): {mode_names}")
cur_mode = 0

parser = argparse.ArgumentParser(description='MatrixMirror Host')

parser.add_argument("--service", action="store_true", help="Run as a service. Disables high-frequency logging")
parser.add_argument("--perfmon", action="store_true", help="Enable per-component performance logging")

args = parser.parse_args()

components: List[PipelineComponent] = [
    autocam(),
    #CVCameraSource((1280, 720)),

    # Mode-specific components will be inserted here

    SplitDisplay(
        left=NullDisplay(), #AsyncDisplay(SPIDisplay(bus=1, show_fps=not args.service)),
        right=AsyncDisplay(SPIDisplay(bus=0, show_fps=not args.service)),
    ),
    #CVDisplay(),
]

pipeline = Pipeline(components, enable_perfmonitor=args.perfmon)
update_mode()

print("Done initializing, starting pipeline")

try:
    i = 0
    while True:
        i += 1
        pipeline.do_single_frame()

        # if i % 30 == 0:
        #     if has_buttons:
        #         print(f"Button states: prev={button_prev.value} next={button_next.value} reset={button_reset.value} shutdown={button_shutdown.value}")
finally:
    pipeline.close()
