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

from util import *
from camera import *
from display import *
from components import *

def main():
    parser = argparse.ArgumentParser(description='MatrixMirror Host')

    parser.add_argument("--service", action="store_true", help="Run as a service. Disables high-frequency logging")
    parser.add_argument("--perfmon", action="store_true", help="Enable per-component performance logging")

    args = parser.parse_args()

    components: List[PipelineComponent] = [
        autocam(),
        #CVCameraSource((1280, 720)),
        SquareCrop(),
        ScaleDown(),
        #MotionExtract([0.5, 0.5]),

        AsyncDisplay(SPIDisplay(show_fps=not args.service)),
        #CVDisplay(),
    ]

    pipeline = Pipeline(components, enable_perfmonitor=args.perfmon)
    print(f"mm-host parameters:")
    pprint.pprint(pipeline.params)

    try:
        while True:
            pipeline.do_single_frame()
    finally:
        pipeline.close()

if __name__ == "__main__":
    main()