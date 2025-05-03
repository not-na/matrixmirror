#!/usr/bin/env python
# -*- coding: utf-8 -*-
import abc
import time
from typing import List, Type, Dict


class PipelineComponent(abc.ABC):
    @abc.abstractmethod
    def get_config_params(self) -> Dict[str, Dict]:
        pass

    @abc.abstractmethod
    def on_param_changed(self, param_name: str, new_value) -> None:
        pass

    def get_cfg(self, key: str):
        return self.pipeline.get_cfg(key)

    def close(self):
        pass


class FrameSink(PipelineComponent):
    @abc.abstractmethod
    def push_frame(self, frame):
        pass


class FrameSource(PipelineComponent):
    @abc.abstractmethod
    def get_frame(self):
        pass


class FrameProcessor(PipelineComponent):
    @abc.abstractmethod
    def process(self, frame):
        pass


class Pipeline:
    def __init__(self, pipeline: List[PipelineComponent], enable_perfmonitor: bool = False):
        self.perfmonitor = enable_perfmonitor

        self.pipeline = pipeline
        assert isinstance(self.pipeline[0], FrameSource)
        for c in self.pipeline[1:-1]:
            assert isinstance(c, FrameProcessor)
        assert isinstance(self.pipeline[-1], FrameSink)

        self.params = {k: v for c in self.pipeline for k, v in c.get_config_params().items()}

        self.cfg = {k: v["default"] for k, v in self.params.items()}

        for c in self.pipeline:
            c.pipeline = self

        self.n_frame = 0

    def get_cfg(self, key: str):
        return self.cfg[key]

    def set_cfg(self, key, value):
        self.cfg[key] = value
        for c in self.pipeline:
            if key in c.get_config_params():
                c.on_param_changed(key, value)

    def do_single_frame(self):
        s1t = time.perf_counter()
        f = self.pipeline[0].get_frame()
        et = time.perf_counter()
        if self.perfmonitor and self.n_frame % 60 == 0:
            print("===========================")
            print(f"Source took {et-s1t:.6f}s => output shape {f.shape}")

        for c in self.pipeline[1:-1]:
            st = time.perf_counter()
            f = c.process(f)
            et = time.perf_counter()

            if self.perfmonitor and self.n_frame % 60 == 0:
                print(f"Component {c.__class__.__name__} took {et - st:.6f}s => output shape {f.shape}")

        st = time.perf_counter()
        self.pipeline[-1].push_frame(f)
        et = time.perf_counter()

        if self.perfmonitor and self.n_frame % 60 == 0:
            print(f"Sink took {et - st:.6f}s for shape {f.shape}")
            print(f" => Overall: {et-s1t:.6f}s ({1/(et-s1t):.3f}FPS)")

        self.n_frame += 1

    def close(self):
        for c in self.pipeline:
            c.close()
