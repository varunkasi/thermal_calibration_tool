#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera interface package for the Thermal Camera Calibration Tool.

This package contains modules for interfacing with the FLIR Boson+
thermal camera and other camera-related functionality.
"""

from src.camera.camera_interface import CameraInterface
from src.camera.boson_camera import BosonCamera

__all__ = [
    'CameraInterface',
    'BosonCamera',
]