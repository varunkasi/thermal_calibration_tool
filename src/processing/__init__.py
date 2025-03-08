#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Image processing package for the Thermal Camera Calibration Tool.

This package contains modules for thermal image processing,
including visualization, filtering, and analysis.
"""

from src.processing.thermal_processor import ThermalProcessor, ColorPalette
from src.processing.image_utils import (
    apply_gaussian_filter, apply_median_filter, normalize_image,
    scale_image, create_pseudo_color_image
)

__all__ = [
    'ThermalProcessor',
    'ColorPalette',
    'apply_gaussian_filter',
    'apply_median_filter',
    'normalize_image',
    'scale_image',
    'create_pseudo_color_image',
]