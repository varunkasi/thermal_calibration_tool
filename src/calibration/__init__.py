#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration package for the Thermal Camera Calibration Tool.

This package contains modules for thermal camera calibration,
including regression analysis and calibration model management.
"""

from src.calibration.calibration_model import CalibrationModel, CalibrationPoint
from src.calibration.regression import create_calibration_function, evaluate_calibration

__all__ = [
    'CalibrationModel',
    'CalibrationPoint',
    'create_calibration_function',
    'evaluate_calibration',
]