#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Abstract camera interface for the Thermal Camera Calibration Tool.

This module defines the abstract base class for camera interfaces,
allowing for different camera implementations while maintaining a consistent API.
"""

from abc import ABC, abstractmethod
from typing import Optional, Tuple, Any

import numpy as np


class CameraInterface(ABC):
    """Abstract base class for thermal camera interfaces."""

    @abstractmethod
    def connect(self) -> bool:
        """
        Connect to the camera.

        Returns:
            bool: True if connection successful, False otherwise.
        """
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the camera and release resources."""
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """
        Check if the camera is connected.

        Returns:
            bool: True if connected, False otherwise.
        """
        pass

    @abstractmethod
    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the current frame from the camera.

        Returns:
            Optional[np.ndarray]: Raw thermal data as a numpy array or None if unavailable.
        """
        pass

    @abstractmethod
    def get_resolution(self) -> Tuple[int, int]:
        """
        Get the camera resolution.

        Returns:
            Tuple[int, int]: Width and height of the camera resolution.
        """
        pass

    @abstractmethod
    def get_serial_number(self) -> str:
        """
        Get the camera serial number.

        Returns:
            str: Camera serial number.
        """
        pass

    @abstractmethod
    def get_camera_info(self) -> dict:
        """
        Get detailed camera information.

        Returns:
            dict: Dictionary containing camera information.
        """
        pass

    @abstractmethod
    def get_raw_value_at_pixel(self, x: int, y: int) -> Optional[int]:
        """
        Get the raw thermal value at the specified pixel.

        Args:
            x (int): X coordinate.
            y (int): Y coordinate.

        Returns:
            Optional[int]: Raw thermal value at the specified pixel or None if unavailable.
        """
        pass

    @abstractmethod
    def close(self) -> None:
        """Clean up resources and close the camera connection."""
        pass

    @abstractmethod
    def get_camera_properties(self) -> dict:
        """
        Get camera properties.

        Returns:
            dict: Dictionary of camera properties like bit depth, temperature range, etc.
        """
        pass

    @abstractmethod
    def set_camera_property(self, property_name: str, value: Any) -> bool:
        """
        Set a camera property.

        Args:
            property_name (str): Name of the property to set.
            value (Any): Value to set.

        Returns:
            bool: True if property was set successfully, False otherwise.
        """
        pass