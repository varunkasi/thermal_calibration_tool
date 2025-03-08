#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Thermal image processing for the Thermal Camera Calibration Tool.

This module provides functions for processing raw thermal camera data,
including normalization, visualization, and color mapping.
"""

import enum
from typing import Optional, Tuple, Dict, Any, List, Union

import numpy as np
from loguru import logger

from src.utils.config import Config


class ColorPalette(enum.Enum):
    """Enumeration of available color palettes for thermal visualization."""
    
    INFERNO = "inferno"
    JET = "jet"
    VIRIDIS = "viridis"
    GRAYSCALE = "grayscale"
    IRONBOW = "ironbow"


class ThermalProcessor:
    """Processor for thermal image data."""
    
    def __init__(self, config: Config):
        """
        Initialize the thermal processor.
        
        Args:
            config (Config): Configuration instance.
        """
        self.config = config
        
        # Set default palette
        palette_name = config.get("gui.thermal_view.current_palette", "Inferno")
        self.current_palette = ColorPalette[palette_name.upper()]
        
        # Initialize processing parameters
        self.auto_range = config.get("processing.auto_range", True)
        self.min_value = config.get("processing.min_value", 0)
        self.max_value = config.get("processing.max_value", 65535)
        
        # Last processed frame info
        self.last_min = 0
        self.last_max = 65535
        
        # Generate color maps
        self._generate_color_maps()
    
    def process_frame(
        self, 
        frame: np.ndarray, 
        min_value: Optional[int] = None, 
        max_value: Optional[int] = None
    ) -> np.ndarray:
        """
        Process a raw thermal frame for display.
        
        Args:
            frame (np.ndarray): Raw thermal frame (16-bit).
            min_value (Optional[int], optional): Minimum value for normalization.
                Defaults to None (auto-calculated).
            max_value (Optional[int], optional): Maximum value for normalization.
                Defaults to None (auto-calculated).
        
        Returns:
            np.ndarray: Processed frame for display (8-bit RGB).
        """
        # Ensure frame is numpy array with proper dtype
        if frame is None:
            logger.warning("Received None frame for processing")
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Ensure we have a 2D array
        if len(frame.shape) != 2:
            logger.warning(f"Expected 2D frame, got {frame.shape}. Reshaping if possible.")
            try:
                frame = frame.reshape(frame.shape[0], frame.shape[1])
            except Exception as e:
                logger.error(f"Could not reshape frame: {e}")
                return np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)
        
        # Determine value range for normalization
        if self.auto_range:
            # Calculate non-zero percentiles to avoid outliers
            non_zero = frame[frame > 0]
            if len(non_zero) > 0:
                self.last_min = np.percentile(non_zero, 1)
                self.last_max = np.percentile(non_zero, 99)
            else:
                self.last_min = 0
                self.last_max = 65535
        else:
            # Use provided values or defaults
            self.last_min = min_value if min_value is not None else self.min_value
            self.last_max = max_value if max_value is not None else self.max_value
        
        # Ensure we have a valid range
        if self.last_min >= self.last_max:
            self.last_max = self.last_min + 1
        
        # Normalize frame to 0-1 range
        norm_frame = np.clip((frame - self.last_min) / (self.last_max - self.last_min), 0, 1)
        
        # Apply color palette
        return self._apply_colormap(norm_frame)
    
    def _generate_color_maps(self):
        """Generate color maps for different palettes."""
        self.color_maps = {}
        
        # Inferno colormap (perceptually uniform, good for thermal imaging)
        # Approximation of Matplotlib's inferno
        self.color_maps[ColorPalette.INFERNO] = self._generate_inferno_colormap()
        
        # Jet colormap (traditional rainbow)
        self.color_maps[ColorPalette.JET] = self._generate_jet_colormap()
        
        # Viridis colormap (perceptually uniform, colorblind-friendly)
        # Approximation of Matplotlib's viridis
        self.color_maps[ColorPalette.VIRIDIS] = self._generate_viridis_colormap()
        
        # Grayscale colormap
        self.color_maps[ColorPalette.GRAYSCALE] = self._generate_grayscale_colormap()
        
        # Ironbow colormap (popular in thermal imaging, similar to FLIR's ironbow)
        self.color_maps[ColorPalette.IRONBOW] = self._generate_ironbow_colormap()
    
    def _generate_inferno_colormap(self) -> np.ndarray:
        """
        Generate an approximation of the Inferno colormap.
        
        Returns:
            np.ndarray: Colormap array of shape (256, 3).
        """
        # Simplified approximation of Matplotlib's inferno
        colormap = np.zeros((256, 3), dtype=np.uint8)
        
        # Generate colormap
        for i in range(256):
            # Normalized position
            t = i / 255.0
            
            # RGB calculation (approximation)
            r = np.clip(np.interp(t, [0.0, 0.5, 1.0], [0, 255, 255]), 0, 255)
            g = np.clip(np.interp(t, [0.0, 0.4, 0.8, 1.0], [0, 0, 140, 255]), 0, 255)
            b = np.clip(np.interp(t, [0.0, 0.3, 0.8, 1.0], [0, 120, 100, 0]), 0, 255)
            
            colormap[i] = [r, g, b]
        
        return colormap
    
    def _generate_jet_colormap(self) -> np.ndarray:
        """
        Generate a Jet colormap (blue-cyan-yellow-red).
        
        Returns:
            np.ndarray: Colormap array of shape (256, 3).
        """
        colormap = np.zeros((256, 3), dtype=np.uint8)
        
        # Generate colormap
        for i in range(256):
            # Normalized position
            t = i / 255.0
            
            # RGB calculation
            r = np.clip(np.interp(t, [0.0, 0.35, 0.66, 0.89, 1.0], [0, 0, 255, 255, 128]), 0, 255)
            g = np.clip(np.interp(t, [0.0, 0.125, 0.375, 0.64, 0.91, 1.0], 
                                 [0, 0, 255, 255, 0, 0]), 0, 255)
            b = np.clip(np.interp(t, [0.0, 0.11, 0.34, 0.65, 1.0], 
                                 [128, 255, 255, 0, 0]), 0, 255)
            
            colormap[i] = [r, g, b]
        
        return colormap
    
    def _generate_viridis_colormap(self) -> np.ndarray:
        """
        Generate an approximation of the Viridis colormap.
        
        Returns:
            np.ndarray: Colormap array of shape (256, 3).
        """
        # Simplified approximation of Matplotlib's viridis
        colormap = np.zeros((256, 3), dtype=np.uint8)
        
        # Generate colormap
        for i in range(256):
            # Normalized position
            t = i / 255.0
            
            # RGB calculation (approximation)
            r = np.clip(np.interp(t, [0.0, 0.4, 0.8, 1.0], [68, 33, 215, 253]), 0, 255)
            g = np.clip(np.interp(t, [0.0, 0.4, 0.8, 1.0], [1, 144, 209, 231]), 0, 255)
            b = np.clip(np.interp(t, [0.0, 0.4, 0.8, 1.0], [84, 140, 64, 37]), 0, 255)
            
            colormap[i] = [r, g, b]
        
        return colormap
    
    def _generate_grayscale_colormap(self) -> np.ndarray:
        """
        Generate a Grayscale colormap.
        
        Returns:
            np.ndarray: Colormap array of shape (256, 3).
        """
        colormap = np.zeros((256, 3), dtype=np.uint8)
        
        # Generate colormap
        for i in range(256):
            colormap[i] = [i, i, i]
        
        return colormap
    
    def _generate_ironbow_colormap(self) -> np.ndarray:
        """
        Generate an approximation of the Ironbow colormap (similar to FLIR's ironbow).
        
        Returns:
            np.ndarray: Colormap array of shape (256, 3).
        """
        colormap = np.zeros((256, 3), dtype=np.uint8)
        
        # Generate colormap
        for i in range(256):
            # Normalized position
            t = i / 255.0
            
            # RGB calculation (approximation of ironbow)
            r = np.clip(np.interp(t, [0.0, 0.2, 0.4, 0.6, 0.8, 1.0], 
                                 [0, 16, 120, 220, 255, 255]), 0, 255)
            g = np.clip(np.interp(t, [0.0, 0.2, 0.4, 0.6, 0.8, 1.0], 
                                 [0, 0, 32, 120, 220, 255]), 0, 255)
            b = np.clip(np.interp(t, [0.0, 0.2, 0.4, 0.6, 0.8, 1.0], 
                                 [0, 60, 160, 68, 32, 0]), 0, 255)
            
            colormap[i] = [r, g, b]
        
        return colormap
    
    def _apply_colormap(self, normalized_frame: np.ndarray) -> np.ndarray:
        """
        Apply a colormap to a normalized frame.
        
        Args:
            normalized_frame (np.ndarray): Normalized frame (0-1 range).
        
        Returns:
            np.ndarray: Colorized frame (8-bit RGB).
        """
        # Get selected colormap
        colormap = self.color_maps.get(self.current_palette, self.color_maps[ColorPalette.INFERNO])
        
        # Convert normalized values to colormap indices
        indices = np.clip(normalized_frame * 255, 0, 255).astype(np.uint8)
        
        # Map indices to colors
        height, width = indices.shape
        rgb_frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Vectorized mapping
        rgb_frame = colormap[indices]
        
        return rgb_frame
    
    def set_palette(self, palette: Union[ColorPalette, str]):
        """
        Set the color palette.
        
        Args:
            palette (Union[ColorPalette, str]): Color palette to use.
        """
        if isinstance(palette, str):
            try:
                palette = ColorPalette[palette.upper()]
            except KeyError:
                logger.warning(f"Unknown palette: {palette}. Using default.")
                palette = ColorPalette.INFERNO
        
        if palette not in self.color_maps:
            logger.warning(f"Palette {palette} not found in color maps. Using default.")
            palette = ColorPalette.INFERNO
        
        self.current_palette = palette
    
    def set_range(self, min_value: int, max_value: int):
        """
        Set the value range for normalization.
        
        Args:
            min_value (int): Minimum value.
            max_value (int): Maximum value.
        """
        if min_value >= max_value:
            logger.warning(f"Invalid range: {min_value} >= {max_value}. Adjusting.")
            max_value = min_value + 1
        
        self.min_value = min_value
        self.max_value = max_value
        self.auto_range = False
    
    def set_auto_range(self, auto_range: bool):
        """
        Set whether to automatically determine the value range.
        
        Args:
            auto_range (bool): Whether to use auto range.
        """
        self.auto_range = auto_range
    
    def get_current_range(self) -> Tuple[int, int]:
        """
        Get the current value range.
        
        Returns:
            Tuple[int, int]: (min_value, max_value)
        """
        return (self.last_min, self.last_max)
    
    def create_color_scale_image(self, width: int, height: int) -> np.ndarray:
        """
        Create a color scale image for the current palette.
        
        Args:
            width (int): Width of the image.
            height (int): Height of the image.
        
        Returns:
            np.ndarray: Color scale image (8-bit RGB).
        """
        # Create a gradient from 0 to 1
        gradient = np.linspace(0, 1, width)
        gradient = np.tile(gradient, (height, 1))
        
        # Apply colormap
        return self._apply_colormap(gradient)