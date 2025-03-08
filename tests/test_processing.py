#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests for the image processing modules.

This module contains unit tests for the thermal image processing functionality.
"""

import unittest
import numpy as np

from src.processing.thermal_processor import ThermalProcessor, ColorPalette
from src.processing.image_utils import (
    apply_gaussian_filter, apply_median_filter, normalize_image,
    scale_image, create_pseudo_color_image
)
from src.utils.config import Config


class TestThermalProcessor(unittest.TestCase):
    """Test cases for the ThermalProcessor class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = Config()
        self.processor = ThermalProcessor(self.config)
        
        # Create a test thermal image (gradient from 0 to 65535)
        height, width = 100, 200
        self.test_image = np.zeros((height, width), dtype=np.uint16)
        for y in range(height):
            for x in range(width):
                # Create a gradient
                self.test_image[y, x] = int((x / width) * 65535)
    
    def test_process_frame(self):
        """Test processing a thermal frame."""
        # Process the test image
        processed = self.processor.process_frame(self.test_image)
        
        # Check output shape and type
        self.assertEqual(processed.shape[:2], self.test_image.shape)
        self.assertEqual(processed.shape[2], 3)  # RGB output
        self.assertEqual(processed.dtype, np.uint8)
    
    def test_set_palette(self):
        """Test setting the color palette."""
        # Test enum-based setting
        self.processor.set_palette(ColorPalette.JET)
        self.assertEqual(self.processor.current_palette, ColorPalette.JET)
        
        # Test string-based setting
        self.processor.set_palette("INFERNO")
        self.assertEqual(self.processor.current_palette, ColorPalette.INFERNO)
        
        # Test invalid palette
        self.processor.set_palette("INVALID")
        self.assertEqual(self.processor.current_palette, ColorPalette.INFERNO)  # Should stay the same
    
    def test_set_range(self):
        """Test setting the normalization range."""
        # Set a custom range
        self.processor.set_range(10000, 50000)
        
        # Auto range should be disabled
        self.assertFalse(self.processor.auto_range)
        self.assertEqual(self.processor.min_value, 10000)
        self.assertEqual(self.processor.max_value, 50000)
        
        # Test invalid range (min > max)
        self.processor.set_range(50000, 10000)
        self.assertEqual(self.processor.min_value, 50000)
        self.assertEqual(self.processor.max_value, 50001)  # Should adjust max to min+1
    
    def test_apply_colormap(self):
        """Test applying a colormap to normalized data."""
        # Create a simple normalized gradient
        normalized = np.linspace(0, 1, 256).reshape(1, 256)
        
        # Apply colormap
        colorized = self.processor._apply_colormap(normalized)
        
        # Check output shape and type
        self.assertEqual(colorized.shape[:2], normalized.shape)
        self.assertEqual(colorized.shape[2], 3)  # RGB output
        self.assertEqual(colorized.dtype, np.uint8)
        
        # Inferno colormap should map 0->dark, 1->bright
        self.assertTrue(np.all(colorized[0, 0] < colorized[0, -1]))
    
    def test_create_color_scale_image(self):
        """Test creating a color scale image."""
        # Create a color scale
        width, height = 256, 32
        scale = self.processor.create_color_scale_image(width, height)
        
        # Check output shape and type
        self.assertEqual(scale.shape, (height, width, 3))
        self.assertEqual(scale.dtype, np.uint8)


class TestImageUtils(unittest.TestCase):
    """Test cases for the image utility functions."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create a test image (10x10, 8-bit)
        self.test_image = np.zeros((10, 10), dtype=np.uint8)
        
        # Add some test patterns
        # Cross pattern
        self.test_image[4:6, :] = 128
        self.test_image[:, 4:6] = 128
        
        # Corner values
        self.test_image[0:3, 0:3] = 255
        self.test_image[7:10, 7:10] = 255
        
        # Noisy point
        self.test_image[2, 7] = 200
    
    def test_apply_gaussian_filter(self):
        """Test Gaussian filtering."""
        # Apply filter
        filtered = apply_gaussian_filter(self.test_image, sigma=1.0)
        
        # Check output shape and type
        self.assertEqual(filtered.shape, self.test_image.shape)
        self.assertEqual(filtered.dtype, self.test_image.dtype)
        
        # Filtering should reduce extremes
        self.assertTrue(np.max(filtered) < np.max(self.test_image))
    
    def test_apply_median_filter(self):
        """Test median filtering."""
        # Apply filter
        filtered = apply_median_filter(self.test_image, kernel_size=3)
        
        # Check output shape and type
        self.assertEqual(filtered.shape, self.test_image.shape)
        self.assertEqual(filtered.dtype, self.test_image.dtype)
        
        # Median filter should remove the noisy point
        self.assertNotEqual(self.test_image[2, 7], filtered[2, 7])
    
    def test_normalize_image(self):
        """Test image normalization."""
        # Create a 16-bit test image with values from 1000 to 5000
        test = np.ones((10, 10), dtype=np.uint16) * 1000
        test[5:, 5:] = 5000
        
        # Normalize to 8-bit
        normalized = normalize_image(test, min_val=1000, max_val=5000)
        
        # Check output shape and type
        self.assertEqual(normalized.shape, test.shape)
        self.assertEqual(normalized.dtype, np.uint8)
        
        # Check normalization results
        self.assertEqual(normalized[0, 0], 0)    # Min value -> 0
        self.assertEqual(normalized[9, 9], 255)  # Max value -> 255
    
    def test_scale_image(self):
        """Test image scaling."""
        # Scale up
        scaled_up = scale_image(self.test_image, scale_factor=2.0)
        self.assertEqual(scaled_up.shape, (20, 20))
        
        # Scale down
        scaled_down = scale_image(self.test_image, scale_factor=0.5)
        self.assertEqual(scaled_down.shape, (5, 5))
    
    def test_create_pseudo_color_image(self):
        """Test creating a pseudo-color image from grayscale."""
        # Create a pseudo-color image
        colored = create_pseudo_color_image(self.test_image, colormap_name="inferno")
        
        # Check output shape and type
        self.assertEqual(colored.shape[:2], self.test_image.shape)
        self.assertEqual(colored.shape[2], 3)  # RGB output
        self.assertEqual(colored.dtype, np.uint8)


if __name__ == '__main__':
    unittest.main()