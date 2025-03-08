#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests for the camera interface modules.

This module contains unit tests for the camera interface implementation.
"""

import unittest
from unittest.mock import patch, MagicMock
import numpy as np

from src.camera.boson_camera import BosonCamera
from src.utils.config import Config


class TestBosonCamera(unittest.TestCase):
    """Test cases for the BosonCamera class."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create a mock configuration
        self.config = Config()
        
        # Patch pyflir module
        self.pyflir_patcher = patch('src.camera.boson_camera.pyflir')
        self.mock_pyflir = self.pyflir_patcher.start()
        
        # Create a camera instance
        with patch('src.camera.boson_camera.BosonCamera.connect', return_value=True):
            self.camera = BosonCamera(self.config)
            self.camera.connected = False  # Reset connection state
    
    def tearDown(self):
        """Tear down test fixtures."""
        self.pyflir_patcher.stop()
    
    def test_connect(self):
        """Test camera connection."""
        # Mock successful connection
        self.camera.connect = MagicMock(return_value=True)
        
        result = self.camera.connect()
        
        self.assertTrue(result)
        self.camera.connect.assert_called_once()
    
    def test_disconnect(self):
        """Test camera disconnection."""
        # Set up mock camera
        self.camera.camera = MagicMock()
        self.camera.connected = True
        
        self.camera.disconnect()
        
        self.assertFalse(self.camera.connected)
        self.assertIsNone(self.camera.camera)
    
    def test_is_connected(self):
        """Test connection status check."""
        # Not connected
        self.camera.connected = False
        self.camera.camera = None
        self.assertFalse(self.camera.is_connected())
        
        # Connected
        self.camera.connected = True
        self.camera.camera = MagicMock()
        self.assertTrue(self.camera.is_connected())
    
    def test_get_frame(self):
        """Test frame acquisition."""
        # Not connected
        self.camera.connected = False
        self.assertIsNone(self.camera.get_frame())
        
        # Connected
        self.camera.connected = True
        self.camera.camera = MagicMock()
        
        # Test with simulated frame
        frame = self.camera.get_frame()
        
        self.assertIsNotNone(frame)
        self.assertEqual(frame.shape, (self.camera.height, self.camera.width))
        self.assertEqual(frame.dtype, np.uint16)
    
    def test_get_resolution(self):
        """Test resolution retrieval."""
        resolution = self.camera.get_resolution()
        self.assertEqual(resolution, (self.camera.width, self.camera.height))
        self.assertEqual(resolution, (640, 512))
    
    def test_get_raw_value_at_pixel(self):
        """Test pixel value retrieval."""
        # Create a mock frame
        mock_frame = np.zeros((512, 640), dtype=np.uint16)
        mock_frame[250, 320] = 12345  # Set a specific value
        
        # Set up camera
        self.camera.connected = True
        self.camera.last_frame = mock_frame
        
        # Valid pixel
        value = self.camera.get_raw_value_at_pixel(320, 250)
        self.assertEqual(value, 12345)
        
        # Out of bounds pixel
        value = self.camera.get_raw_value_at_pixel(700, 600)
        self.assertIsNone(value)
        
        # No frame available
        self.camera.last_frame = None
        value = self.camera.get_raw_value_at_pixel(320, 250)
        self.assertIsNone(value)


if __name__ == '__main__':
    unittest.main()