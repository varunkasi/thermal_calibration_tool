#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests for the utility modules.

This module contains unit tests for the configuration, logging, and data management utilities.
"""

import unittest
import tempfile
import os
import json
import yaml
from unittest.mock import patch, MagicMock

from src.utils.config import Config
from src.utils.logger import setup_logger, get_logger
from src.utils.data_manager import DataManager


class TestConfig(unittest.TestCase):
    """Test cases for the Config class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = Config()
    
    def test_get_default_values(self):
        """Test getting default configuration values."""
        # Check some default values
        self.assertTrue(self.config.get("camera.auto_connect"))
        self.assertEqual(self.config.get("camera.max_connection_attempts"), 3)
        self.assertEqual(self.config.get("gui.thermal_view.current_palette"), "Inferno")
    
    def test_get_with_default(self):
        """Test getting values with a default fallback."""
        # Non-existent key with default
        value = self.config.get("non.existent.key", "default_value")
        self.assertEqual(value, "default_value")
    
    def test_set_and_get(self):
        """Test setting and getting configuration values."""
        # Set a value
        self.config.set("test.key", "test_value")
        
        # Get the value back
        value = self.config.get("test.key")
        self.assertEqual(value, "test_value")
        
        # Set a nested value
        self.config.set("test.nested.key", 123)
        
        # Get the nested value
        value = self.config.get("test.nested.key")
        self.assertEqual(value, 123)
    
    def test_save_and_load(self):
        """Test saving and loading configuration to/from a file."""
        # Create a temporary file
        with tempfile.NamedTemporaryFile(suffix='.yaml', delete=False) as tf:
            filepath = tf.name
        
        try:
            # Set a test value
            self.config.set("test.key", "test_value")
            
            # Save to file
            result = self.config.save_to_file(filepath)
            self.assertTrue(result)
            
            # Create a new config
            new_config = Config()
            
            # Load from file
            result = new_config.load_from_file(filepath)
            self.assertTrue(result)
            
            # Check if value was loaded
            value = new_config.get("test.key")
            self.assertEqual(value, "test_value")
            
        finally:
            # Clean up
            if os.path.exists(filepath):
                os.unlink(filepath)
    
    def test_update(self):
        """Test updating configuration with new values."""
        # Create update data
        update_data = {
            "camera": {
                "auto_connect": False,
                "max_connection_attempts": 5
            },
            "new_section": {
                "new_key": "new_value"
            }
        }
        
        # Update configuration
        self.config.update(update_data)
        
        # Check updated values
        self.assertFalse(self.config.get("camera.auto_connect"))
        self.assertEqual(self.config.get("camera.max_connection_attempts"), 5)
        self.assertEqual(self.config.get("new_section.new_key"), "new_value")
        
        # Check that other values remain unchanged
        self.assertEqual(self.config.get("camera.connection_retry_delay"), 2)
    
    def test_reset_to_defaults(self):
        """Test resetting configuration to default values."""
        # Change some values
        self.config.set("camera.auto_connect", False)
        self.config.set("gui.thermal_view.current_palette", "Jet")
        
        # Reset to defaults
        self.config.reset_to_defaults()
        
        # Check that values are reset
        self.assertTrue(self.config.get("camera.auto_connect"))
        self.assertEqual(self.config.get("gui.thermal_view.current_palette"), "Inferno")


class TestLogger(unittest.TestCase):
    """Test cases for the logger module."""
    
    def test_setup_logger(self):
        """Test setting up a logger instance."""
        # Setup logger with custom configuration
        config = {
            "level": "DEBUG",
            "file": None,  # No file logging
            "console": True
        }
        
        logger = setup_logger(config)
        
        # Since loguru's interface is different from standard logging,
        # we mainly check that setup_logger returns a valid logger object
        self.assertIsNotNone(logger)
    
    def test_get_logger(self):
        """Test getting a logger instance with a specific name."""
        # Get a named logger
        logger = get_logger("test_module")
        
        # Check that we got a valid logger
        self.assertIsNotNone(logger)
        
        # Get a default logger
        default_logger = get_logger()
        
        # Check that we got a valid logger
        self.assertIsNotNone(default_logger)


class TestDataManager(unittest.TestCase):
    """Test cases for the DataManager class."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create a data manager with a temporary base directory
        self.temp_dir = tempfile.mkdtemp()
        self.data_manager = DataManager(self.temp_dir)
        
        # Sample calibration data
        self.calibration_data = {
            "model_type": "polynomial",
            "model_params": {
                "coefficients": [10.0, 0.001, 0.0000001]
            },
            "calibration_points": [
                {"x": 100, "y": 100, "raw_value": 10000, "reference_temp": 20.0},
                {"x": 200, "y": 200, "raw_value": 20000, "reference_temp": 30.0}
            ]
        }
        
        # Sample camera info
        self.camera_info = {
            "model": "FLIR Boson+",
            "serial_number": "123456789",
            "resolution": "640x512"
        }
    
    def tearDown(self):
        """Tear down test fixtures."""
        # Clean up temporary directory
        import shutil
        shutil.rmtree(self.temp_dir)
    
    def test_save_and_load_calibration(self):
        """Test saving and loading calibration data."""
        # Save calibration with a specific filename
        success, filepath = self.data_manager.save_calibration(
            self.calibration_data,
            filename="test_calibration.json",
            camera_info=self.camera_info
        )
        
        # Check results
        self.assertTrue(success)
        self.assertTrue(os.path.exists(filepath))
        
        # Load the calibration
        success, loaded_data = self.data_manager.load_calibration(filepath)
        
        # Check results
        self.assertTrue(success)
        self.assertEqual(loaded_data["model_type"], self.calibration_data["model_type"])
        self.assertEqual(len(loaded_data["calibration_points"]), 
                         len(self.calibration_data["calibration_points"]))
    
    def test_get_available_calibrations(self):
        """Test getting a list of available calibration files."""
        # Save a few calibration files
        self.data_manager.save_calibration(
            self.calibration_data,
            filename="test_cal_1.json"
        )
        self.data_manager.save_calibration(
            self.calibration_data,
            filename="test_cal_2.json"
        )
        
        # Get available calibrations
        calibrations = self.data_manager.get_available_calibrations()
        
        # Check results
        self.assertGreaterEqual(len(calibrations), 2)
        self.assertIn("filename", calibrations[0])
        self.assertIn("filepath", calibrations[0])
        self.assertIn("modified", calibrations[0])
    
    def test_export_calibration(self):
        """Test exporting calibration data."""
        # Export path
        export_path = os.path.join(self.temp_dir, "exports/exported_cal.json")
        
        # Export calibration
        success = self.data_manager.export_calibration(
            self.calibration_data,
            export_path,
            camera_info=self.camera_info,
            include_metadata=True
        )
        
        # Check results
        self.assertTrue(success)
        self.assertTrue(os.path.exists(export_path))
        
        # Check exported file content
        with open(export_path, 'r') as f:
            exported_data = json.load(f)
        
        # Validate structure
        self.assertIn("calibration", exported_data)
        self.assertIn("camera_info", exported_data)
        self.assertIn("metadata", exported_data)
    
    def test_save_and_load_thermal_image(self):
        """Test saving and loading a thermal image."""
        # Create a simple binary "image"
        image_data = bytes([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        
        # Metadata
        metadata = {
            "width": 2,
            "height": 5,
            "bit_depth": 8,
            "timestamp": 123456789.0
        }
        
        # Save the image
        success, filepath = self.data_manager.save_thermal_image(
            image_data,
            metadata=metadata,
            filename="test_image.bin"
        )
        
        # Check results
        self.assertTrue(success)
        self.assertTrue(os.path.exists(filepath))
        
        # Also check for metadata file
        metadata_path = os.path.splitext(filepath)[0] + '.json'
        self.assertTrue(os.path.exists(metadata_path))
        
        # Load the image
        success, loaded_data, loaded_metadata = self.data_manager.load_thermal_image(filepath)
        
        # Check results
        self.assertTrue(success)
        self.assertEqual(loaded_data, image_data)
        self.assertEqual(loaded_metadata["width"], metadata["width"])
        self.assertEqual(loaded_metadata["height"], metadata["height"])
    
    def test_backup_and_restore(self):
        """Test backing up and restoring data."""
        # Save some data first
        self.data_manager.save_calibration(
            self.calibration_data,
            filename="backup_test.json"
        )
        
        # Create a backup
        success, backup_path = self.data_manager.backup_data()
        
        # Check results
        self.assertTrue(success)
        self.assertTrue(os.path.exists(backup_path))
        
        # Clear the data directory
        for subdir in ['calibrations', 'images', 'exports']:
            for filename in os.listdir(os.path.join(self.temp_dir, subdir)):
                file_path = os.path.join(self.temp_dir, subdir, filename)
                if os.path.isfile(file_path):
                    os.unlink(file_path)
        
        # Restore from backup
        success = self.data_manager.restore_data(backup_path)
        
        # Check results
        self.assertTrue(success)
        
        # Check if files were restored
        restored_file = os.path.join(self.temp_dir, "calibrations/backup_test.json")
        self.assertTrue(os.path.exists(restored_file))


if __name__ == '__main__':
    unittest.main()