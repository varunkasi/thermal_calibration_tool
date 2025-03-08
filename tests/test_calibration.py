#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests for the calibration modules.

This module contains unit tests for the calibration model implementation.
"""

import unittest
import tempfile
import os
import numpy as np

from src.calibration.calibration_model import CalibrationModel, CalibrationPoint
from src.calibration.regression import create_calibration_function, evaluate_calibration


class TestCalibrationPoint(unittest.TestCase):
    """Test cases for the CalibrationPoint class."""
    
    def test_creation(self):
        """Test calibration point creation."""
        point = CalibrationPoint(x=10, y=20, raw_value=12345, reference_temp=25.5)
        
        self.assertEqual(point.x, 10)
        self.assertEqual(point.y, 20)
        self.assertEqual(point.raw_value, 12345)
        self.assertEqual(point.reference_temp, 25.5)
        self.assertIsNotNone(point.timestamp)
    
    def test_to_dict(self):
        """Test conversion to dictionary."""
        point = CalibrationPoint(x=10, y=20, raw_value=12345, reference_temp=25.5, timestamp=123456789.0)
        
        data = point.to_dict()
        
        self.assertEqual(data["x"], 10)
        self.assertEqual(data["y"], 20)
        self.assertEqual(data["raw_value"], 12345)
        self.assertEqual(data["reference_temp"], 25.5)
        self.assertEqual(data["timestamp"], 123456789.0)
    
    def test_from_dict(self):
        """Test creation from dictionary."""
        data = {
            "x": 10,
            "y": 20,
            "raw_value": 12345,
            "reference_temp": 25.5,
            "timestamp": 123456789.0
        }
        
        point = CalibrationPoint.from_dict(data)
        
        self.assertEqual(point.x, 10)
        self.assertEqual(point.y, 20)
        self.assertEqual(point.raw_value, 12345)
        self.assertEqual(point.reference_temp, 25.5)
        self.assertEqual(point.timestamp, 123456789.0)


class TestCalibrationModel(unittest.TestCase):
    """Test cases for the CalibrationModel class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.model = CalibrationModel()
        
        # Add some calibration points
        self.model.add_calibration_point(100, 100, 10000, 20.0)
        self.model.add_calibration_point(200, 200, 20000, 30.0)
        self.model.add_calibration_point(300, 300, 30000, 40.0)
        self.model.add_calibration_point(400, 400, 40000, 50.0)
    
    def test_add_calibration_point(self):
        """Test adding calibration points."""
        # Check points were added in setUp
        self.assertEqual(len(self.model.calibration_points), 4)
        
        # Add another point
        point = self.model.add_calibration_point(500, 500, 50000, 60.0)
        
        self.assertEqual(len(self.model.calibration_points), 5)
        self.assertEqual(point.x, 500)
        self.assertEqual(point.y, 500)
        self.assertEqual(point.raw_value, 50000)
        self.assertEqual(point.reference_temp, 60.0)
        self.assertFalse(self.model.is_calibrated)  # Should reset calibration state
    
    def test_remove_calibration_point(self):
        """Test removing calibration points."""
        # Check initial count
        self.assertEqual(len(self.model.calibration_points), 4)
        
        # Remove a point
        result = self.model.remove_calibration_point(1)
        
        self.assertTrue(result)
        self.assertEqual(len(self.model.calibration_points), 3)
        self.assertEqual(self.model.calibration_points[1].raw_value, 30000)  # Third point moved up
        
        # Try to remove a non-existent point
        result = self.model.remove_calibration_point(10)
        
        self.assertFalse(result)
        self.assertEqual(len(self.model.calibration_points), 3)
    
    def test_clear_calibration_points(self):
        """Test clearing all calibration points."""
        # Check initial count
        self.assertEqual(len(self.model.calibration_points), 4)
        
        # Clear points
        self.model.clear_calibration_points()
        
        self.assertEqual(len(self.model.calibration_points), 0)
        self.assertFalse(self.model.is_calibrated)
        self.assertIsNone(self.model.calibration_timestamp)
    
    def test_calibrate(self):
        """Test calibration process."""
        # Should not be calibrated initially
        self.assertFalse(self.model.is_calibrated)
        
        # Run calibration
        result = self.model.calibrate()
        
        self.assertTrue(result)
        self.assertTrue(self.model.is_calibrated)
        self.assertIsNotNone(self.model.calibration_timestamp)
        self.assertNotEqual(self.model.model_params, {})
        self.assertNotEqual(self.model.calibration_stats, {})
    
    def test_calibrate_insufficient_points(self):
        """Test calibration with insufficient points."""
        # Clear points
        self.model.clear_calibration_points()
        
        # Add just one point
        self.model.add_calibration_point(100, 100, 10000, 20.0)
        
        # Try to calibrate
        result = self.model.calibrate()
        
        self.assertFalse(result)
        self.assertFalse(self.model.is_calibrated)
    
    def test_raw_to_temperature(self):
        """Test converting raw values to temperature."""
        # Calibrate first
        self.model.calibrate()
        
        # Test with a raw value in the calibrated range
        temp = self.model.raw_to_temperature(25000)
        
        self.assertIsInstance(temp, float)
        self.assertTrue(20.0 <= temp <= 50.0)  # Should be in our calibrated range
        
        # Test with an array of raw values
        raw_values = np.array([10000, 20000, 30000, 40000])
        temps = self.model.raw_to_temperature(raw_values)
        
        self.assertIsInstance(temps, np.ndarray)
        self.assertEqual(temps.shape, (4,))
    
    def test_temperature_to_raw(self):
        """Test converting temperature to raw values."""
        # Calibrate first
        self.model.calibrate()
        
        # Test with a temperature in the calibrated range
        raw = self.model.temperature_to_raw(35.0)
        
        self.assertIsInstance(raw, int)
        self.assertTrue(10000 <= raw <= 40000)  # Should be in our calibrated range
        
        # Test with an array of temperatures
        temps = np.array([20.0, 30.0, 40.0, 50.0])
        raw_values = self.model.temperature_to_raw(temps)
        
        self.assertIsInstance(raw_values, np.ndarray)
        self.assertEqual(raw_values.shape, (4,))
    
    def test_save_and_load(self):
        """Test saving and loading calibration model."""
        # Calibrate first
        self.model.calibrate()
        
        # Save to a temporary file
        with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as tf:
            filepath = tf.name
        
        try:
            # Save to file
            result = self.model.save_to_file(filepath)
            self.assertTrue(result)
            
            # Create a new model
            new_model = CalibrationModel()
            
            # Load from file
            result = new_model.load_from_file(filepath)
            self.assertTrue(result)
            
            # Compare models
            self.assertEqual(len(new_model.calibration_points), len(self.model.calibration_points))
            self.assertEqual(new_model.model_type, self.model.model_type)
            self.assertEqual(new_model.polynomial_degree, self.model.polynomial_degree)
            self.assertEqual(new_model.is_calibrated, self.model.is_calibrated)
            
        finally:
            # Clean up
            if os.path.exists(filepath):
                os.unlink(filepath)


class TestRegression(unittest.TestCase):
    """Test cases for the regression module."""
    
    def test_create_calibration_function_polynomial(self):
        """Test creating a polynomial calibration function."""
        # Create test data
        raw_values = np.array([10000, 20000, 30000, 40000, 50000])
        ref_temps = np.array([20.0, 30.0, 40.0, 50.0, 60.0])
        
        # Create calibration function
        model_params, func = create_calibration_function(
            raw_values, ref_temps, model_type="polynomial", degree=2
        )
        
        # Check model parameters
        self.assertIn("coefficients", model_params)
        self.assertEqual(len(model_params["coefficients"]), 3)  # degree 2 = 3 coefficients
        
        # Check function works
        pred_temps = func(raw_values)
        self.assertIsInstance(pred_temps, np.ndarray)
        self.assertEqual(pred_temps.shape, (5,))
        
        # Check prediction accuracy
        errors = np.abs(pred_temps - ref_temps)
        self.assertTrue(np.max(errors) < 1.0)  # Should fit well
    
    def test_evaluate_calibration(self):
        """Test evaluating a calibration model."""
        # Create test data
        raw_values = np.array([10000, 20000, 30000, 40000, 50000])
        ref_temps = np.array([20.0, 30.0, 40.0, 50.0, 60.0])
        
        # Create linear calibration function
        def func(x):
            return 20.0 + (x - 10000) * 0.001
        
        # Evaluate the calibration
        stats = evaluate_calibration(raw_values, ref_temps, func)
        
        # Check statistics
        self.assertIn("rmse", stats)
        self.assertIn("r_squared", stats)
        self.assertIn("mean_error", stats)
        self.assertIn("mean_abs_error", stats)


if __name__ == '__main__':
    unittest.main()