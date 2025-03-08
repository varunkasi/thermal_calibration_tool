#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration model for the Thermal Camera Calibration Tool.

This module manages the calibration data collection and model creation
for mapping raw thermal camera values to temperature values.
"""

import os
import time
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple, Any, Union
import json

import numpy as np
from loguru import logger
from dataclasses_json import dataclass_json

from src.calibration.regression import create_calibration_function, evaluate_calibration


@dataclass_json
@dataclass
class CalibrationPoint:
    """A single calibration point with raw and reference values."""
    
    x: int  # X coordinate in the image
    y: int  # Y coordinate in the image
    raw_value: int  # Raw thermal camera value
    reference_temp: float  # Reference temperature in Celsius
    timestamp: float = field(default_factory=time.time)  # When the point was captured
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert calibration point to dictionary."""
        return {
            "x": self.x,
            "y": self.y, 
            "raw_value": self.raw_value,
            "reference_temp": self.reference_temp,
            "timestamp": self.timestamp
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CalibrationPoint':
        """Create calibration point from dictionary."""
        return cls(
            x=data["x"],
            y=data["y"],
            raw_value=data["raw_value"],
            reference_temp=data["reference_temp"],
            timestamp=data.get("timestamp", time.time())
        )


@dataclass_json
@dataclass
class CalibrationModel:
    """Model for calibrating thermal camera raw values to temperature values."""
    
    calibration_points: List[CalibrationPoint] = field(default_factory=list)
    model_type: str = "polynomial"  # Type of regression model (polynomial, exponential, etc.)
    model_params: Dict[str, Any] = field(default_factory=dict)  # Model parameters
    polynomial_degree: int = 2  # Degree of polynomial if model_type is polynomial
    is_calibrated: bool = False  # Flag to indicate if model is calibrated
    calibration_timestamp: Optional[float] = None  # When the model was last calibrated
    calibration_stats: Dict[str, float] = field(default_factory=dict)  # Calibration quality stats
    temperature_range: Tuple[float, float] = (-20.0, 120.0)  # Min/max temperature in Celsius
    
    def add_calibration_point(
        self, x: int, y: int, raw_value: int, reference_temp: float
    ) -> CalibrationPoint:
        """
        Add a new calibration point.
        
        Args:
            x (int): X coordinate of the pixel.
            y (int): Y coordinate of the pixel.
            raw_value (int): Raw thermal value from the camera.
            reference_temp (float): Reference temperature in Celsius.
            
        Returns:
            CalibrationPoint: The newly created calibration point.
        """
        # Create a new calibration point
        point = CalibrationPoint(
            x=x,
            y=y,
            raw_value=raw_value,
            reference_temp=reference_temp
        )
        
        # Add to list of points
        self.calibration_points.append(point)
        
        # Set calibration status to false since we have a new point
        self.is_calibrated = False
        
        logger.info(f"Added calibration point: raw={raw_value}, temp={reference_temp}°C at ({x}, {y})")
        return point
    
    def remove_calibration_point(self, index: int) -> bool:
        """
        Remove a calibration point by index.
        
        Args:
            index (int): Index of the calibration point to remove.
            
        Returns:
            bool: True if the point was successfully removed.
        """
        if 0 <= index < len(self.calibration_points):
            point = self.calibration_points.pop(index)
            self.is_calibrated = False
            logger.info(f"Removed calibration point {index}: {point}")
            return True
        else:
            logger.warning(f"Invalid calibration point index: {index}")
            return False
    
    def clear_calibration_points(self) -> None:
        """Clear all calibration points and reset calibration."""
        self.calibration_points.clear()
        self.is_calibrated = False
        self.calibration_timestamp = None
        self.model_params.clear()
        self.calibration_stats.clear()
        logger.info("Cleared all calibration points and reset calibration model")
    
    def calibrate(self) -> bool:
        """
        Perform calibration using the collected points.
        
        Returns:
            bool: True if calibration was successful.
        """
        # Check if we have enough points for calibration
        if len(self.calibration_points) < 3:
            logger.warning("Not enough calibration points. Need at least 3 points.")
            return False
        
        try:
            # Extract raw values and reference temperatures
            raw_values = np.array([point.raw_value for point in self.calibration_points])
            ref_temps = np.array([point.reference_temp for point in self.calibration_points])
            
            # Create calibration function (implemented in regression.py)
            model_params, model_func = create_calibration_function(
                raw_values, ref_temps, model_type=self.model_type, 
                degree=self.polynomial_degree
            )
            
            # Evaluate calibration quality
            stats = evaluate_calibration(raw_values, ref_temps, model_func)
            
            # Update model parameters and stats
            self.model_params = model_params
            self.calibration_stats = stats
            self.is_calibrated = True
            self.calibration_timestamp = time.time()
            
            # Update temperature range based on calibration points
            min_temp = min(ref_temps)
            max_temp = max(ref_temps)
            # Add some margin to the range
            margin = (max_temp - min_temp) * 0.1
            self.temperature_range = (min_temp - margin, max_temp + margin)
            
            logger.success("Calibration successful")
            logger.info(f"Calibration stats: {stats}")
            
            return True
            
        except Exception as e:
            logger.error(f"Error during calibration: {e}")
            return False
    
    def raw_to_temperature(self, raw_value: Union[int, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Convert raw thermal value to temperature using the calibration model.
        
        Args:
            raw_value (Union[int, np.ndarray]): Raw thermal value or array of values.
            
        Returns:
            Union[float, np.ndarray]: Temperature in Celsius or array of temperatures.
        """
        if not self.is_calibrated:
            logger.warning("Model not calibrated. Using linear approximation.")
            # Simple linear approximation based on temperature range
            min_val, max_val = 0, 65535  # 16-bit values
            min_temp, max_temp = self.temperature_range
            
            # Linear mapping
            temp = min_temp + (raw_value - min_val) * (max_temp - min_temp) / (max_val - min_val)
            return temp
        
        try:
            # Apply calibration model based on model type
            if self.model_type == "polynomial":
                # Get polynomial coefficients
                coeffs = self.model_params.get("coefficients", [0, 1])
                
                # Apply polynomial model: temp = a0 + a1*x + a2*x^2 + ...
                if isinstance(raw_value, np.ndarray):
                    temp = np.zeros_like(raw_value, dtype=float)
                    for i, coef in enumerate(coeffs):
                        temp += coef * (raw_value ** i)
                else:
                    temp = sum(coef * (raw_value ** i) for i, coef in enumerate(coeffs))
                
                return temp
                
            elif self.model_type == "exponential":
                # Get exponential model parameters
                a = self.model_params.get("a", 0)
                b = self.model_params.get("b", 1)
                c = self.model_params.get("c", 0)
                
                # Apply exponential model: temp = a * exp(b * x) + c
                return a * np.exp(b * raw_value) + c
                
            else:
                logger.warning(f"Unknown model type: {self.model_type}. Using linear approximation.")
                # Fall back to linear model
                min_val, max_val = 0, 65535  # 16-bit values
                min_temp, max_temp = self.temperature_range
                return min_temp + (raw_value - min_val) * (max_temp - min_temp) / (max_val - min_val)
                
        except Exception as e:
            logger.error(f"Error converting raw value to temperature: {e}")
            # Return a reasonable default
            return 20.0  # Room temperature as fallback
    
    def temperature_to_raw(self, temperature: Union[float, np.ndarray]) -> Union[int, np.ndarray]:
        """
        Convert temperature to raw thermal value using inverse of calibration model.
        
        Args:
            temperature (Union[float, np.ndarray]): Temperature in Celsius or array of temperatures.
            
        Returns:
            Union[int, np.ndarray]: Raw thermal value or array of values.
        """
        if not self.is_calibrated:
            logger.warning("Model not calibrated. Using linear approximation.")
            # Simple linear approximation based on temperature range
            min_val, max_val = 0, 65535  # 16-bit values
            min_temp, max_temp = self.temperature_range
            
            # Linear mapping (inverse)
            raw = min_val + (temperature - min_temp) * (max_val - min_val) / (max_temp - min_temp)
            
            if isinstance(temperature, np.ndarray):
                return raw.astype(np.uint16)
            else:
                return int(raw)
        
        try:
            # For polynomial models, we can't easily solve for the inverse.
            # So, we'll use a lookup table approach or numerical methods.
            # For simplicity, here we'll use a linear approximation based on our current model.
            
            # Create a range of raw values
            raw_values = np.linspace(0, 65535, 1000)
            
            # Convert to temperatures using our model
            temperatures = self.raw_to_temperature(raw_values)
            
            # Now for the given temperature, find the closest match
            if isinstance(temperature, np.ndarray):
                result = np.zeros_like(temperature, dtype=np.uint16)
                for i, temp in enumerate(temperature):
                    idx = np.argmin(np.abs(temperatures - temp))
                    result[i] = raw_values[idx]
                return result
            else:
                idx = np.argmin(np.abs(temperatures - temperature))
                return int(raw_values[idx])
                
        except Exception as e:
            logger.error(f"Error converting temperature to raw value: {e}")
            # Return a reasonable default
            return 32768  # Middle of 16-bit range as fallback
    
    def save_to_file(self, filepath: str) -> bool:
        """
        Save calibration model to file.
        
        Args:
            filepath (str): Path to save the model.
            
        Returns:
            bool: True if model was successfully saved.
        """
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
            
            # Convert model to JSON
            model_data = {
                "calibration_points": [point.to_dict() for point in self.calibration_points],
                "model_type": self.model_type,
                "model_params": self.model_params,
                "polynomial_degree": self.polynomial_degree,
                "is_calibrated": self.is_calibrated,
                "calibration_timestamp": self.calibration_timestamp,
                "calibration_stats": self.calibration_stats,
                "temperature_range": self.temperature_range,
                "version": "1.0.0",  # Add version for future compatibility
                "created_at": time.time()
            }
            
            # Write to file
            with open(filepath, 'w') as f:
                json.dump(model_data, f, indent=2)
                
            logger.success(f"Saved calibration model to {filepath}")
            return True
            
        except Exception as e:
            logger.error(f"Error saving calibration model: {e}")
            return False
    
    def load_from_file(self, filepath: str) -> bool:
        """
        Load calibration model from file.
        
        Args:
            filepath (str): Path to load the model from.
            
        Returns:
            bool: True if model was successfully loaded.
        """
        try:
            # Read from file
            with open(filepath, 'r') as f:
                model_data = json.load(f)
                
            # Load calibration points
            self.calibration_points = [
                CalibrationPoint.from_dict(point_data) 
                for point_data in model_data.get("calibration_points", [])
            ]
            
            # Load model parameters
            self.model_type = model_data.get("model_type", "polynomial")
            self.model_params = model_data.get("model_params", {})
            self.polynomial_degree = model_data.get("polynomial_degree", 2)
            self.is_calibrated = model_data.get("is_calibrated", False)
            self.calibration_timestamp = model_data.get("calibration_timestamp")
            self.calibration_stats = model_data.get("calibration_stats", {})
            self.temperature_range = model_data.get("temperature_range", (-20.0, 120.0))
            
            logger.success(f"Loaded calibration model from {filepath}")
            return True
            
        except Exception as e:
            logger.error(f"Error loading calibration model: {e}")
            return False
    
    def get_recommended_calibration_point(self) -> Optional[Tuple[float, str]]:
        """
        Get a recommended temperature for the next calibration point.
        
        Returns:
            Optional[Tuple[float, str]]: Recommended temperature and reason, or None if no recommendation.
        """
        if not self.calibration_points:
            return 20.0, "Start with room temperature reference"
        
        # Get current range of reference temperatures
        temps = [point.reference_temp for point in self.calibration_points]
        min_temp, max_temp = min(temps), max(temps)
        
        # If range is small, recommend expanding it
        if max_temp - min_temp < 30:
            if min_temp > 0:
                return min_temp - 10, "Add a lower temperature point to expand range"
            else:
                return max_temp + 10, "Add a higher temperature point to expand range"
        
        # Find largest gap in the temperature range
        temps.sort()
        max_gap = 0
        gap_temp = None
        
        for i in range(len(temps) - 1):
            gap = temps[i+1] - temps[i]
            if gap > max_gap:
                max_gap = gap
                gap_temp = (temps[i] + temps[i+1]) / 2
        
        if max_gap > 10:
            return gap_temp, f"Fill gap between {temps[i]:.1f}°C and {temps[i+1]:.1f}°C"
        
        # If we have a good range without large gaps, suggest adding redundant points
        # for better statistical significance
        if len(self.calibration_points) < 10:
            # Get least represented temperature region
            regions = np.linspace(min_temp, max_temp, 5)
            counts = [0] * 4
            
            for temp in temps:
                for i in range(4):
                    if regions[i] <= temp < regions[i+1]:
                        counts[i] += 1
                        break
            
            min_count_region = counts.index(min(counts))
            target_temp = (regions[min_count_region] + regions[min_count_region+1]) / 2
            
            return target_temp, f"Add point around {target_temp:.1f}°C for better coverage"
        
        return None
    
    def get_calibration_quality_indicator(self) -> Tuple[float, str]:
        """
        Get a quality indicator for the current calibration.
        
        Returns:
            Tuple[float, str]: Quality score (0-1) and description.
        """
        if not self.is_calibrated:
            return 0.0, "Not calibrated"
        
        # Get stats
        r_squared = self.calibration_stats.get("r_squared", 0)
        rmse = self.calibration_stats.get("rmse", float('inf'))
        num_points = len(self.calibration_points)
        
        # Calculate quality score based on various factors
        r_squared_score = min(r_squared, 1.0)  # Should be between 0 and 1
        
        # RMSE score (lower is better)
        # Normalize RMSE to a 0-1 scale, assuming good RMSE is < 1°C, bad is > 5°C
        rmse_score = max(0, min(1, 1 - (rmse - 1) / 4))
        
        # Number of points score (more is better, up to about 15 points)
        points_score = min(num_points / 15, 1.0)
        
        # Temperature range score
        temps = [point.reference_temp for point in self.calibration_points]
        min_temp, max_temp = min(temps), max(temps)
        range_score = min((max_temp - min_temp) / 50, 1.0)  # Good range is > 50°C
        
        # Weighted overall score
        quality_score = (0.4 * r_squared_score + 
                         0.3 * rmse_score + 
                         0.15 * points_score + 
                         0.15 * range_score)
        
        # Description based on score
        if quality_score > 0.8:
            description = "Excellent calibration"
        elif quality_score > 0.6:
            description = "Good calibration"
        elif quality_score > 0.4:
            description = "Fair calibration"
        elif quality_score > 0.2:
            description = "Poor calibration"
        else:
            description = "Very poor calibration"
        
        return quality_score, description