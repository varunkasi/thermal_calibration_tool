#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Configuration management for the Thermal Camera Calibration Tool.

This module provides a configuration management system with default values,
loading from YAML files, and runtime configuration updates.
"""

import os
import yaml
from typing import Any, Dict, Optional, List, Union
from loguru import logger


class Config:
    """Configuration manager with dot notation access and defaults."""
    
    # Default configuration values
    DEFAULT_CONFIG = {
        # Camera settings
        "camera": {
            "auto_connect": True,
            "max_connection_attempts": 3,
            "connection_retry_delay": 2,
            "device_id": 0,
        },
        
        # GUI settings
        "gui": {
            "update_interval_ms": 100,
            "thermal_view": {
                "show_grid": False,
                "radiometric_mode": False,
                "current_palette": "Inferno",
                "palettes": ["Inferno", "Jet", "Viridis", "Grayscale", "IronBow"],
                "zoom_step": 0.1,
                "max_zoom": 3.0,
                "min_zoom": 0.1
            },
            "window": {
                "title": "Thermal Camera Calibration Tool",
                "width": 1200,
                "height": 800,
                "maximized": False,
                "remember_position": True
            }
        },
        
        # Processing settings
        "processing": {
            "auto_range": True,
            "min_value": 0,
            "max_value": 65535,
            "gaussian_filter": {
                "enabled": False,
                "sigma": 1.0
            }
        },
        
        # Calibration settings
        "calibration": {
            "default_model_type": "polynomial",
            "default_polynomial_degree": 2,
            "min_points": 3,
            "min_temperature": -20.0,
            "max_temperature": 120.0
        },
        
        # Data storage settings
        "data": {
            "base_directory": "~/thermal_calibration_data",
            "calibration_file": "calibration.json",
            "autosave": True,
            "autosave_interval_sec": 300  # 5 minutes
        },
        
        # Logging settings
        "logging": {
            "level": "INFO",
            "file": "thermal_calibration.log",
            "max_size": 10485760,  # 10 MB
            "backup_count": 3,
            "console": True
        }
    }
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize the configuration.
        
        Args:
            config_file (Optional[str], optional): Path to configuration file.
                Defaults to None.
        """
        # Initialize with default configuration
        self.config_data = dict(self.DEFAULT_CONFIG)
        
        # Load configuration from file if provided
        if config_file:
            self.load_from_file(config_file)
    
    def load_from_file(self, config_file: str) -> bool:
        """
        Load configuration from a YAML file.
        
        Args:
            config_file (str): Path to configuration file.
        
        Returns:
            bool: True if configuration was loaded successfully, False otherwise.
        """
        try:
            # Expand user directory if needed
            config_file = os.path.expanduser(config_file)
            
            # Check if file exists
            if not os.path.isfile(config_file):
                logger.warning(f"Configuration file not found: {config_file}")
                return False
            
            # Load YAML file
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
            
            # Merge with default configuration
            if config_data:
                self._merge_config(self.config_data, config_data)
                logger.info(f"Loaded configuration from {config_file}")
                return True
            else:
                logger.warning(f"Empty or invalid configuration file: {config_file}")
                return False
                
        except Exception as e:
            logger.error(f"Error loading configuration from {config_file}: {e}")
            return False
    
    def save_to_file(self, config_file: str) -> bool:
        """
        Save configuration to a YAML file.
        
        Args:
            config_file (str): Path to configuration file.
        
        Returns:
            bool: True if configuration was saved successfully, False otherwise.
        """
        try:
            # Expand user directory if needed
            config_file = os.path.expanduser(config_file)
            
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(os.path.abspath(config_file)), exist_ok=True)
            
            # Save YAML file
            with open(config_file, 'w') as f:
                yaml.dump(self.config_data, f, default_flow_style=False)
            
            logger.info(f"Saved configuration to {config_file}")
            return True
                
        except Exception as e:
            logger.error(f"Error saving configuration to {config_file}: {e}")
            return False
    
    def _merge_config(self, base: Dict[str, Any], overlay: Dict[str, Any]) -> None:
        """
        Recursively merge overlay configuration into base configuration.
        
        Args:
            base (Dict[str, Any]): Base configuration.
            overlay (Dict[str, Any]): Overlay configuration.
        """
        for key, value in overlay.items():
            if (
                key in base and 
                isinstance(base[key], dict) and 
                isinstance(value, dict)
            ):
                # Recursively merge dictionaries
                self._merge_config(base[key], value)
            else:
                # Otherwise, overwrite the value
                base[key] = value
    
    def get(self, key_path: str, default: Any = None) -> Any:
        """
        Get a configuration value by its dot-separated path.
        
        Args:
            key_path (str): Dot-separated path to the configuration value.
            default (Any, optional): Default value if not found. Defaults to None.
        
        Returns:
            Any: Configuration value.
        """
        # Split the key path into parts
        keys = key_path.split('.')
        
        # Start at the root of the configuration data
        value = self.config_data
        
        # Traverse the nested dictionaries
        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default
    
    def set(self, key_path: str, value: Any) -> None:
        """
        Set a configuration value by its dot-separated path.
        
        Args:
            key_path (str): Dot-separated path to the configuration value.
            value (Any): Value to set.
        """
        # Split the key path into parts
        keys = key_path.split('.')
        
        # Start at the root of the configuration data
        config = self.config_data
        
        # Traverse the nested dictionaries until the second-to-last key
        for key in keys[:-1]:
            if key not in config or not isinstance(config[key], dict):
                config[key] = {}
            config = config[key]
        
        # Set the value at the last key
        config[keys[-1]] = value
    
    def get_all(self) -> Dict[str, Any]:
        """
        Get the entire configuration.
        
        Returns:
            Dict[str, Any]: Configuration data.
        """
        return dict(self.config_data)
    
    def update(self, config_data: Dict[str, Any]) -> None:
        """
        Update the configuration with new values.
        
        Args:
            config_data (Dict[str, Any]): New configuration data.
        """
        self._merge_config(self.config_data, config_data)
    
    def reset_to_defaults(self) -> None:
        """Reset the configuration to default values."""
        self.config_data = dict(self.DEFAULT_CONFIG)


# Create a global instance
config = Config()