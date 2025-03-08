#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logging configuration for the Thermal Camera Calibration Tool.

This module provides a standardized logging setup using the loguru library
for consistent logging across the application.
"""

import os
import sys
from typing import Optional, Dict, Any

from loguru import logger as loguru_logger


def setup_logger(config: Optional[Dict[str, Any]] = None) -> Any:
    """
    Set up and configure the logger.
    
    Args:
        config (Optional[Dict[str, Any]], optional): Logger configuration.
            Defaults to None.
    
    Returns:
        Any: Configured logger instance.
    """
    # Remove default loguru handler
    loguru_logger.remove()
    
    # Default configuration
    default_config = {
        "level": "INFO",
        "file": "thermal_calibration.log",
        "max_size": 10485760,  # 10 MB
        "backup_count": 3,
        "console": True
    }
    
    # Merge with provided configuration
    log_config = default_config
    if config:
        log_config.update(config)
    
    # Add console handler if enabled
    if log_config["console"]:
        # Console format with colors
        loguru_logger.add(
            sys.stderr,
            level=log_config["level"],
            format="<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | "
                   "<level>{level: <8}</level> | "
                   "<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - "
                   "<level>{message}</level>"
        )
    
    # Add file handler
    if log_config["file"]:
        # Create log directory if it doesn't exist
        log_dir = os.path.dirname(os.path.abspath(log_config["file"]))
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        
        # File format (no colors)
        loguru_logger.add(
            log_config["file"],
            level=log_config["level"],
            format="{time:YYYY-MM-DD HH:mm:ss.SSS} | {level: <8} | "
                  "{name}:{function}:{line} - {message}",
            rotation=log_config["max_size"],
            retention=log_config["backup_count"]
        )
    
    # Enable success level
    loguru_logger.level("SUCCESS", no=25, color="<green>")
    
    return loguru_logger


def get_logger(name: Optional[str] = None) -> Any:
    """
    Get a logger instance, optionally with a specific name.
    
    Args:
        name (Optional[str], optional): Logger name. Defaults to None.
    
    Returns:
        Any: Logger instance.
    """
    if name:
        # To truly get a named logger in loguru, we need to patch the context
        # and return the same logger instance for consistent behavior
        logger_instance = loguru_logger.bind(name=name)
        return logger_instance
    else:
        return loguru_logger


# Initialize logger with default configuration
logger = setup_logger()