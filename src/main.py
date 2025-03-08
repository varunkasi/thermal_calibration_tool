#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main entry point for the Thermal Camera Calibration Tool.

This module initializes the application, connects to the FLIR Boson+ camera,
and launches the GUI for thermal camera calibration.
"""

import os
import sys
import signal
from typing import Optional

from PyQt5.QtWidgets import QApplication

from src.utils.logger import setup_logger
from src.utils.config import Config
from src.camera.boson_camera import BosonCamera
from src.gui.main_window import MainWindow
from src.processing.thermal_processor import ThermalProcessor
from src.calibration.calibration_model import CalibrationModel


logger = setup_logger()


def signal_handler(sig, frame):
    """Handle termination signals gracefully."""
    logger.info("Received termination signal. Shutting down...")
    if QApplication.instance():
        QApplication.instance().quit()
    sys.exit(0)


def main():
    """Initialize and run the Thermal Camera Calibration Tool."""
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Load configuration
    config_path = os.environ.get("THERMAL_CONFIG_PATH", "config.yaml")
    config = Config(config_path)
    logger.info(f"Loaded configuration from {config_path}")

    # Create QApplication instance
    app = QApplication(sys.argv)
    app.setApplicationName("Thermal Camera Calibration Tool")
    app.setQuitOnLastWindowClosed(True)

    # Initialize camera
    camera: Optional[BosonCamera] = None
    try:
        logger.info("Initializing FLIR Boson+ camera...")
        camera = BosonCamera(config)
    except Exception as e:
        logger.error(f"Failed to initialize camera: {e}")
        # If in development mode, allow running without camera for UI development
        if not os.environ.get("THERMAL_DEV_MODE"):
            sys.exit(1)

    # Initialize thermal processor
    thermal_processor = ThermalProcessor(config)

    # Initialize calibration model
    calibration_model = CalibrationModel()

    # Create and show main window
    main_window = MainWindow(
        camera=camera,
        thermal_processor=thermal_processor,
        calibration_model=calibration_model,
        config=config,
    )
    main_window.show()

    # Start the event loop
    exit_code = app.exec_()

    # Clean up
    if camera:
        logger.info("Closing camera connection...")
        camera.close()

    logger.info("Application shutdown complete")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())