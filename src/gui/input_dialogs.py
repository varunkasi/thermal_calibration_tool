#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Input dialogs for the Thermal Camera Calibration Tool.

This module provides specialized input dialogs for the application,
such as the temperature input dialog for calibration points.
"""

from typing import Optional

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QDoubleSpinBox, QFormLayout, QGroupBox, QDialogButtonBox
)
from PyQt5.QtCore import Qt, QTimer


class TemperatureInputDialog(QDialog):
    """Dialog for entering reference temperature values."""
    
    def __init__(
        self,
        x: int,
        y: int,
        raw_value: int,
        recommended_temp: Optional[float] = None,
        parent=None
    ):
        """
        Initialize the temperature input dialog.
        
        Args:
            x (int): X coordinate of the selected pixel.
            y (int): Y coordinate of the selected pixel.
            raw_value (int): Raw thermal value of the selected pixel.
            recommended_temp (Optional[float], optional): Recommended temperature value.
                Defaults to None.
            parent ([type], optional): Parent widget. Defaults to None.
        """
        super().__init__(parent)
        
        self.x = x
        self.y = y
        self.raw_value = raw_value
        self.recommended_temp = recommended_temp
        
        # Set up dialog
        self.setWindowTitle("Enter Reference Temperature")
        self.setModal(True)
        self.setMinimumWidth(350)
        
        # Set up layout
        self._init_ui()
    
    def _init_ui(self):
        """Initialize the user interface components."""
        # Main layout
        main_layout = QVBoxLayout(self)
        
        # Create info group
        info_group = QGroupBox("Selected Pixel")
        info_layout = QFormLayout(info_group)
        info_layout.addRow("Coordinates:", QLabel(f"({self.x}, {self.y})"))
        info_layout.addRow("Raw Value:", QLabel(f"{self.raw_value}"))
        main_layout.addWidget(info_group)
        
        # Create temperature input group
        temp_group = QGroupBox("Reference Temperature")
        temp_layout = QVBoxLayout(temp_group)
        
        # Instructions
        instructions = QLabel(
            "Enter the temperature reading from your thermal gun for this point."
        )
        instructions.setWordWrap(True)
        temp_layout.addWidget(instructions)
        
        # Temperature input
        input_layout = QFormLayout()
        self.temp_spin = QDoubleSpinBox()
        self.temp_spin.setRange(-50.0, 500.0)
        self.temp_spin.setDecimals(1)
        self.temp_spin.setSuffix(" °C")
        self.temp_spin.setSingleStep(0.1)
        
        # Set recommended value if available
        if self.recommended_temp is not None:
            self.temp_spin.setValue(self.recommended_temp)
        else:
            self.temp_spin.setValue(20.0)  # Default to room temperature
            
        input_layout.addRow("Temperature:", self.temp_spin)
        temp_layout.addLayout(input_layout)
        
        # Recommendation if available
        if self.recommended_temp is not None:
            recommendation = QLabel(
                f"Recommended: {self.recommended_temp:.1f}°C based on your current calibration points."
            )
            recommendation.setStyleSheet("color: green;")
            temp_layout.addWidget(recommendation)
        
        # Add reminder about proper measurement
        reminder = QLabel(
            "For accurate calibration, ensure your thermal gun is properly aligned with "
            "the selected pixel on the thermal camera."
        )
        reminder.setWordWrap(True)
        reminder.setStyleSheet("color: gray; font-style: italic;")
        temp_layout.addWidget(reminder)
        
        main_layout.addWidget(temp_group)
        
        # Button box
        button_box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        main_layout.addWidget(button_box)
    
    def get_temperature(self) -> float:
        """
        Get the entered temperature value.
        
        Returns:
            float: Temperature in Celsius.
        """
        return self.temp_spin.value()


class CalibrationDetailsDialog(QDialog):
    """Dialog showing detailed calibration results."""
    
    def __init__(
        self,
        calibration_stats,
        model_params,
        temperature_range,
        parent=None
    ):
        """
        Initialize the calibration details dialog.
        
        Args:
            calibration_stats: Statistics from the calibration.
            model_params: Parameters of the calibration model.
            temperature_range: Min and max temperature range of the calibration.
            parent: Parent widget. Defaults to None.
        """
        super().__init__(parent)
        
        self.stats = calibration_stats
        self.params = model_params
        self.temp_range = temperature_range
        
        # Set up dialog
        self.setWindowTitle("Calibration Details")
        self.setModal(True)
        self.setMinimumWidth(400)
        
        # Set up layout
        self._init_ui()
    
    def _init_ui(self):
        """Initialize the user interface components."""
        # Main layout
        main_layout = QVBoxLayout(self)
        
        # Statistics group
        stats_group = QGroupBox("Calibration Statistics")
        stats_layout = QFormLayout(stats_group)
        
        # Add statistics
        for stat_name, stat_value in self.stats.items():
            # Format the statistic name
            formatted_name = stat_name.replace('_', ' ').capitalize()
            
            # Format the value
            if isinstance(stat_value, float):
                formatted_value = f"{stat_value:.4f}"
            else:
                formatted_value = str(stat_value)
            
            # Add row
            stats_layout.addRow(f"{formatted_name}:", QLabel(formatted_value))
        
        main_layout.addWidget(stats_group)
        
        # Model parameters group
        params_group = QGroupBox("Model Parameters")
        params_layout = QFormLayout(params_group)
        
        # Add parameters
        for param_name, param_value in self.params.items():
            # Format the parameter name
            formatted_name = param_name.replace('_', ' ').capitalize()
            
            # Format the value
            if isinstance(param_value, list):
                # For coefficients, show each one
                formatted_value = ", ".join([f"{v:.4f}" for v in param_value])
            elif isinstance(param_value, float):
                formatted_value = f"{param_value:.4f}"
            else:
                formatted_value = str(param_value)
            
            # Add row
            params_layout.addRow(f"{formatted_name}:", QLabel(formatted_value))
        
        main_layout.addWidget(params_group)
        
        # Temperature range
        range_group = QGroupBox("Temperature Range")
        range_layout = QFormLayout(range_group)
        min_temp, max_temp = self.temp_range
        range_layout.addRow("Minimum temperature:", QLabel(f"{min_temp:.1f}°C"))
        range_layout.addRow("Maximum temperature:", QLabel(f"{max_temp:.1f}°C"))
        main_layout.addWidget(range_group)
        
        # Button box
        button_box = QDialogButtonBox(QDialogButtonBox.Ok)
        button_box.accepted.connect(self.accept)
        main_layout.addWidget(button_box)


class SplashScreen(QDialog):
    """Splash screen dialog showing application information during startup."""
    
    def __init__(self, parent=None):
        """
        Initialize the splash screen dialog.
        
        Args:
            parent: Parent widget. Defaults to None.
        """
        super().__init__(parent, Qt.SplashScreen | Qt.WindowStaysOnTopHint)
        
        # Set up dialog
        self.setWindowTitle("Thermal Camera Calibration Tool")
        self.setModal(True)
        self.setFixedSize(500, 300)
        
        # Remove window frame
        self.setWindowFlags(Qt.SplashScreen | Qt.WindowStaysOnTopHint)
        
        # Set up layout
        self._init_ui()
        
        # Auto-close timer
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.accept)
        self.timer.start(3000)  # Close after 3 seconds
    
    def _init_ui(self):
        """Initialize the user interface components."""
        # Main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # App title
        title = QLabel("Thermal Camera Calibration Tool")
        title.setStyleSheet("font-size: 24px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # App version
        version = QLabel("Version 1.0.0")
        version.setStyleSheet("font-size: 16px;")
        version.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(version)
        
        # Spacer
        main_layout.addSpacing(20)
        
        # Loading message
        loading = QLabel("Loading application...")
        loading.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(loading)
        
        # Copyright notice
        copyright_notice = QLabel("© 2025 Thermal Tools Inc.")
        copyright_notice.setAlignment(Qt.AlignCenter)
        copyright_notice.setStyleSheet("font-size: 12px; color: gray;")
        main_layout.addWidget(copyright_notice, alignment=Qt.AlignBottom)