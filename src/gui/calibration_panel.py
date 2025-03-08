#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration panel for the Thermal Camera Calibration Tool.

This module implements the UI panel for managing calibration points,
entering reference temperatures, and running the calibration process.
"""

from typing import Optional, Tuple, Dict, Any, List

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QMessageBox, QCheckBox,
    QComboBox, QSpinBox, QDoubleSpinBox, QFormLayout, QFrame, QSplitter
)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer

from src.utils.logger import get_logger
from src.utils.config import Config
from src.calibration.calibration_model import CalibrationModel
from src.gui.input_dialogs import TemperatureInputDialog


logger = get_logger(__name__)


class CalibrationPanel(QWidget):
    """Panel for managing calibration points and running calibration."""
    
    # Signals
    calibration_updated = pyqtSignal()
    radiometric_mode_toggled = pyqtSignal(bool)
    
    def __init__(
        self,
        calibration_model: CalibrationModel,
        config: Config,
        parent: Optional[QWidget] = None
    ):
        """
        Initialize the calibration panel.
        
        Args:
            calibration_model (CalibrationModel): Calibration model instance.
            config (Config): Configuration instance.
            parent (Optional[QWidget], optional): Parent widget. Defaults to None.
        """
        super().__init__(parent)
        
        self.calibration_model = calibration_model
        self.config = config
        
        # State variables
        self.selected_pixel = None  # (x, y, raw_value)
        self.is_entering_temperature = False
        
        # Initialize UI components
        self._init_ui()
        
        # Update UI to reflect initial state
        self.update_calibration_points()
        self.update_calibration_status()
    
    def _init_ui(self):
        """Initialize the user interface components."""
        # Main layout
        main_layout = QVBoxLayout(self)
        
        # Input controls group
        input_group = QGroupBox("Reference Temperature Input")
        input_layout = QVBoxLayout(input_group)
        
        # Selected pixel information
        pixel_info_frame = QFrame()
        pixel_info_layout = QFormLayout(pixel_info_frame)
        self.pixel_x_label = QLabel("N/A")
        self.pixel_y_label = QLabel("N/A")
        self.pixel_value_label = QLabel("N/A")
        pixel_info_layout.addRow("X Coordinate:", self.pixel_x_label)
        pixel_info_layout.addRow("Y Coordinate:", self.pixel_y_label)
        pixel_info_layout.addRow("Raw Value:", self.pixel_value_label)
        input_layout.addWidget(pixel_info_frame)
        
        # Temperature input button
        self.temp_input_button = QPushButton("Enter Temperature Value")
        self.temp_input_button.clicked.connect(self._on_enter_temperature)
        self.temp_input_button.setEnabled(False)  # Disabled until pixel selected
        input_layout.addWidget(self.temp_input_button)
        
        # Recommendation section
        recommendation_frame = QFrame()
        recommendation_layout = QVBoxLayout(recommendation_frame)
        recommendation_label = QLabel("Calibration Recommendation:")
        recommendation_layout.addWidget(recommendation_label)
        self.recommendation_text = QLabel("Select pixels and add calibration points.")
        self.recommendation_text.setWordWrap(True)
        recommendation_layout.addWidget(self.recommendation_text)
        input_layout.addWidget(recommendation_frame)
        
        main_layout.addWidget(input_group)
        
        # Calibration points group
        points_group = QGroupBox("Calibration Points")
        points_layout = QVBoxLayout(points_group)
        
        # Calibration points table
        self.points_table = QTableWidget(0, 4)
        self.points_table.setHorizontalHeaderLabels(["X", "Y", "Raw Value", "Temperature (°C)"])
        self.points_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.points_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.points_table.setEditTriggers(QTableWidget.NoEditTriggers)
        points_layout.addWidget(self.points_table)
        
        # Calibration points controls
        points_controls_layout = QHBoxLayout()
        self.remove_point_button = QPushButton("Remove Point")
        self.remove_point_button.clicked.connect(self._on_remove_point)
        self.remove_point_button.setEnabled(False)  # Disabled until point selected
        points_controls_layout.addWidget(self.remove_point_button)
        
        self.clear_points_button = QPushButton("Clear All Points")
        self.clear_points_button.clicked.connect(self._on_clear_points)
        self.clear_points_button.setEnabled(False)  # Disabled until points added
        points_controls_layout.addWidget(self.clear_points_button)
        
        points_layout.addLayout(points_controls_layout)
        
        main_layout.addWidget(points_group)
        
        # Calibration settings group
        settings_group = QGroupBox("Calibration Settings")
        settings_layout = QFormLayout(settings_group)
        
        # Model type selection
        self.model_type_combo = QComboBox()
        self.model_type_combo.addItems(["Polynomial", "Exponential", "Logarithmic"])
        current_model_type = self.calibration_model.model_type.capitalize()
        current_index = self.model_type_combo.findText(current_model_type)
        if current_index >= 0:
            self.model_type_combo.setCurrentIndex(current_index)
        self.model_type_combo.currentTextChanged.connect(self._on_model_type_changed)
        settings_layout.addRow("Model Type:", self.model_type_combo)
        
        # Polynomial degree (visible only for polynomial model)
        self.degree_spin = QSpinBox()
        self.degree_spin.setMinimum(1)
        self.degree_spin.setMaximum(4)
        self.degree_spin.setValue(self.calibration_model.polynomial_degree)
        self.degree_spin.valueChanged.connect(self._on_degree_changed)
        self.degree_label = QLabel("Polynomial Degree:")
        settings_layout.addRow(self.degree_label, self.degree_spin)
        
        # Update visibility based on model type
        self._update_settings_visibility()
        
        main_layout.addWidget(settings_group)
        
        # Calibration status and controls
        status_group = QGroupBox("Calibration Status")
        status_layout = QVBoxLayout(status_group)
        
        # Status indicators
        status_frame = QFrame()
        status_info_layout = QFormLayout(status_frame)
        self.cal_status_label = QLabel("Not Calibrated")
        self.cal_quality_label = QLabel("N/A")
        self.cal_points_label = QLabel("0 points")
        status_info_layout.addRow("Status:", self.cal_status_label)
        status_info_layout.addRow("Quality:", self.cal_quality_label)
        status_info_layout.addRow("Points:", self.cal_points_label)
        status_layout.addWidget(status_frame)
        
        # Calibration controls
        cal_controls_layout = QHBoxLayout()
        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self.run_calibration)
        self.calibrate_button.setEnabled(False)  # Disabled until enough points added
        cal_controls_layout.addWidget(self.calibrate_button)
        
        self.export_button = QPushButton("Export Calibration")
        self.export_button.clicked.connect(self._on_export_calibration)
        self.export_button.setEnabled(False)  # Disabled until calibrated
        cal_controls_layout.addWidget(self.export_button)
        
        status_layout.addLayout(cal_controls_layout)
        
        # Radiometric mode toggle
        self.radiometric_toggle = QCheckBox("Enable Radiometric Mode")
        self.radiometric_toggle.setChecked(self.config.get("gui.thermal_view.radiometric_mode", False))
        self.radiometric_toggle.setEnabled(False)  # Disabled until calibrated
        self.radiometric_toggle.stateChanged.connect(self._on_radiometric_toggled)
        status_layout.addWidget(self.radiometric_toggle)
        
        main_layout.addWidget(status_group)
        
        # Connect selection signals
        self.points_table.itemSelectionChanged.connect(self._on_point_selection_changed)
    
    @pyqtSlot(int, int, int)
    def on_pixel_selected(self, x: int, y: int, raw_value: int):
        """
        Handle pixel selection from thermal view.
        
        Args:
            x (int): X coordinate.
            y (int): Y coordinate.
            raw_value (int): Raw thermal value.
        """
        # Store selected pixel
        self.selected_pixel = (x, y, raw_value)
        
        # Update UI
        self.pixel_x_label.setText(str(x))
        self.pixel_y_label.setText(str(y))
        self.pixel_value_label.setText(str(raw_value))
        
        # Enable temperature input button
        self.temp_input_button.setEnabled(True)
        
        logger.debug(f"Selected pixel: ({x}, {y}) with raw value {raw_value}")
    
    def update_calibration_points(self):
        """Update the calibration points table and controls."""
        # Clear table
        self.points_table.setRowCount(0)
        
        # Add points from calibration model
        points = self.calibration_model.calibration_points
        self.points_table.setRowCount(len(points))
        
        for i, point in enumerate(points):
            # X coordinate
            x_item = QTableWidgetItem(str(point.x))
            self.points_table.setItem(i, 0, x_item)
            
            # Y coordinate
            y_item = QTableWidgetItem(str(point.y))
            self.points_table.setItem(i, 1, y_item)
            
            # Raw value
            raw_item = QTableWidgetItem(str(point.raw_value))
            self.points_table.setItem(i, 2, raw_item)
            
            # Temperature
            temp_item = QTableWidgetItem(f"{point.reference_temp:.1f}")
            self.points_table.setItem(i, 3, temp_item)
        
        # Update button states
        self.clear_points_button.setEnabled(len(points) > 0)
        self.calibrate_button.setEnabled(len(points) >= 3)
        
        # Update status label
        self.cal_points_label.setText(f"{len(points)} points")
        
        # Update recommendation
        self._update_recommendation()
    
    def update_calibration_status(self):
        """Update the calibration status indicators."""
        # Update calibration status
        if self.calibration_model.is_calibrated:
            self.cal_status_label.setText("Calibrated")
            
            # Enable radiometric mode and export
            self.radiometric_toggle.setEnabled(True)
            self.export_button.setEnabled(True)
            
            # Update quality indicator
            quality_score, quality_description = self.calibration_model.get_calibration_quality_indicator()
            self.cal_quality_label.setText(f"{quality_description} ({quality_score:.2f})")
            
            # Set label color based on quality
            if quality_score > 0.8:
                self.cal_quality_label.setStyleSheet("color: green;")
            elif quality_score > 0.5:
                self.cal_quality_label.setStyleSheet("color: orange;")
            else:
                self.cal_quality_label.setStyleSheet("color: red;")
        else:
            self.cal_status_label.setText("Not Calibrated")
            self.cal_quality_label.setText("N/A")
            self.cal_quality_label.setStyleSheet("")
            
            # Disable radiometric mode and export
            self.radiometric_toggle.setEnabled(False)
            self.export_button.setEnabled(False)
    
    def _on_enter_temperature(self):
        """Handle temperature input button click."""
        if self.selected_pixel is None:
            return
        
        x, y, raw_value = self.selected_pixel
        
        # Get calibration recommendation
        recommended_temp = None
        if not self.calibration_model.calibration_points:
            # For first point, recommend room temperature
            recommended_temp = 20.0
        elif self.calibration_model.get_recommended_calibration_point():
            # Get recommended temperature from model
            recommended_temp, _ = self.calibration_model.get_recommended_calibration_point()
        
        # Show temperature input dialog
        dialog = TemperatureInputDialog(
            x, y, raw_value, 
            recommended_temp=recommended_temp,
            parent=self
        )
        
        if dialog.exec_():
            # Get temperature value
            temperature = dialog.get_temperature()
            
            # Add calibration point
            self.calibration_model.add_calibration_point(
                x, y, raw_value, temperature
            )
            
            # Update UI
            self.update_calibration_points()
            
            # Emit signal to update thermal view
            self.calibration_updated.emit()
            
            logger.info(f"Added calibration point: ({x}, {y}) = {temperature}°C")
    
    def _on_remove_point(self):
        """Handle remove point button click."""
        # Get selected row
        selected_rows = self.points_table.selectedIndexes()
        if not selected_rows:
            return
        
        # Get row index
        row = selected_rows[0].row()
        
        # Remove point from model
        if self.calibration_model.remove_calibration_point(row):
            # Update UI
            self.update_calibration_points()
            
            # Reset calibration status
            self.update_calibration_status()
            
            # Emit signal to update thermal view
            self.calibration_updated.emit()
            
            logger.info(f"Removed calibration point {row}")
    
    def _on_clear_points(self):
        """Handle clear all points button click."""
        # Show confirmation dialog
        reply = QMessageBox.question(
            self, 
            "Clear Calibration Points", 
            "Are you sure you want to clear all calibration points?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Clear points from model
            self.calibration_model.clear_calibration_points()
            
            # Update UI
            self.update_calibration_points()
            
            # Reset calibration status
            self.update_calibration_status()
            
            # Emit signal to update thermal view
            self.calibration_updated.emit()
            
            logger.info("Cleared all calibration points")
    
    def _on_point_selection_changed(self):
        """Handle selection changes in the points table."""
        # Enable/disable remove button based on selection
        selected_rows = self.points_table.selectedIndexes()
        self.remove_point_button.setEnabled(bool(selected_rows))
    
    def _on_model_type_changed(self, model_type: str):
        """
        Handle model type selection change.
        
        Args:
            model_type (str): New model type.
        """
        # Update model type
        self.calibration_model.model_type = model_type.lower()
        
        # Update settings visibility
        self._update_settings_visibility()
        
        logger.debug(f"Model type changed to {model_type}")
    
    def _on_degree_changed(self, degree: int):
        """
        Handle polynomial degree change.
        
        Args:
            degree (int): New polynomial degree.
        """
        # Update polynomial degree
        self.calibration_model.polynomial_degree = degree
        
        logger.debug(f"Polynomial degree changed to {degree}")
    
    def _update_settings_visibility(self):
        """Update visibility of settings based on model type."""
        # Show polynomial degree only for polynomial model
        is_polynomial = self.calibration_model.model_type.lower() == "polynomial"
        self.degree_label.setVisible(is_polynomial)
        self.degree_spin.setVisible(is_polynomial)
    
    def run_calibration(self):
        """Run the calibration process."""
        # Check if we have enough points
        if len(self.calibration_model.calibration_points) < 3:
            QMessageBox.warning(
                self,
                "Insufficient Calibration Points",
                "At least 3 calibration points are required. Please add more points."
            )
            return
        
        try:
            # Run calibration
            success = self.calibration_model.calibrate()
            
            if success:
                # Update status
                self.update_calibration_status()
                
                # Show success message
                QMessageBox.information(
                    self,
                    "Calibration Complete",
                    "Calibration completed successfully."
                )
                
                # Update recommendation
                self._update_recommendation()
                
                # Emit signal to update thermal view
                self.calibration_updated.emit()
                
                logger.info("Calibration completed successfully")
            else:
                # Show error message
                QMessageBox.warning(
                    self,
                    "Calibration Failed",
                    "Failed to create calibration model. Try adding more points or different temperatures."
                )
        except Exception as e:
            logger.error(f"Error during calibration: {e}")
            QMessageBox.critical(
                self,
                "Calibration Error",
                f"An error occurred during calibration: {e}"
            )
    
    def _on_export_calibration(self):
        """Handle export calibration button click."""
        from PyQt5.QtWidgets import QFileDialog
        import os
        
        # Check if calibrated
        if not self.calibration_model.is_calibrated:
            QMessageBox.warning(
                self,
                "Not Calibrated",
                "Please run calibration before exporting."
            )
            return
        
        # Show file save dialog
        filepath, _ = QFileDialog.getSaveFileName(
            self,
            "Export Calibration",
            os.path.expanduser("~/thermal_calibration.json"),
            "Calibration Files (*.json)"
        )
        
        if not filepath:
            return
        
        # Add .json extension if needed
        if not filepath.lower().endswith('.json'):
            filepath += '.json'
        
        try:
            # Save calibration model
            if self.calibration_model.save_to_file(filepath):
                QMessageBox.information(
                    self,
                    "Export Successful",
                    f"Calibration exported to {filepath}"
                )
                logger.info(f"Exported calibration to {filepath}")
            else:
                QMessageBox.warning(
                    self,
                    "Export Failed",
                    "Failed to export calibration."
                )
        except Exception as e:
            logger.error(f"Error exporting calibration: {e}")
            QMessageBox.critical(
                self,
                "Export Error",
                f"An error occurred during export: {e}"
            )
    
    def _on_radiometric_toggled(self, state: int):
        """
        Handle radiometric mode toggle.
        
        Args:
            state (int): Checkbox state.
        """
        enabled = state == Qt.Checked
        
        # Emit signal to update thermal view
        self.radiometric_mode_toggled.emit(enabled)
        
        logger.debug(f"Radiometric mode {'enabled' if enabled else 'disabled'}")
    
    def _update_recommendation(self):
        """Update the calibration recommendation label."""
        if not self.calibration_model.calibration_points:
            self.recommendation_text.setText(
                "Select a pixel and add your first calibration point. "
                "Start with a room temperature reference."
            )
            return
        
        # Get recommendation from model
        recommendation = self.calibration_model.get_recommended_calibration_point()
        
        if not recommendation:
            if self.calibration_model.is_calibrated:
                quality_score, _ = self.calibration_model.get_calibration_quality_indicator()
                if quality_score > 0.8:
                    self.recommendation_text.setText("Your calibration quality is excellent!")
                else:
                    self.recommendation_text.setText(
                        "Consider adding more points to improve calibration quality."
                    )
            else:
                self.recommendation_text.setText("Run calibration with your current points.")
            return
        
        # Display recommendation
        temp, reason = recommendation
        self.recommendation_text.setText(
            f"Recommended next point: {temp:.1f}°C\n"
            f"Reason: {reason}"
        )