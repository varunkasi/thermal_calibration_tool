#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main window implementation for the Thermal Camera Calibration Tool.

This module implements the main GUI window for the thermal camera calibration tool,
integrating the thermal view and calibration panels.
"""

import os
from typing import Optional, Dict, Any

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QSplitter,
    QLabel, QPushButton, QStatusBar, QAction, QFileDialog, QMessageBox,
    QMenu, QToolBar
)
from PyQt5.QtGui import QIcon, QKeySequence
from PyQt5.QtCore import Qt, QSettings, QSize, QPoint, pyqtSlot, QTimer

from src.utils.logger import get_logger
from src.utils.config import Config
from src.camera.boson_camera import BosonCamera
from src.calibration.calibration_model import CalibrationModel
from src.processing.thermal_processor import ThermalProcessor
from src.gui.thermal_view import ThermalView
from src.gui.calibration_panel import CalibrationPanel


logger = get_logger(__name__)


class MainWindow(QMainWindow):
    """Main window for the Thermal Camera Calibration Tool."""

    def __init__(
        self,
        camera: Optional[BosonCamera],
        thermal_processor: ThermalProcessor,
        calibration_model: CalibrationModel,
        config: Config,
        parent: Optional[QWidget] = None,
    ):
        """
        Initialize the main window.

        Args:
            camera (Optional[BosonCamera]): Camera interface instance.
            thermal_processor (ThermalProcessor): Thermal processor instance.
            calibration_model (CalibrationModel): Calibration model instance.
            config (Config): Configuration instance.
            parent (Optional[QWidget], optional): Parent widget. Defaults to None.
        """
        super().__init__(parent)
        
        self.camera = camera
        self.thermal_processor = thermal_processor
        self.calibration_model = calibration_model
        self.config = config
        
        # Window setup
        self.setWindowTitle("Thermal Camera Calibration Tool")
        self.setMinimumSize(1024, 768)
        
        # Load window settings
        self._load_window_settings()
        
        # Initialize UI components
        self._init_ui()
        
        # Set up timer for frame updates
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_frame)
        self.update_interval = config.get("gui.update_interval_ms", 100)
        
        # Start frame updates if camera is connected
        if self.camera and self.camera.is_connected():
            self.update_timer.start(self.update_interval)
            self.statusBar().showMessage("Camera connected", 3000)
        else:
            self.statusBar().showMessage("Camera not connected", 3000)
    
    def _init_ui(self):
        """Initialize the user interface components."""
        # Create central widget and layout
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        
        # Main horizontal layout using splitter
        self.main_splitter = QSplitter(Qt.Horizontal, self.central_widget)
        
        # Create thermal view (left panel)
        self.thermal_view = ThermalView(
            self.thermal_processor, self.calibration_model, self.config, self.main_splitter
        )
        
        # Create calibration panel (right panel)
        self.calibration_panel = CalibrationPanel(
            self.calibration_model, self.config, self.main_splitter
        )
        
        # Connect signals between panels
        self._connect_signals()
        
        # Set initial splitter sizes (60% thermal view, 40% calibration panel)
        self.main_splitter.setSizes([600, 400])
        
        # Set main layout
        main_layout = QHBoxLayout(self.central_widget)
        main_layout.addWidget(self.main_splitter)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Create status bar
        self.status_bar = QStatusBar(self)
        self.setStatusBar(self.status_bar)
        
        # Create camera info label in status bar
        self.camera_info_label = QLabel()
        self.status_bar.addPermanentWidget(self.camera_info_label)
        self._update_camera_info()
        
        # Create menu bar and toolbar
        self._create_menu_bar()
        self._create_tool_bar()
    
    def _create_menu_bar(self):
        """Create the menu bar with File, View, Tools, and Help menus."""
        # File menu
        file_menu = self.menuBar().addMenu("&File")
        
        new_action = QAction("&New Calibration", self)
        new_action.setShortcut(QKeySequence.New)
        new_action.triggered.connect(self._new_calibration)
        file_menu.addAction(new_action)
        
        open_action = QAction("&Open Calibration...", self)
        open_action.setShortcut(QKeySequence.Open)
        open_action.triggered.connect(self._open_calibration)
        file_menu.addAction(open_action)
        
        save_action = QAction("&Save Calibration", self)
        save_action.setShortcut(QKeySequence.Save)
        save_action.triggered.connect(self._save_calibration)
        file_menu.addAction(save_action)
        
        save_as_action = QAction("Save Calibration &As...", self)
        save_as_action.setShortcut(QKeySequence.SaveAs)
        save_as_action.triggered.connect(self._save_calibration_as)
        file_menu.addAction(save_as_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("E&xit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # View menu
        view_menu = self.menuBar().addMenu("&View")
        
        toggle_grid_action = QAction("Show &Grid", self)
        toggle_grid_action.setCheckable(True)
        toggle_grid_action.setChecked(self.config.get("gui.thermal_view.show_grid", False))
        toggle_grid_action.triggered.connect(self.thermal_view.toggle_grid)
        view_menu.addAction(toggle_grid_action)
        
        view_menu.addSeparator()
        
        palette_menu = QMenu("Color &Palette", self)
        view_menu.addMenu(palette_menu)
        
        palette_group = QActionGroup(self)
        palette_group.setExclusive(True)
        
        palettes = self.config.get("gui.thermal_view.palettes", ["Inferno", "Jet", "Grayscale"])
        current_palette = self.config.get("gui.thermal_view.current_palette", "Inferno")
        
        for palette in palettes:
            action = QAction(palette, self)
            action.setCheckable(True)
            action.setChecked(palette == current_palette)
            action.triggered.connect(lambda checked, p=palette: self.thermal_view.set_palette(p))
            palette_group.addAction(action)
            palette_menu.addAction(action)
        
        # Tools menu
        tools_menu = self.menuBar().addMenu("&Tools")
        
        connect_camera_action = QAction("&Connect Camera", self)
        connect_camera_action.triggered.connect(self._connect_camera)
        tools_menu.addAction(connect_camera_action)
        
        disconnect_camera_action = QAction("&Disconnect Camera", self)
        disconnect_camera_action.triggered.connect(self._disconnect_camera)
        tools_menu.addAction(disconnect_camera_action)
        
        tools_menu.addSeparator()
        
        toggle_radiometric_action = QAction("&Radiometric Mode", self)
        toggle_radiometric_action.setCheckable(True)
        toggle_radiometric_action.setChecked(self.config.get("gui.thermal_view.radiometric_mode", False))
        toggle_radiometric_action.triggered.connect(self.thermal_view.toggle_radiometric_mode)
        tools_menu.addAction(toggle_radiometric_action)
        
        # Help menu
        help_menu = self.menuBar().addMenu("&Help")
        
        about_action = QAction("&About", self)
        about_action.triggered.connect(self._show_about_dialog)
        help_menu.addAction(about_action)
    
    def _create_tool_bar(self):
        """Create the main toolbar."""
        self.toolbar = QToolBar("Main Toolbar", self)
        self.toolbar.setMovable(False)
        self.toolbar.setIconSize(QSize(24, 24))
        self.addToolBar(self.toolbar)
        
        # Add toolbar actions
        # In a real implementation, we would use actual icons
        new_action = QAction("New", self)
        new_action.triggered.connect(self._new_calibration)
        self.toolbar.addAction(new_action)
        
        open_action = QAction("Open", self)
        open_action.triggered.connect(self._open_calibration)
        self.toolbar.addAction(open_action)
        
        save_action = QAction("Save", self)
        save_action.triggered.connect(self._save_calibration)
        self.toolbar.addAction(save_action)
        
        self.toolbar.addSeparator()
        
        connect_action = QAction("Connect", self)
        connect_action.triggered.connect(self._connect_camera)
        self.toolbar.addAction(connect_action)
        
        self.toolbar.addSeparator()
        
        calibrate_action = QAction("Calibrate", self)
        calibrate_action.triggered.connect(self.calibration_panel.run_calibration)
        self.toolbar.addAction(calibrate_action)
        
        self.toolbar.addSeparator()
        
        radiometric_action = QAction("Radiometric", self)
        radiometric_action.setCheckable(True)
        radiometric_action.setChecked(self.config.get("gui.thermal_view.radiometric_mode", False))
        radiometric_action.triggered.connect(self.thermal_view.toggle_radiometric_mode)
        self.toolbar.addAction(radiometric_action)
    
    def _connect_signals(self):
        """Connect signals between components."""
        # Connect thermal view pixel selection to calibration panel
        self.thermal_view.pixel_selected.connect(self.calibration_panel.on_pixel_selected)
        
        # Connect calibration panel signals to thermal view
        self.calibration_panel.calibration_updated.connect(self.thermal_view.on_calibration_updated)
        self.calibration_panel.radiometric_mode_toggled.connect(self.thermal_view.set_radiometric_mode)
    
    def _update_frame(self):
        """Update the thermal frame from the camera."""
        if not self.camera or not self.camera.is_connected():
            return
        
        try:
            # Get frame from camera
            frame = self.camera.get_frame()
            if frame is not None:
                # Process frame
                processed_frame = self.thermal_processor.process_frame(frame)
                
                # Update thermal view
                self.thermal_view.update_frame(processed_frame, frame)
                
                # Update status
                self._update_camera_info()
        except Exception as e:
            logger.error(f"Error updating frame: {e}")
            self.statusBar().showMessage(f"Error: {e}", 3000)
    
    def _update_camera_info(self):
        """Update camera information in the status bar."""
        if self.camera and self.camera.is_connected():
            info = self.camera.get_camera_info()
            self.camera_info_label.setText(
                f"Camera: {info.get('model', 'Unknown')} | "
                f"S/N: {info.get('serial_number', 'Unknown')} | "
                f"Resolution: {info.get('resolution', 'Unknown')}"
            )
        else:
            self.camera_info_label.setText("Camera: Not connected")
    
    def _connect_camera(self):
        """Connect to the camera."""
        if not self.camera:
            self.statusBar().showMessage("Camera interface not available", 3000)
            return
        
        if self.camera.is_connected():
            self.statusBar().showMessage("Camera already connected", 3000)
            return
        
        try:
            # Attempt to connect to the camera
            if self.camera.connect():
                self.update_timer.start(self.update_interval)
                self.statusBar().showMessage("Camera connected successfully", 3000)
            else:
                self.statusBar().showMessage("Failed to connect to camera", 3000)
        except Exception as e:
            logger.error(f"Error connecting to camera: {e}")
            self.statusBar().showMessage(f"Error connecting to camera: {e}", 3000)
    
    def _disconnect_camera(self):
        """Disconnect from the camera."""
        if not self.camera or not self.camera.is_connected():
            self.statusBar().showMessage("Camera not connected", 3000)
            return
        
        try:
            # Stop frame update timer
            self.update_timer.stop()
            
            # Disconnect from camera
            self.camera.disconnect()
            self.statusBar().showMessage("Camera disconnected", 3000)
            
            # Update camera info
            self._update_camera_info()
        except Exception as e:
            logger.error(f"Error disconnecting from camera: {e}")
            self.statusBar().showMessage(f"Error disconnecting from camera: {e}", 3000)
    
    def _new_calibration(self):
        """Create a new calibration."""
        if self.calibration_model.calibration_points and not self._confirm_discard_changes():
            return
        
        try:
            # Clear calibration model
            self.calibration_model.clear_calibration_points()
            
            # Update panels
            self.calibration_panel.update_calibration_points()
            self.thermal_view.on_calibration_updated()
            
            self.statusBar().showMessage("New calibration created", 3000)
        except Exception as e:
            logger.error(f"Error creating new calibration: {e}")
            self.statusBar().showMessage(f"Error creating new calibration: {e}", 3000)
    
    def _open_calibration(self):
        """Open a calibration file."""
        if self.calibration_model.calibration_points and not self._confirm_discard_changes():
            return
        
        try:
            # Show file dialog
            filepath, _ = QFileDialog.getOpenFileName(
                self, "Open Calibration", "", "Calibration Files (*.json);;All Files (*)"
            )
            
            if not filepath:
                return
            
            # Load calibration model
            if self.calibration_model.load_from_file(filepath):
                # Update panels
                self.calibration_panel.update_calibration_points()
                self.thermal_view.on_calibration_updated()
                
                self.statusBar().showMessage(f"Loaded calibration from {filepath}", 3000)
            else:
                self.statusBar().showMessage("Failed to load calibration", 3000)
        except Exception as e:
            logger.error(f"Error opening calibration: {e}")
            self.statusBar().showMessage(f"Error opening calibration: {e}", 3000)
    
    def _save_calibration(self):
        """Save the current calibration."""
        try:
            # Check if we have a current filepath
            data_dir = os.path.expanduser("~/thermal_calibration_data")
            os.makedirs(data_dir, exist_ok=True)
            
            filepath = os.path.join(data_dir, "current_calibration.json")
            
            # Save calibration model
            if self.calibration_model.save_to_file(filepath):
                self.statusBar().showMessage(f"Saved calibration to {filepath}", 3000)
            else:
                self.statusBar().showMessage("Failed to save calibration", 3000)
        except Exception as e:
            logger.error(f"Error saving calibration: {e}")
            self.statusBar().showMessage(f"Error saving calibration: {e}", 3000)
    
    def _save_calibration_as(self):
        """Save the current calibration to a new file."""
        try:
            # Show file dialog
            filepath, _ = QFileDialog.getSaveFileName(
                self, "Save Calibration As", "", "Calibration Files (*.json);;All Files (*)"
            )
            
            if not filepath:
                return
            
            # Add .json extension if not present
            if not filepath.lower().endswith('.json'):
                filepath += '.json'
            
            # Save calibration model
            if self.calibration_model.save_to_file(filepath):
                self.statusBar().showMessage(f"Saved calibration to {filepath}", 3000)
            else:
                self.statusBar().showMessage("Failed to save calibration", 3000)
        except Exception as e:
            logger.error(f"Error saving calibration: {e}")
            self.statusBar().showMessage(f"Error saving calibration: {e}", 3000)
    
    def _show_about_dialog(self):
        """Show the about dialog."""
        QMessageBox.about(
            self,
            "About Thermal Camera Calibration Tool",
            "<h1>Thermal Camera Calibration Tool</h1>"
            "<p>Version 1.0.0</p>"
            "<p>A tool for radiometric calibration of FLIR Boson+ thermal cameras.</p>"
            "<p>&copy; 2025 Thermal Tools Inc.</p>"
        )
    
    def _confirm_discard_changes(self) -> bool:
        """
        Confirm discarding changes to the current calibration.
        
        Returns:
            bool: True if user confirms or no changes to discard, False otherwise.
        """
        if not self.calibration_model.calibration_points:
            return True
        
        reply = QMessageBox.question(
            self,
            "Discard Changes",
            "You have unsaved calibration points. Discard changes?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        return reply == QMessageBox.Yes
    
    def _load_window_settings(self):
        """Load window settings from QSettings."""
        settings = QSettings("ThermalTools", "ThermalCalibration")
        
        # Load window geometry
        geometry = settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)
        else:
            # Default position and size
            self.resize(1200, 800)
            self.move(100, 100)
        
        # Load window state
        state = settings.value("windowState")
        if state:
            self.restoreState(state)
    
    def _save_window_settings(self):
        """Save window settings to QSettings."""
        settings = QSettings("ThermalTools", "ThermalCalibration")
        
        # Save window geometry
        settings.setValue("geometry", self.saveGeometry())
        
        # Save window state
        settings.setValue("windowState", self.saveState())
    
    def closeEvent(self, event):
        """Handle window close event."""
        # Save window settings
        self._save_window_settings()
        
        # Disconnect from camera
        if self.camera and self.camera.is_connected():
            self.camera.disconnect()
        
        # Stop update timer
        if self.update_timer.isActive():
            self.update_timer.stop()
        
        # Accept the close event
        event.accept()