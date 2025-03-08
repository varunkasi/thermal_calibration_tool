#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Thermal view widget for the Thermal Camera Calibration Tool.

This module implements the thermal image display with interactive features
such as pixel selection, zooming, and temperature display.
"""

import numpy as np
from typing import Optional, Tuple, List, Dict, Any

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QComboBox,
    QPushButton, QGroupBox, QCheckBox, QFrame
)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen, QMouseEvent, QWheelEvent
from PyQt5.QtCore import Qt, QPoint, QRect, QSize, pyqtSignal, pyqtSlot

from src.utils.logger import get_logger
from src.utils.config import Config
from src.calibration.calibration_model import CalibrationModel
from src.processing.thermal_processor import ThermalProcessor, ColorPalette


logger = get_logger(__name__)


class ThermalView(QWidget):
    """Widget for displaying and interacting with thermal images."""
    
    # Signal emitted when a pixel is selected
    pixel_selected = pyqtSignal(int, int, int)  # x, y, raw_value
    
    def __init__(
        self, 
        thermal_processor: ThermalProcessor,
        calibration_model: CalibrationModel,
        config: Config,
        parent: Optional[QWidget] = None
    ):
        """
        Initialize the thermal view widget.
        
        Args:
            thermal_processor (ThermalProcessor): Thermal processor instance.
            calibration_model (CalibrationModel): Calibration model instance.
            config (Config): Configuration instance.
            parent (Optional[QWidget], optional): Parent widget. Defaults to None.
        """
        super().__init__(parent)
        
        self.thermal_processor = thermal_processor
        self.calibration_model = calibration_model
        self.config = config
        
        # Image data
        self.raw_frame = None  # Raw 16-bit thermal data
        self.processed_frame = None  # Processed frame ready for display
        self.display_image = None  # QImage for display
        
        # Selection state
        self.selected_pixel = None  # (x, y) coordinates of selected pixel
        self.highlighted_pixel = None  # (x, y) coordinates of highlighted/hovered pixel
        self.calibration_points = []  # List of calibration points to mark on the image
        
        # Display options
        self.zoom_factor = 1.0
        self.pan_offset = QPoint(0, 0)
        self.show_grid = config.get("gui.thermal_view.show_grid", False)
        self.radiometric_mode = config.get("gui.thermal_view.radiometric_mode", False)
        self.current_palette = config.get("gui.thermal_view.current_palette", "Inferno")
        
        # Initialize UI components
        self._init_ui()
    
    def _init_ui(self):
        """Initialize the user interface components."""
        # Set up main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Image display frame
        self.image_frame = QLabel(self)
        self.image_frame.setAlignment(Qt.AlignCenter)
        self.image_frame.setMinimumSize(320, 240)
        self.image_frame.setSizePolicy(
            QVBoxLayout.SizePolicy.Expanding, 
            QVBoxLayout.SizePolicy.Expanding
        )
        self.image_frame.setStyleSheet("background-color: black;")
        main_layout.addWidget(self.image_frame)
        
        # Controls layout
        controls_layout = QHBoxLayout()
        
        # Palette selection
        palette_group = QGroupBox("Color Palette")
        palette_layout = QHBoxLayout(palette_group)
        self.palette_combo = QComboBox()
        self.palette_combo.addItems(["Inferno", "Jet", "Viridis", "Grayscale", "IronBow"])
        current_index = self.palette_combo.findText(self.current_palette)
        if current_index >= 0:
            self.palette_combo.setCurrentIndex(current_index)
        self.palette_combo.currentTextChanged.connect(self.set_palette)
        palette_layout.addWidget(self.palette_combo)
        controls_layout.addWidget(palette_group)
        
        # Zoom controls
        zoom_group = QGroupBox("Zoom")
        zoom_layout = QHBoxLayout(zoom_group)
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setMinimum(10)  # 10% zoom
        self.zoom_slider.setMaximum(300)  # 300% zoom
        self.zoom_slider.setValue(int(self.zoom_factor * 100))
        self.zoom_slider.setTickPosition(QSlider.TicksBelow)
        self.zoom_slider.setTickInterval(50)
        self.zoom_slider.valueChanged.connect(self._on_zoom_changed)
        zoom_layout.addWidget(self.zoom_slider)
        self.zoom_label = QLabel(f"{int(self.zoom_factor * 100)}%")
        zoom_layout.addWidget(self.zoom_label)
        controls_layout.addWidget(zoom_group)
        
        # Grid and radiometric mode toggles
        display_group = QGroupBox("Display Options")
        display_layout = QHBoxLayout(display_group)
        
        self.grid_checkbox = QCheckBox("Show Grid")
        self.grid_checkbox.setChecked(self.show_grid)
        self.grid_checkbox.stateChanged.connect(self.toggle_grid)
        display_layout.addWidget(self.grid_checkbox)
        
        self.radiometric_checkbox = QCheckBox("Radiometric Mode")
        self.radiometric_checkbox.setChecked(self.radiometric_mode)
        self.radiometric_checkbox.stateChanged.connect(self.toggle_radiometric_mode)
        display_layout.addWidget(self.radiometric_checkbox)
        
        controls_layout.addWidget(display_group)
        
        # Reset view button
        self.reset_view_button = QPushButton("Reset View")
        self.reset_view_button.clicked.connect(self._reset_view)
        controls_layout.addWidget(self.reset_view_button)
        
        main_layout.addLayout(controls_layout)
        
        # Temperature info bar
        info_layout = QHBoxLayout()
        self.pixel_info_label = QLabel("Pixel: N/A")
        self.temp_info_label = QLabel("Temperature: N/A")
        info_layout.addWidget(self.pixel_info_label)
        info_layout.addWidget(self.temp_info_label)
        main_layout.addLayout(info_layout)
        
        # Enable mouse tracking for hover effects
        self.image_frame.setMouseTracking(True)
        self.setMouseTracking(True)
        
        # Install event filter for mouse events on the image frame
        self.image_frame.installEventFilter(self)
    
    def update_frame(self, processed_frame: np.ndarray, raw_frame: Optional[np.ndarray] = None):
        """
        Update the display with a new thermal frame.
        
        Args:
            processed_frame (np.ndarray): Processed frame for display (8-bit RGB).
            raw_frame (Optional[np.ndarray], optional): Raw 16-bit thermal data. Defaults to None.
        """
        if processed_frame is None:
            return
        
        # Store frame data
        self.processed_frame = processed_frame
        if raw_frame is not None:
            self.raw_frame = raw_frame
        
        # Update the displayed image
        self._update_display()
    
    def _update_display(self):
        """Update the displayed image with current settings."""
        if self.processed_frame is None:
            return
        
        # Get image dimensions
        height, width, _ = self.processed_frame.shape
        
        # Create QImage from the processed frame
        bytes_per_line = 3 * width
        q_img = QImage(
            self.processed_frame.data, 
            width, 
            height, 
            bytes_per_line, 
            QImage.Format_RGB888
        )
        
        # Create a pixmap from the QImage
        pixmap = QPixmap.fromImage(q_img)
        
        # Apply zoom
        zoomed_size = QSize(
            int(pixmap.width() * self.zoom_factor),
            int(pixmap.height() * self.zoom_factor)
        )
        pixmap = pixmap.scaled(
            zoomed_size, 
            Qt.KeepAspectRatio, 
            Qt.SmoothTransformation
        )
        
        # Create a painter to draw on the pixmap
        painter = QPainter(pixmap)
        
        # Draw grid if enabled
        if self.show_grid:
            self._draw_grid(painter, pixmap.width(), pixmap.height())
        
        # Draw calibration points
        self._draw_calibration_points(painter)
        
        # Draw selected pixel
        if self.selected_pixel:
            x, y = self.selected_pixel
            x_scaled = int(x * self.zoom_factor)
            y_scaled = int(y * self.zoom_factor)
            
            # Draw a crosshair at the selected pixel
            self._draw_crosshair(
                painter, x_scaled, y_scaled, 
                Qt.red, 2, 10
            )
        
        # Draw highlighted pixel (hover)
        if self.highlighted_pixel:
            x, y = self.highlighted_pixel
            x_scaled = int(x * self.zoom_factor)
            y_scaled = int(y * self.zoom_factor)
            
            # Draw a crosshair at the highlighted pixel
            self._draw_crosshair(
                painter, x_scaled, y_scaled, 
                Qt.yellow, 1, 8
            )
            
            # Show temperature if in radiometric mode
            if self.radiometric_mode and self.raw_frame is not None and self.calibration_model.is_calibrated:
                try:
                    raw_value = int(self.raw_frame[y, x])
                    temp = self.calibration_model.raw_to_temperature(raw_value)
                    
                    # Draw temperature value
                    painter.setPen(QPen(Qt.white))
                    painter.drawText(
                        x_scaled + 12, 
                        y_scaled - 12, 
                        f"{temp:.1f}°C"
                    )
                except Exception as e:
                    logger.error(f"Error displaying temperature: {e}")
        
        # End painting
        painter.end()
        
        # Set the pixmap to the image frame
        self.image_frame.setPixmap(pixmap)
        
        # Update display image for reference
        self.display_image = pixmap
    
    def _draw_grid(self, painter: QPainter, width: int, height: int):
        """
        Draw a grid on the image.
        
        Args:
            painter (QPainter): QPainter instance.
            width (int): Image width.
            height (int): Image height.
        """
        # Set up pen for grid lines
        painter.setPen(QPen(QColor(255, 255, 255, 100), 1, Qt.DashLine))
        
        # Draw vertical grid lines
        step = max(20, int(width / 20))  # Grid spacing
        for x in range(0, width, step):
            painter.drawLine(x, 0, x, height)
        
        # Draw horizontal grid lines
        for y in range(0, height, step):
            painter.drawLine(0, y, width, y)
    
    def _draw_calibration_points(self, painter: QPainter):
        """
        Draw calibration points on the image.
        
        Args:
            painter (QPainter): QPainter instance.
        """
        if not self.calibration_model:
            return
        
        # Iterate through calibration points
        for i, point in enumerate(self.calibration_model.calibration_points):
            # Calculate scaled coordinates
            x_scaled = int(point.x * self.zoom_factor)
            y_scaled = int(point.y * self.zoom_factor)
            
            # Draw a circle for each calibration point
            painter.setPen(QPen(Qt.green, 2))
            painter.drawEllipse(x_scaled - 5, y_scaled - 5, 10, 10)
            
            # Draw point index
            painter.setPen(QPen(Qt.white))
            painter.drawText(x_scaled + 8, y_scaled + 4, str(i+1))
    
    def _draw_crosshair(
        self, 
        painter: QPainter, 
        x: int, 
        y: int, 
        color: QColor, 
        width: int,
        size: int
    ):
        """
        Draw a crosshair at the specified coordinates.
        
        Args:
            painter (QPainter): QPainter instance.
            x (int): X coordinate.
            y (int): Y coordinate.
            color (QColor): Color of the crosshair.
            width (int): Line width.
            size (int): Size of the crosshair.
        """
        # Set up pen
        painter.setPen(QPen(color, width))
        
        # Draw horizontal line
        painter.drawLine(x - size, y, x + size, y)
        
        # Draw vertical line
        painter.drawLine(x, y - size, x, y + size)
    
    def set_palette(self, palette_name: str):
        """
        Set the color palette for thermal display.
        
        Args:
            palette_name (str): Name of the palette to use.
        """
        try:
            # Update current palette
            self.current_palette = palette_name
            
            # Update thermal processor palette
            palette = ColorPalette[palette_name.upper()]
            self.thermal_processor.set_palette(palette)
            
            # If we have a frame, reprocess and update
            if self.raw_frame is not None:
                self.processed_frame = self.thermal_processor.process_frame(self.raw_frame)
                self._update_display()
            
            logger.debug(f"Set color palette to {palette_name}")
        except Exception as e:
            logger.error(f"Error setting palette {palette_name}: {e}")
    
    def toggle_grid(self, state: Optional[bool] = None):
        """
        Toggle the grid display.
        
        Args:
            state (Optional[bool], optional): Grid state to set, or None to toggle. Defaults to None.
        """
        if state is None:
            self.show_grid = not self.show_grid
        else:
            self.show_grid = bool(state)
        
        # Update checkbox if called programmatically
        if self.grid_checkbox.isChecked() != self.show_grid:
            self.grid_checkbox.setChecked(self.show_grid)
        
        # Update display
        self._update_display()
        logger.debug(f"Grid display: {'on' if self.show_grid else 'off'}")
    
    def toggle_radiometric_mode(self, state: Optional[bool] = None):
        """
        Toggle radiometric mode.
        
        Args:
            state (Optional[bool], optional): Radiometric mode state to set, or None to toggle. 
                Defaults to None.
        """
        if state is None:
            self.radiometric_mode = not self.radiometric_mode
        else:
            self.radiometric_mode = bool(state)
        
        # Update checkbox if called programmatically
        if self.radiometric_checkbox.isChecked() != self.radiometric_mode:
            self.radiometric_checkbox.setChecked(self.radiometric_mode)
        
        # Update display
        self._update_display()
        logger.debug(f"Radiometric mode: {'on' if self.radiometric_mode else 'off'}")
    
    def set_radiometric_mode(self, enabled: bool):
        """
        Set radiometric mode.
        
        Args:
            enabled (bool): Whether to enable radiometric mode.
        """
        self.toggle_radiometric_mode(enabled)
    
    def eventFilter(self, obj, event):
        """
        Filter events for the image frame.
        
        Args:
            obj: Object that received the event.
            event: Event that was received.
        
        Returns:
            bool: True if the event was handled, False otherwise.
        """
        if obj == self.image_frame:
            if event.type() == event.MouseButtonPress:
                return self._handle_mouse_press(event)
            elif event.type() == event.MouseMove:
                return self._handle_mouse_move(event)
            elif event.type() == event.Wheel:
                return self._handle_mouse_wheel(event)
        
        # Let the base class handle the event
        return super().eventFilter(obj, event)
    
    def _handle_mouse_press(self, event: QMouseEvent) -> bool:
        """
        Handle mouse press events on the image frame.
        
        Args:
            event (QMouseEvent): Mouse press event.
        
        Returns:
            bool: True if the event was handled, False otherwise.
        """
        if event.button() == Qt.LeftButton:
            # Get mouse position relative to the image
            pos = event.pos()
            
            # Check if we have an image
            if self.display_image is None or self.raw_frame is None:
                return True
            
            # Calculate coordinates in the original image
            x = int(pos.x() / self.zoom_factor)
            y = int(pos.y() / self.zoom_factor)
            
            # Check if within image bounds
            if 0 <= x < self.raw_frame.shape[1] and 0 <= y < self.raw_frame.shape[0]:
                # Get raw thermal value at the selected pixel
                raw_value = int(self.raw_frame[y, x])
                
                # Update selected pixel
                self.selected_pixel = (x, y)
                
                # Emit signal with pixel information
                self.pixel_selected.emit(x, y, raw_value)
                
                # Update pixel info label
                self.pixel_info_label.setText(f"Pixel: ({x}, {y})")
                
                # Update temperature info if in radiometric mode
                if self.radiometric_mode and self.calibration_model.is_calibrated:
                    try:
                        temp = self.calibration_model.raw_to_temperature(raw_value)
                        self.temp_info_label.setText(f"Temperature: {temp:.1f}°C")
                    except Exception as e:
                        logger.error(f"Error calculating temperature: {e}")
                        self.temp_info_label.setText("Temperature: Error")
                else:
                    self.temp_info_label.setText(f"Raw Value: {raw_value}")
                
                # Update display
                self._update_display()
                
                return True
        
        return False
    
    def _handle_mouse_move(self, event: QMouseEvent) -> bool:
        """
        Handle mouse move events on the image frame.
        
        Args:
            event (QMouseEvent): Mouse move event.
        
        Returns:
            bool: True if the event was handled, False otherwise.
        """
        # Get mouse position relative to the image
        pos = event.pos()
        
        # Check if we have an image
        if self.display_image is None or self.raw_frame is None:
            return True
        
        # Calculate coordinates in the original image
        x = int(pos.x() / self.zoom_factor)
        y = int(pos.y() / self.zoom_factor)
        
        # Check if within image bounds
        if 0 <= x < self.raw_frame.shape[1] and 0 <= y < self.raw_frame.shape[0]:
            # Update highlighted pixel
            self.highlighted_pixel = (x, y)
            
            # Update pixel info label
            self.pixel_info_label.setText(f"Pixel: ({x}, {y})")
            
            # Update temperature info if in radiometric mode
            if self.radiometric_mode and self.calibration_model.is_calibrated:
                try:
                    raw_value = int(self.raw_frame[y, x])
                    temp = self.calibration_model.raw_to_temperature(raw_value)
                    self.temp_info_label.setText(f"Temperature: {temp:.1f}°C")
                except Exception as e:
                    logger.error(f"Error calculating temperature: {e}")
                    self.temp_info_label.setText("Temperature: Error")
            else:
                raw_value = int(self.raw_frame[y, x])
                self.temp_info_label.setText(f"Raw Value: {raw_value}")
            
            # Update display
            self._update_display()
            
            return True
        else:
            # Reset highlighted pixel when outside image bounds
            self.highlighted_pixel = None
            self._update_display()
        
        return False
    
    def _handle_mouse_wheel(self, event: QWheelEvent) -> bool:
        """
        Handle mouse wheel events for zooming.
        
        Args:
            event (QWheelEvent): Mouse wheel event.
        
        Returns:
            bool: True if the event was handled, False otherwise.
        """
        # Get mouse wheel delta (positive for zoom in, negative for zoom out)
        delta = event.angleDelta().y()
        
        # Calculate new zoom factor (each wheel step changes zoom by 10%)
        zoom_step = 0.1
        new_zoom = self.zoom_factor + (zoom_step if delta > 0 else -zoom_step)
        
        # Limit zoom range
        new_zoom = max(0.1, min(3.0, new_zoom))
        
        # Update zoom slider
        self.zoom_slider.setValue(int(new_zoom * 100))
        
        return True
    
    def _on_zoom_changed(self, value: int):
        """
        Handle zoom slider value changes.
        
        Args:
            value (int): New zoom slider value.
        """
        # Calculate zoom factor (slider value is percentage)
        self.zoom_factor = value / 100.0
        
        # Update zoom label
        self.zoom_label.setText(f"{value}%")
        
        # Update display
        self._update_display()
    
    def _reset_view(self):
        """Reset view to default settings."""
        # Reset zoom
        self.zoom_factor = 1.0
        self.zoom_slider.setValue(100)
        
        # Reset pan
        self.pan_offset = QPoint(0, 0)
        
        # Update display
        self._update_display()
    
    @pyqtSlot()
    def on_calibration_updated(self):
        """Handle calibration model updates."""
        # Update display to show calibration points
        self._update_display()