#!/usr/bin/env python3

import os
import sys
import numpy as np
from datetime import datetime

import cv2
from cv_bridge import CvBridge

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QImage, QPixmap, QPen, QColor
from python_qt_binding.QtWidgets import (QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
                                         QLabel, QSplitter, QTableWidget, QTableWidgetItem,
                                         QHeaderView, QMessageBox, QInputDialog, QDoubleSpinBox,
                                         QComboBox, QFileDialog)

from qt_gui.plugin import Plugin
from rqt_gui_py.plugin import Plugin as PyPlugin

from sensor_msgs.msg import Image
from thermal_calibration_interfaces.srv import (
    GetRawValue, AddCalibrationPoint, PerformCalibration, ClearCalibrationData, 
    RawToTemperature, SaveCalibrationModel, LoadCalibrationModel
)

class ThermalImageView(QLabel):
    """Custom widget for displaying the thermal image with interactive features."""
    
    pixel_clicked = Signal(int, int)  # Signal emitted when a pixel is clicked (x, y)
    
    def __init__(self, parent=None):
        super(ThermalImageView, self).__init__(parent)
        self.selected_point = None
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(640, 480)  # Increase minimum size
        self.setCursor(Qt.CrossCursor)
        
        # Set better size policy to maximize use of available space
        from python_qt_binding.QtWidgets import QSizePolicy
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
    def mousePressEvent(self, event):
        """Handle mouse press events to select a pixel."""
        if event.button() == Qt.LeftButton and self.pixmap() is not None:
            # Convert from widget coordinates to image coordinates
            pos = event.pos()
            img_pos = self._map_to_image(pos.x(), pos.y())
            if img_pos:
                x, y = img_pos
                self.selected_point = (x, y)
                self.pixel_clicked.emit(x, y)
                self.update()  # Trigger repaint to show the selection
    
    def _map_to_image(self, widget_x, widget_y):
        """Map widget coordinates to image coordinates."""
        if not self.pixmap() or self.pixmap().isNull():
            return None
            
        # Get the image rectangle within the label
        img_rect = self._get_image_rect()
        if not img_rect:
            return None
            
        # Check if click is within the image bounds
        if (widget_x < img_rect.left() or widget_x >= img_rect.right() or
            widget_y < img_rect.top() or widget_y >= img_rect.bottom()):
            return None
            
        # Calculate normalized coordinates (0-1) within the displayed image
        norm_x = (widget_x - img_rect.left()) / img_rect.width()
        norm_y = (widget_y - img_rect.top()) / img_rect.height()
        
        # Map to actual image coordinates
        img_x = int(norm_x * self.pixmap().width())
        img_y = int(norm_y * self.pixmap().height())
        
        return (img_x, img_y)
    
    def _get_image_rect(self):
        """Get the rectangle where the image is displayed within the label."""
        if not self.pixmap() or self.pixmap().isNull():
            return None
            
        # Calculate scaled image size
        scaledSize = self.pixmap().size()
        scaledSize.scale(self.size(), Qt.KeepAspectRatio)
        
        # Calculate image position (centered in label)
        x = (self.width() - scaledSize.width()) / 2
        y = (self.height() - scaledSize.height()) / 2
        
        from python_qt_binding.QtCore import QRect
        return QRect(int(x), int(y), scaledSize.width(), scaledSize.height())
    
    def paintEvent(self, event):
        """Override paint event to draw selection marker."""
        super(ThermalImageView, self).paintEvent(event)
        
        if self.selected_point and self.pixmap() and not self.pixmap().isNull():
            from python_qt_binding.QtGui import QPainter
            painter = QPainter(self)
            
            # Get image rectangle
            img_rect = self._get_image_rect()
            if not img_rect:
                return
                
            # Map image coordinates to widget coordinates
            img_x, img_y = self.selected_point
            norm_x = img_x / self.pixmap().width()
            norm_y = img_y / self.pixmap().height()
            
            widget_x = img_rect.left() + norm_x * img_rect.width()
            widget_y = img_rect.top() + norm_y * img_rect.height()
            
            # Draw crosshair
            painter.setPen(QPen(QColor(255, 255, 0), 2))
            radius = 10
            painter.drawLine(int(widget_x - radius), int(widget_y), int(widget_x + radius), int(widget_y))
            painter.drawLine(int(widget_x), int(widget_y - radius), int(widget_x), int(widget_y + radius))
            
            # Draw circle
            painter.setPen(QPen(QColor(255, 255, 0), 1))
            painter.drawEllipse(int(widget_x - radius), int(widget_y - radius), radius * 2, radius * 2)


class ThermalCalibrationPlugin(PyPlugin):
    """
    RQT plugin for thermal camera calibration.
    
    This plugin allows users to:
    1. View the thermal camera feed
    2. Select points on the image
    3. Enter reference temperature measurements
    4. Calibrate the camera to map raw values to temperatures
    5. View calibrated temperature values
    """
    
    def __init__(self, context):
        """Initialize the plugin."""
        super(ThermalCalibrationPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ThermalCalibrationPlugin')
        
        # Get the ROS 2 node from the plugin context
        self._node = context.node
        
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        
        # Create the main widget
        self._widget = QWidget()
        self._widget.setObjectName('ThermalCalibrationPluginUi')
        self._widget.setWindowTitle('Thermal Camera Calibration')
        
        # Initialize instance variables
        self.cv_bridge = CvBridge()
        self.current_image = None
        self.current_raw_value = 0
        self.calibration_points = []
        self.calibration_model = None
        self.selected_coords = None
        self.radiometric_mode = False
        
        # Initialize UI
        self._init_ui()
        
        # Initialize ROS communication
        self._setup_ros_communication()
        
        # Add widget to the user interface
        context.add_widget(self._widget)
        
    def _init_ui(self):
        """Initialize the user interface."""
        # Create main layout
        main_layout = QHBoxLayout()
        self._widget.setLayout(main_layout)
        
        # Create splitter for left and right panel
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # Left panel (thermal image)
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)  # Reduce margins

        # Thermal image view
        self.image_view = ThermalImageView()
        left_layout.addWidget(self.image_view, 1)  # Give it a stretch factor of 1
        
        # Connect pixel_clicked signal
        self.image_view.pixel_clicked.connect(self._on_pixel_clicked)
        
        # Add information labels under the image
        img_info_layout = QHBoxLayout()
        self.coords_label = QLabel("Coordinates: -")
        img_info_layout.addWidget(self.coords_label)
        
        self.raw_value_label = QLabel("Raw value: -")
        img_info_layout.addWidget(self.raw_value_label)
        
        self.temp_label = QLabel("Temperature: -")
        img_info_layout.addWidget(self.temp_label)
        
        left_layout.addLayout(img_info_layout)
        
        # Right panel (controls)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # Top section - Controls for entering reference temperatures
        controls_group = QWidget()
        controls_layout = QVBoxLayout(controls_group)

        # Colormap selection
        colormap_layout = QHBoxLayout()
        colormap_layout.addWidget(QLabel("Colormap:"))
        self.colormap_combo = QComboBox()
        self.colormap_combo.addItems(["Grayscale", "Inferno", "Jet", "Viridis", "Rainbow"])
        self.colormap_combo.setCurrentText("Grayscale")  # Set default to grayscale
        self.colormap_combo.currentTextChanged.connect(self._on_colormap_changed)
        colormap_layout.addWidget(self.colormap_combo)
        controls_layout.addLayout(colormap_layout)
        
        # Button to enter temperature
        self.enter_temp_btn = QPushButton("Enter temperature value")
        self.enter_temp_btn.setEnabled(False)
        self.enter_temp_btn.clicked.connect(self._on_enter_temp_clicked)
        controls_layout.addWidget(self.enter_temp_btn)
        
        # Temperature input layout (initially hidden)
        self.temp_input_layout = QHBoxLayout()
        
        self.temp_input = QDoubleSpinBox()
        self.temp_input.setRange(-50.0, 500.0)
        self.temp_input.setDecimals(1)
        self.temp_input.setSuffix(" °C")
        self.temp_input_layout.addWidget(self.temp_input)
        
        self.save_temp_btn = QPushButton("Save")
        self.save_temp_btn.clicked.connect(self._on_save_temp_clicked)
        self.temp_input_layout.addWidget(self.save_temp_btn)
        
        self.cancel_temp_btn = QPushButton("Cancel")
        self.cancel_temp_btn.clicked.connect(self._on_cancel_temp_clicked)
        self.temp_input_layout.addWidget(self.cancel_temp_btn)
        
        # Initially hide temperature input controls
        self.temp_input_widget = QWidget()
        self.temp_input_widget.setLayout(self.temp_input_layout)
        self.temp_input_widget.setVisible(False)
        controls_layout.addWidget(self.temp_input_widget)
        
        right_layout.addWidget(controls_group)
        
        # Bottom section - Calibration points and controls
        calibration_group = QWidget()
        calibration_layout = QVBoxLayout(calibration_group)
        
        # Table of calibration points
        self.points_table = QTableWidget(0, 4)
        self.points_table.setHorizontalHeaderLabels(["ID", "Coords", "Raw Value", "Temp (°C)"])
        self.points_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        calibration_layout.addWidget(self.points_table)
        
        # Calibration controls layout
        cal_controls_layout = QHBoxLayout()
        
        # Model type selection
        cal_controls_layout.addWidget(QLabel("Model:"))
        
        self.model_type_combo = QComboBox()
        self.model_type_combo.addItem("Polynomial")
        cal_controls_layout.addWidget(self.model_type_combo)
        
        # Degree selection (for polynomial)
        cal_controls_layout.addWidget(QLabel("Degree:"))
        
        self.degree_spin = QDoubleSpinBox()
        self.degree_spin.setRange(1, 5)
        self.degree_spin.setDecimals(0)
        self.degree_spin.setValue(2)
        cal_controls_layout.addWidget(self.degree_spin)
        
        # Calibrate button
        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.clicked.connect(self._on_calibrate_clicked)
        cal_controls_layout.addWidget(self.calibrate_btn)
        
        calibration_layout.addLayout(cal_controls_layout)
        
        # Calibration results layout
        cal_results_layout = QVBoxLayout()
        
        # Results text
        self.cal_results_label = QLabel("No calibration performed yet")
        cal_results_layout.addWidget(self.cal_results_label)
        
        # Export/Clear buttons layout
        export_clear_layout = QHBoxLayout()
        
        # Export button
        self.export_btn = QPushButton("Export Calibration")
        self.export_btn.clicked.connect(self._on_export_clicked)
        self.export_btn.setEnabled(False)
        export_clear_layout.addWidget(self.export_btn)
        
        # Clear button
        self.clear_btn = QPushButton("Clear Data")
        self.clear_btn.clicked.connect(self._on_clear_clicked)
        export_clear_layout.addWidget(self.clear_btn)
        
        cal_results_layout.addLayout(export_clear_layout)
        
        # Radiometric toggle
        self.radio_toggle = QPushButton("Enable Radiometric Mode")
        self.radio_toggle.setCheckable(True)
        self.radio_toggle.toggled.connect(self._on_radio_toggled)
        self.radio_toggle.setEnabled(False)
        cal_results_layout.addWidget(self.radio_toggle)
        
        calibration_layout.addLayout(cal_results_layout)
        
        right_layout.addWidget(calibration_group)
        
        # Add panels to splitter
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)

        # Set initial splitter sizes (70% left, 30% right)
        splitter.setSizes([700, 300])  # Change from [400, 600] to give more space to the image
        
        # Create timer for regular UI updates
        self.update_timer = QTimer(self._widget)
        self.update_timer.timeout.connect(self._update_ui)
        self.update_timer.start(100)  # Update every 100ms
        
    def _setup_ros_communication(self):
        """Set up ROS subscribers and service clients."""
        # Image subscribers for both 16-bit and 8-bit thermal streams
        self.image_16bit_sub = self._node.create_subscription(
            Image,
            '/mono16_converter/image',  # 16-bit thermal image used for calibration
            self._image_16bit_callback,
            10
        )
        
        self.image_8bit_sub = self._node.create_subscription(
            Image,
            '/image_raw/mono8',  # 8-bit visualization stream
            self._image_8bit_callback,
            10
        )
        
        # Service clients
        self.get_raw_value_client = self._node.create_client(
            GetRawValue, 'get_raw_value')
        self.add_calibration_point_client = self._node.create_client(
            AddCalibrationPoint, 'add_calibration_point')
        self.perform_calibration_client = self._node.create_client(
            PerformCalibration, 'perform_calibration')
        self.clear_calibration_data_client = self._node.create_client(
            ClearCalibrationData, 'clear_calibration_data')
        self.raw_to_temperature_client = self._node.create_client(
            RawToTemperature, 'raw_to_temperature')
        self.save_calibration_model_client = self._node.create_client(
            SaveCalibrationModel, 'save_calibration_model')
        self.load_calibration_model_client = self._node.create_client(
            LoadCalibrationModel, 'load_calibration_model')
        
        # Wait for services to be available
        self._node.get_logger().info('Waiting for thermal calibration services...')
        
    def _image_16bit_callback(self, msg):
        """Callback for 16-bit thermal image used for calibration."""
        try:
            # Convert ROS image message to OpenCV image
            self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
            
            # Process this image for display if we don't have an 8-bit stream
            if not hasattr(self, 'has_8bit_stream') or not self.has_8bit_stream:
                self._update_display_from_16bit()
            
            # If radiometric mode is on and we have a model, update temperature at selected point
            if self.radiometric_mode and self.calibration_model and self.selected_coords:
                x, y = self.selected_coords
                if 0 <= y < self.current_image.shape[0] and 0 <= x < self.current_image.shape[1]:
                    raw_value = int(self.current_image[y, x])
                    self._update_temperature_display(raw_value)
        
        except Exception as e:
            self._node.get_logger().error(f'Error processing 16-bit image: {e}')
    
    def _image_8bit_callback(self, msg):
        """Callback for 8-bit thermal image used for visualization."""
        try:
            # Set flag that we have 8-bit stream
            self.has_8bit_stream = True
            
            # Convert ROS image message to OpenCV image
            img_8bit = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            
            # Store the image for colormap changes
            self.last_8bit_image = img_8bit
            
            # Update the display with the current image
            self._update_image_display(img_8bit)
        
        except Exception as e:
            self._node.get_logger().error(f'Error processing 8-bit image: {e}')

    def _update_image_display(self, img_8bit):
        """Update the image display with the given 8-bit image using the selected colormap."""
        try:
            # Apply selected colormap
            if not hasattr(self, 'current_colormap'):
                self.current_colormap = "Grayscale"
                
            if self.current_colormap == "Grayscale":
                # For grayscale, we don't apply a colormap, but convert to RGB
                colored_img = cv2.cvtColor(img_8bit, cv2.COLOR_GRAY2BGR)
            else:
                # Map colormap name to OpenCV constant
                colormap_map = {
                    "Inferno": cv2.COLORMAP_INFERNO,
                    "Jet": cv2.COLORMAP_JET,
                    "Viridis": cv2.COLORMAP_VIRIDIS,
                    "Rainbow": cv2.COLORMAP_RAINBOW
                }
                colormap = colormap_map.get(self.current_colormap, cv2.COLORMAP_INFERNO)
                colored_img = cv2.applyColorMap(img_8bit, colormap)
            
            # Convert OpenCV image to QImage then QPixmap for display
            h, w, c = colored_img.shape
            q_img = QImage(colored_img.data, w, h, w * c, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img)
            
            # Scale the pixmap to fit the view while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(
                self.image_view.width(), 
                self.image_view.height(),
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            
            # Update image view
            self.image_view.setPixmap(scaled_pixmap)
        
        except Exception as e:
            self._node.get_logger().error(f'Error updating image display: {e}')
    
    def _update_display_from_16bit(self):
        """Update the display using the 16-bit image when 8-bit stream is not available."""
        if self.current_image is None:
            return
            
        try:
            # Convert to display image (apply colormap)
            display_img = cv2.normalize(self.current_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            colored_img = cv2.applyColorMap(display_img, cv2.COLORMAP_INFERNO)
            
            # Convert OpenCV image to QImage then QPixmap for display
            h, w, c = colored_img.shape
            q_img = QImage(colored_img.data, w, h, w * c, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img)
            
            # Update image view
            self.image_view.setPixmap(pixmap)
        
        except Exception as e:
            self._node.get_logger().error(f'Error updating display from 16-bit: {e}')
    
    def _on_pixel_clicked(self, x, y):
        """Handle pixel selection in the image."""
        if self.current_image is None:
            self._node.get_logger().warn('No thermal image available')
            return
            
        try:
            self.selected_coords = (x, y)
            
            # Update coordinate display
            self.coords_label.setText(f"Coordinates: ({x}, {y})")
            
            # Debug output
            self._node.get_logger().info(f'Click at coordinates ({x}, {y})')
            self._node.get_logger().info(f'Image shape: {self.current_image.shape}')
            
            # Get raw value from the image
            if 0 <= y < self.current_image.shape[0] and 0 <= x < self.current_image.shape[1]:
                raw_value = int(self.current_image[y, x])
                self.current_raw_value = raw_value
                self.raw_value_label.setText(f"Raw value: {raw_value}")
                
                # Force enable the button to enter temperature
                self.enter_temp_btn.setEnabled(True)
                self._node.get_logger().info(f'Enter temp button enabled: {self.enter_temp_btn.isEnabled()}')
                
                # If in radiometric mode, update temperature display
                if self.radiometric_mode and self.calibration_model:
                    self._update_temperature_display(raw_value)
                else:
                    self.temp_label.setText("Temperature: -")
            else:
                self._node.get_logger().warn(f'Coordinates ({x}, {y}) out of bounds. Image shape: {self.current_image.shape}')
        
        except Exception as e:
            self._node.get_logger().error(f'Error handling pixel click: {e}')
    
    def _update_temperature_display(self, raw_value):
        """Update the temperature display for a given raw value using the current calibration model."""
        self._call_raw_to_temperature(raw_value)
    
    def _on_enter_temp_clicked(self):
        """Handle click on enter temperature button."""
        # Show temperature input controls
        self.temp_input_widget.setVisible(True)
        self.enter_temp_btn.setEnabled(False)
    
    def _on_save_temp_clicked(self):
        """Handle click on save temperature button."""
        if self.selected_coords is None:
            QMessageBox.warning(self._widget, "No Selection", 
                               "Please select a point on the image first.")
            return
            
        # Get reference temperature from input
        reference_temp = self.temp_input.value()
        
        # Add calibration point
        x, y = self.selected_coords
        self._call_add_calibration_point(x, y, self.current_raw_value, reference_temp)
        
        # Hide temperature input controls
        self.temp_input_widget.setVisible(False)
        self.enter_temp_btn.setEnabled(True)
    
    def _on_cancel_temp_clicked(self):
        """Handle click on cancel temperature button."""
        # Hide temperature input controls
        self.temp_input_widget.setVisible(False)
        self.enter_temp_btn.setEnabled(True)
    
    def _on_calibrate_clicked(self):
        """Handle click on calibrate button."""
        if len(self.calibration_points) < 2:
            QMessageBox.warning(self._widget, "Insufficient Data", 
                               "At least 2 calibration points are required.")
            return
            
        # Get model type and degree
        model_type = self.model_type_combo.currentText().lower()
        degree = int(self.degree_spin.value())
        
        # Perform calibration
        self._call_perform_calibration(model_type, degree)
    
    def _on_export_clicked(self):
        """Handle click on export calibration button."""
        filename, _ = QFileDialog.getSaveFileName(
            self._widget, "Save Calibration Model", "", "JSON Files (*.json)"
        )
        
        if filename:
            # If user didn't add .json extension, add it
            if not filename.endswith('.json'):
                filename += '.json'
                
            # Call service to save calibration model
            self._call_save_calibration_model(filename)
    
    def _on_clear_clicked(self):
        """Handle click on clear data button."""
        reply = QMessageBox.question(
            self._widget, "Clear Data", 
            "Are you sure you want to clear all calibration data?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Call service to clear calibration data
            self._call_clear_calibration_data()
    
    def _on_radio_toggled(self, checked):
        """Handle toggle of radiometric mode."""
        self.radiometric_mode = checked
        
        # Update button text
        if checked:
            self.radio_toggle.setText("Disable Radiometric Mode")
        else:
            self.radio_toggle.setText("Enable Radiometric Mode")
            self.temp_label.setText("Temperature: -")
    
    def _update_ui(self):
        """Periodically update UI elements."""
        # This method can be used for any UI updates that need to happen regularly
        pass
    
    def _update_points_table(self):
        """Update the calibration points table."""
        # Clear table
        self.points_table.setRowCount(0)
        
        # Add rows for each calibration point
        for point in self.calibration_points:
            row = self.points_table.rowCount()
            self.points_table.insertRow(row)
            
            # Set ID
            id_item = QTableWidgetItem(str(point['id']))
            self.points_table.setItem(row, 0, id_item)
            
            # Set coordinates
            coords_item = QTableWidgetItem(f"({point['x']}, {point['y']})")
            self.points_table.setItem(row, 1, coords_item)
            
            # Set raw value
            raw_item = QTableWidgetItem(str(point['raw_value']))
            self.points_table.setItem(row, 2, raw_item)
            
            # Set temperature
            temp_item = QTableWidgetItem(f"{point['reference_temp']:.1f}")
            self.points_table.setItem(row, 3, temp_item)
    
    def _update_calibration_results(self):
        """Update the calibration results display."""
        if self.calibration_model:
            model_type = self.calibration_model['model_type']
            r_squared = self.calibration_model['r_squared']
            rmse = self.calibration_model['rmse']
            
            result_text = (f"Calibration Model: {model_type.title()}\n"
                          f"R²: {r_squared:.4f}\n"
                          f"RMSE: {rmse:.2f}°C")
            
            self.cal_results_label.setText(result_text)
            
            # Enable export and radiometric mode buttons
            self.export_btn.setEnabled(True)
            self.radio_toggle.setEnabled(True)
        else:
            self.cal_results_label.setText("No calibration performed yet")
            self.export_btn.setEnabled(False)
            self.radio_toggle.setEnabled(False)

    def _on_colormap_changed(self, colormap_name):
        """Handle change in colormap selection."""
        self.current_colormap = colormap_name
        # Refresh the display if we have an image
        if hasattr(self, 'has_8bit_stream') and self.has_8bit_stream and hasattr(self, 'last_8bit_image'):
            self._update_image_display(self.last_8bit_image)
        elif self.current_image is not None:
            self._update_display_from_16bit()

    # Service call methods
    
    def _call_get_raw_value(self, x, y):
        """Call the get_raw_value service."""
        if not self.get_raw_value_client.service_is_ready():
            self._node.get_logger().warn('get_raw_value service not available')
            return
            
        request = GetRawValue.Request()
        request.x = x
        request.y = y
        
        future = self.get_raw_value_client.call_async(request)
        future.add_done_callback(self._get_raw_value_done)
    
    def _get_raw_value_done(self, future):
        """Callback for get_raw_value service response."""
        try:
            response = future.result()
            if response.success:
                self.current_raw_value = response.raw_value
                self.raw_value_label.setText(f"Raw value: {response.raw_value}")
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
    
    def _call_add_calibration_point(self, x, y, raw_value, reference_temp):
        """Call the add_calibration_point service."""
        if not self.add_calibration_point_client.service_is_ready():
            self._node.get_logger().warn('add_calibration_point service not available')
            return
            
        request = AddCalibrationPoint.Request()
        request.x = x
        request.y = y
        request.raw_value = raw_value
        request.reference_temp = reference_temp
        
        future = self.add_calibration_point_client.call_async(request)
        future.add_done_callback(self._add_calibration_point_done)
    
    def _add_calibration_point_done(self, future):
        """Callback for add_calibration_point service response."""
        try:
            response = future.result()
            if response.success:
                # Add point to local list
                point = {
                    'id': response.point_id,
                    'x': self.selected_coords[0],
                    'y': self.selected_coords[1],
                    'raw_value': self.current_raw_value,
                    'reference_temp': self.temp_input.value(),
                    'timestamp': datetime.now().isoformat()
                }
                self.calibration_points.append(point)
                
                # Update UI
                self._update_points_table()
                QMessageBox.information(self._widget, "Success", response.message)
            else:
                QMessageBox.warning(self._widget, "Error", response.message)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            QMessageBox.critical(self._widget, "Error", f"Service call failed: {e}")
    
    def _call_perform_calibration(self, model_type, degree):
        """Call the perform_calibration service."""
        if not self.perform_calibration_client.service_is_ready():
            self._node.get_logger().warn('perform_calibration service not available')
            return
            
        request = PerformCalibration.Request()
        request.model_type = model_type
        request.degree = degree
        
        future = self.perform_calibration_client.call_async(request)
        future.add_done_callback(self._perform_calibration_done)
    
    def _perform_calibration_done(self, future):
        """Callback for perform_calibration service response."""
        try:
            response = future.result()
            if response.success:
                # Store calibration model
                self.calibration_model = {
                    'model_type': self.model_type_combo.currentText().lower(),
                    'degree': int(self.degree_spin.value()),
                    'parameters': list(response.model_parameters),
                    'r_squared': response.r_squared,
                    'rmse': response.rmse,
                    'timestamp': datetime.now().isoformat()
                }
                
                # Update UI
                self._update_calibration_results()
                QMessageBox.information(self._widget, "Success", response.message)
            else:
                QMessageBox.warning(self._widget, "Calibration Error", response.message)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            QMessageBox.critical(self._widget, "Error", f"Service call failed: {e}")
    
    def _call_clear_calibration_data(self):
        """Call the clear_calibration_data service."""
        if not self.clear_calibration_data_client.service_is_ready():
            self._node.get_logger().warn('clear_calibration_data service not available')
            return
            
        request = ClearCalibrationData.Request()
        request.confirm = True
        
        future = self.clear_calibration_data_client.call_async(request)
        future.add_done_callback(self._clear_calibration_data_done)
    
    def _clear_calibration_data_done(self, future):
        """Callback for clear_calibration_data service response."""
        try:
            response = future.result()
            if response.success:
                # Clear local data
                self.calibration_points = []
                self.calibration_model = None
                
                # Update UI
                self._update_points_table()
                self._update_calibration_results()
                
                # Disable radiometric mode if it's enabled
                if self.radiometric_mode:
                    self.radio_toggle.setChecked(False)
                
                QMessageBox.information(self._widget, "Success", response.message)
            else:
                QMessageBox.warning(self._widget, "Error", response.message)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            QMessageBox.critical(self._widget, "Error", f"Service call failed: {e}")
    
    def _call_raw_to_temperature(self, raw_value):
        """Call the raw_to_temperature service."""
        if not self.raw_to_temperature_client.service_is_ready():
            self._node.get_logger().warn('raw_to_temperature service not available')
            return
            
        request = RawToTemperature.Request()
        request.raw_value = raw_value
        
        future = self.raw_to_temperature_client.call_async(request)
        future.add_done_callback(self._raw_to_temperature_done)
    
    def _raw_to_temperature_done(self, future):
        """Callback for raw_to_temperature service response."""
        try:
            response = future.result()
            if response.success:
                temp = response.temperature
                self.temp_label.setText(f"Temperature: {temp:.1f}°C")
            else:
                self.temp_label.setText("Temperature: -")
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
    
    def _call_save_calibration_model(self, filename):
        """Call the save_calibration_model service."""
        if not self.save_calibration_model_client.service_is_ready():
            self._node.get_logger().warn('save_calibration_model service not available')
            return
            
        request = SaveCalibrationModel.Request()
        request.filename = os.path.basename(filename)
        
        future = self.save_calibration_model_client.call_async(request)
        future.add_done_callback(self._save_calibration_model_done)
    
    def _save_calibration_model_done(self, future):
        """Callback for save_calibration_model service response."""
        try:
            response = future.result()
            if response.success:
                QMessageBox.information(self._widget, "Success", response.message)
            else:
                QMessageBox.warning(self._widget, "Error", response.message)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            QMessageBox.critical(self._widget, "Error", f"Service call failed: {e}")
    
    def _call_load_calibration_model(self, path):
        """Call the load_calibration_model service."""
        if not self.load_calibration_model_client.service_is_ready():
            self._node.get_logger().warn('load_calibration_model service not available')
            return
            
        request = LoadCalibrationModel.Request()
        request.path = path
        
        future = self.load_calibration_model_client.call_async(request)
        future.add_done_callback(self._load_calibration_model_done)
    
    def _load_calibration_model_done(self, future):
        """Callback for load_calibration_model service response."""
        try:
            response = future.result()
            if response.success:
                # Create a calibration model from the response
                self.calibration_model = {
                    'model_type': response.model_type,
                    'parameters': list(response.model_parameters),
                    'r_squared': 0.0,  # Not provided in response
                    'rmse': 0.0,       # Not provided in response
                    'timestamp': datetime.now().isoformat()
                }
                
                # Update UI
                self._update_calibration_results()
                QMessageBox.information(self._widget, "Success", response.message)
            else:
                QMessageBox.warning(self._widget, "Error", response.message)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            QMessageBox.critical(self._widget, "Error", f"Service call failed: {e}")
    
    def shutdown_plugin(self):
        """
        Callback when the plugin is closed.
        Unregisters all publishers and subscribers.
        """
        # Unregister subscribers
        self.image_16bit_sub = None
        self.image_8bit_sub = None
        
        # Stop the timer
        self.update_timer.stop()
    
    def save_settings(self, plugin_settings, instance_settings):
        """Save the intrinsic configuration of the plugin."""
        # You can save intrinsic configuration here
        pass
    
    def restore_settings(self, plugin_settings, instance_settings):
        """Restore the intrinsic configuration of the plugin."""
        # You can restore intrinsic configuration here
        pass


def main(args=None):
    """Main function to allow standalone operation of the plugin."""
    # For standalone use, we need to initialize ROS here, but
    # when run as an RQT plugin, this initialization is done by RQT
    rclpy.init(args=args)
    from rqt_gui.main import Main
    main = Main()
    sys.exit(main.main(sys.argv, standalone='thermal_calibration_rqt.thermal_calibration_plugin:ThermalCalibrationPlugin'))