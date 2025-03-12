#!/usr/bin/env python3

import os
import sys
import numpy as np
from datetime import datetime

import cv2
from cv_bridge import CvBridge

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QObject, QMutex, QMutexLocker, pyqtSignal, QRect

from python_qt_binding.QtGui import QImage, QPixmap, QPen, QColor, QPainter, QFontMetrics
from python_qt_binding.QtWidgets import (QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
                                         QLabel, QSplitter, QTableWidget, QTableWidgetItem,
                                         QHeaderView, QMessageBox, QInputDialog, QDoubleSpinBox,
                                         QComboBox, QFileDialog, QStyle, QSizePolicy, QProgressDialog)

import traceback
import time
import threading
from functools import partial
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSHistoryPolicy

from qt_gui.plugin import Plugin
from rqt_gui_py.plugin import Plugin as PyPlugin

from sensor_msgs.msg import Image
from thermal_calibration_interfaces.srv import (
    GetRawValue, AddCalibrationPoint, PerformCalibration, ClearCalibrationData, 
    RawToTemperature, SaveCalibrationModel, LoadCalibrationModel
)

class SignalHelper(QObject):
    """Helper class to hold Qt signals."""
    image_update_signal = pyqtSignal(object)
    raw_value_update_signal = pyqtSignal(str)
    temp_update_signal = pyqtSignal(str)
    points_table_update_signal = pyqtSignal()
    cal_results_update_signal = pyqtSignal()
    timer_cleanup_signal = pyqtSignal(str)
    calibration_complete_signal = pyqtSignal(object)  # For calibration done with future
    calibration_progress_signal = pyqtSignal(bool)  # For showing/hiding progress dialog
    ui_button_signal = pyqtSignal(str, bool, str)  # For updating buttons (name, enabled, text)

class ThermalImageOverlay(QWidget):
    """
    Transparent overlay for the thermal image view.
    Handles all user interactions and visual feedback.
    """
    
    pixel_clicked = Signal(int, int)  # Signal emitted when a pixel is clicked (x, y)
    
    def __init__(self, parent=None):
        super(ThermalImageOverlay, self).__init__(parent)
        
        # Make the widget transparent for mouse events to pass through
        self.setAttribute(Qt.WA_TransparentForMouseEvents, False)
        
        # Make the widget background transparent
        self.setAttribute(Qt.WA_TranslucentBackground)
        
        # Enable mouse tracking for hover effects
        self.setMouseTracking(True)
        
        # Initialize variables
        self.selected_point = None
        self.calibration_points = []  # List to store saved calibration points [(x, y, temp, raw_value), ...]
        self.hover_point = None  # Store mouse position for hover effects
        
        # Set cursor
        self.setCursor(Qt.CrossCursor)
        
        # Image dimensions (these will be updated when the parent image view updates)
        self.image_width = 0
        self.image_height = 0
        self.image_rect = None  # The rectangle where the image is displayed
    
    def update_image_dimensions(self, width, height, rect):
        """Update the dimensions of the underlying image."""
        # Safety checks to prevent exceptions
        if width <= 0 or height <= 0 or rect is None:
            return
            
        self.image_width = width
        self.image_height = height
        self.image_rect = rect
        self.update()  # Redraw overlay
    
    def add_calibration_point(self, x, y, temp, raw_value):
        """Add a point to the calibration points list."""
        self.calibration_points.append((x, y, temp, raw_value))
        self.update()  # Trigger repaint
        
    def remove_calibration_point(self, index):
        """Remove a point from the calibration points list."""
        if 0 <= index < len(self.calibration_points):
            removed_point = self.calibration_points[index]
            del self.calibration_points[index]
            self.update()  # Trigger repaint
            print(f"Removed point at ({removed_point[0]}, {removed_point[1]}) from overlay")
            return True
        else:
            print(f"Error: Cannot remove point at index {index}. Have {len(self.calibration_points)} points.")
            return False
    
    def clear_calibration_points(self):
        """Clear all calibration points."""
        self.calibration_points = []
        self.update()  # Trigger repaint
    
    def mousePressEvent(self, event):
        """Handle mouse press events to select a pixel."""
        if event.button() == Qt.LeftButton and self.image_rect is not None:
            # Convert from widget coordinates to image coordinates
            img_pos = self._map_to_image(event.pos().x(), event.pos().y())
            if img_pos:
                x, y = img_pos
                self.selected_point = (x, y)
                self.pixel_clicked.emit(x, y)
                self.update()  # Trigger repaint to show the selection
    
    def mouseMoveEvent(self, event):
        """Handle mouse move events for hover effects."""
        if self.image_rect is not None:
            # Convert from widget coordinates to image coordinates
            img_pos = self._map_to_image(event.pos().x(), event.pos().y())
            if img_pos:
                self.hover_point = img_pos
                self.update()  # Trigger repaint to show hover effect
            else:
                if self.hover_point is not None:
                    self.hover_point = None
                    self.update()
    
    def _map_to_image(self, widget_x, widget_y):
        """Map widget coordinates to image coordinates."""
        if self.image_rect is None or self.image_width == 0 or self.image_height == 0:
            return None
            
        # Check if click is within the image bounds
        if (widget_x < self.image_rect.left() or widget_x >= self.image_rect.right() or
            widget_y < self.image_rect.top() or widget_y >= self.image_rect.bottom()):
            return None
            
        # Calculate normalized coordinates (0-1) within the displayed image
        norm_x = (widget_x - self.image_rect.left()) / self.image_rect.width()
        norm_y = (widget_y - self.image_rect.top()) / self.image_rect.height()
        
        # Map to actual image coordinates
        img_x = int(norm_x * self.image_width)
        img_y = int(norm_y * self.image_height)
        
        # Ensure coordinates are within bounds - this is a critical safeguard
        img_x = max(0, min(img_x, self.image_width - 1))
        img_y = max(0, min(img_y, self.image_height - 1))
        
        return (img_x, img_y)
    
    def _map_to_widget(self, img_x, img_y):
        """Map image coordinates to widget coordinates."""
        if self.image_rect is None or self.image_width == 0 or self.image_height == 0:
            return None
            
        # Map from image coordinates to normalized coordinates (0-1)
        norm_x = img_x / self.image_width
        norm_y = img_y / self.image_height
        
        # Map to widget coordinates
        widget_x = self.image_rect.left() + norm_x * self.image_rect.width()
        widget_y = self.image_rect.top() + norm_y * self.image_rect.height()
        
        return (int(widget_x), int(widget_y))
    
    def paintEvent(self, event):
        """Paint the overlay with selection markers and calibration points."""
        super(ThermalImageOverlay, self).paintEvent(event)
        
        if self.image_rect is None:
            return
            
        painter = QPainter(self)
        
        # Enable anti-aliasing for smoother lines
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw hover crosshair
        if self.hover_point and self.hover_point != self.selected_point:
            img_x, img_y = self.hover_point
            widget_pos = self._map_to_widget(img_x, img_y)
            if widget_pos:
                widget_x, widget_y = widget_pos
                
                # Draw subtle crosshair
                painter.setPen(QPen(QColor(200, 200, 200, 150), 1))
                radius = 8
                painter.drawLine(widget_x - radius, widget_y, widget_x + radius, widget_y)
                painter.drawLine(widget_x, widget_y - radius, widget_x, widget_y + radius)
        
        # Draw current selection (yellow crosshair)
        if self.selected_point:
            img_x, img_y = self.selected_point
            widget_pos = self._map_to_widget(img_x, img_y)
            if widget_pos:
                widget_x, widget_y = widget_pos
                
                # Draw crosshair
                painter.setPen(QPen(QColor(255, 255, 0), 2))
                radius = 10
                painter.drawLine(widget_x - radius, widget_y, widget_x + radius, widget_y)
                painter.drawLine(widget_x, widget_y - radius, widget_x, widget_y + radius)
                
                # Draw circle
                painter.setPen(QPen(QColor(255, 255, 0), 1))
                painter.drawEllipse(widget_x - radius, widget_y - radius, radius * 2, radius * 2)
        
        # Draw saved calibration points (green squares with text)
        for idx, (x, y, temp, raw_value) in enumerate(self.calibration_points):
            widget_pos = self._map_to_widget(x, y)
            if widget_pos:
                widget_x, widget_y = widget_pos
                
                # Draw square
                painter.setPen(QPen(QColor(0, 255, 0), 2))
                size = 8
                painter.drawRect(widget_x - size, widget_y - size, size * 2, size * 2)
                
                # Draw point ID and temperature
                # Create a small background for the text to make it more readable
                point_id = idx + 1
                text = f"P{point_id}: {temp}°C"
                font = painter.font()
                font_metrics = QFontMetrics(font)
                text_width = font_metrics.width(text)
                text_height = font_metrics.height()
                
                # Draw text background
                painter.fillRect(
                    widget_x + size + 2, 
                    widget_y - text_height//2, 
                    text_width + 4, 
                    text_height, 
                    QColor(0, 0, 0, 180)
                )
                
                # Draw text
                painter.setPen(QPen(QColor(255, 255, 255), 1))
                painter.drawText(
                    widget_x + size + 4, 
                    widget_y + text_height//2 - 2, 
                    text
                )
        
        painter.end()

class ThermalImageView(QLabel):
    """Custom widget for displaying the thermal image with overlay for interactive features."""
    
    def __init__(self, parent=None):
        super(ThermalImageView, self).__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        
        # Increase the minimum size to ensure we have enough space
        self.setMinimumSize(800, 600)
        
        # Set better size policy to maximize use of available space
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # Set a frame around the image for better visibility
        self.setFrameShape(QLabel.Box)
        self.setFrameShadow(QLabel.Sunken)
        self.setLineWidth(1)
        
        # Create overlay widget for handling interactions
        self.overlay = ThermalImageOverlay(self)
        
        # Ensure the overlay fills the entire widget
        self.overlay.setGeometry(0, 0, self.width(), self.height())
    
    def add_calibration_point(self, x, y, temp, raw_value):
        """Add a point to the calibration points list."""
        self.overlay.add_calibration_point(x, y, temp, raw_value)
        
    def remove_calibration_point(self, index):
        """Remove a point from the calibration points list."""
        return self.overlay.remove_calibration_point(index)
        
    def clear_calibration_points(self):
        """Clear all calibration points."""
        self.overlay.clear_calibration_points()
    
    def setPixmap(self, pixmap):
        """Override setPixmap to update overlay dimensions."""
        super(ThermalImageView, self).setPixmap(pixmap)
        
        if not pixmap.isNull():
            # Update overlay dimensions
            self.overlay.update_image_dimensions(
                pixmap.width(), 
                pixmap.height(),
                self._get_image_rect()
            )
    
    def _get_image_rect(self):
        """Get the rectangle where the image is displayed within the label."""
        if not self.pixmap() or self.pixmap().isNull():
            return None
            
        # Calculate scaled image size
        pixmap_size = self.pixmap().size()
        scaled_size = pixmap_size.scaled(self.size(), Qt.KeepAspectRatio)
        
        # Calculate image position (centered in label)
        x = (self.width() - scaled_size.width()) / 2
        y = (self.height() - scaled_size.height()) / 2
        
        return QRect(int(x), int(y), scaled_size.width(), scaled_size.height())
    
    def resizeEvent(self, event):
        """Handle resize events to update overlay geometry."""
        super(ThermalImageView, self).resizeEvent(event)
        
        # Resize overlay to match widget size
        self.overlay.setGeometry(0, 0, self.width(), self.height())
        
        # Update overlay dimensions if we have a pixmap
        if self.pixmap() and not self.pixmap().isNull():
            self.overlay.update_image_dimensions(
                self.pixmap().width(),
                self.pixmap().height(),
                self._get_image_rect()
            )


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
        self.current_raw_value = None  # Store current raw value separately from frame
        self.calibration_points = []
        self.calibration_model = None
        self.selected_coords = None
        self.radiometric_mode = False
        self.last_raw_values = {}  # Dictionary to store raw values by coordinates
        self.image_mutex = QMutex()  # For thread safety
        self.raw_value_mutex = QMutex()  # Additional mutex for raw values
        
        # Add these new tracking variables
        self.has_valid_current_image = False
        self.last_16bit_timestamp = 0
        self.last_8bit_timestamp = 0
        
        # Initialize mutex for service call tracking
        self.service_mutex = QMutex()
        
        # Dictionary to track pending service calls
        self.pending_service_calls = {}  # Format: {"service_name:identifier": future}
        
        # [Rest of the __init__ method remains unchanged]
        
        # Define callback groups for threading safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Set up signal helper and connect signals
        self.signal_helper = SignalHelper()
        self.signal_helper.image_update_signal.connect(self._update_image_display_from_signal)
        self.signal_helper.raw_value_update_signal.connect(self._update_raw_value_label)
        self.signal_helper.temp_update_signal.connect(self._update_temp_label)
        self.signal_helper.points_table_update_signal.connect(self._update_points_table)
        self.signal_helper.cal_results_update_signal.connect(self._update_calibration_results)

        # after connecting the other signals:
        self.signal_helper.timer_cleanup_signal.connect(self._cleanup_timer_main_thread)
        self.signal_helper.calibration_complete_signal.connect(self._handle_calibration_complete) 

        # Initialize UI
        self._init_ui()
        
        # Initialize ROS communication
        self._setup_ros_communication()
        
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # Create timer for regular UI updates
        self.update_timer = QTimer(self._widget)
        self.update_timer.timeout.connect(self._update_ui)
        self.update_timer.start(100)  # Update every 100ms
        
        # Create timer for service availability checking
        self.service_check_timer = QTimer(self._widget)
        self.service_check_timer.timeout.connect(self._try_reconnect_services)
        self.service_check_timer.start(5000)  # Check services every 5 seconds
        
        # Create timer for reducing update frequency when not in focus
        self.focus_check_timer = QTimer(self._widget)
        self.focus_check_timer.timeout.connect(self._check_focus)
        self.focus_check_timer.start(1000)  # Check focus every 1 second
        
        # Initialize focus status
        self.is_in_focus = False
        
        # Set up pixmap for empty image
        self.empty_pixmap = QPixmap(640, 480)
        self.empty_pixmap.fill(Qt.black)
        self.image_view.setPixmap(self.empty_pixmap)
        
        # Log initialization
        self._node.get_logger().info("Thermal calibration plugin initialized")
        
    def _init_ui(self):
        """Initialize the user interface."""
        # Create main layout
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(5, 5, 5, 5)  # Reduce margins for more space
        self._widget.setLayout(main_layout)
        
        # Create splitter for left and right panel
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # Left panel (thermal image)
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)  # Reduce margins for more space

        # Thermal image view (now with overlay)
        self.image_view = ThermalImageView()
        left_layout.addWidget(self.image_view, 1)  # Give it a stretch factor of 1
        
        # Connect overlay's pixel_clicked signal instead of the direct image view
        self.image_view.overlay.pixel_clicked.connect(self._on_pixel_clicked)
        
        # Add information labels under the image
        img_info_layout = QHBoxLayout()
        self.coords_label = QLabel("Coordinates: -")
        img_info_layout.addWidget(self.coords_label)
        
        self.raw_value_label = QLabel("Raw value: -")
        img_info_layout.addWidget(self.raw_value_label)
        
        self.temp_label = QLabel("Temperature: (calibration pending)")
        img_info_layout.addWidget(self.temp_label)
        
        left_layout.addLayout(img_info_layout)
        
        # Right panel (controls)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(5, 5, 5, 5)  # Reduce margins for more space
        
        # Top section - Controls for entering reference temperatures
        controls_group = QWidget()
        controls_layout = QVBoxLayout(controls_group)
        controls_layout.setContentsMargins(5, 5, 5, 5)  # Reduce margins

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
        self.enter_temp_btn = QPushButton("Add Temperature Point")
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
        self.temp_input.editingFinished.connect(self._on_temp_input_editing_finished)

        self.save_temp_btn = QPushButton("Save Point")
        self.save_temp_btn.clicked.connect(self._on_save_temp_clicked)
        self.temp_input_layout.addWidget(self.save_temp_btn)
        
        # Simple text cancel button
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
        calibration_layout.setContentsMargins(5, 5, 5, 5)  # Reduce margins
        
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
        
        # Add button to remove last calibration point
        self.remove_last_btn = QPushButton("Remove Last Point")
        self.remove_last_btn.setEnabled(False)
        self.remove_last_btn.clicked.connect(self._on_remove_last_clicked)
        cal_controls_layout.addWidget(self.remove_last_btn)
        
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

        # Add service status indicator
        self.service_status_layout = QHBoxLayout()
        self.service_status_indicator = QLabel()
        self.service_status_indicator.setFixedSize(16, 16)
        self.service_status_indicator.setStyleSheet("background-color: gray; border-radius: 8px;")
        self.service_status_layout.addWidget(self.service_status_indicator)
        
        self.service_status_label = QLabel("Checking services...")
        self.service_status_layout.addWidget(self.service_status_label)
        self.service_status_layout.addStretch(1)
        
        # Add the status layout to the bottom of the right panel
        right_layout.addLayout(self.service_status_layout)

        # Add panels to splitter
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)

        # Set initial splitter sizes (80% left, 20% right) to give more space to the image
        splitter.setSizes([800, 200])
        
    def _setup_ros_communication(self):
        """Set up ROS subscribers and service clients."""
        # Create QoS profile for lower rate image subscription
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Change to BEST_EFFORT for image streams
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Only keep the latest message
        )
        
        # Image subscribers for both 16-bit and 8-bit thermal streams with lower rate
        self.image_16bit_sub = self._node.create_subscription(
            Image,
            'image_raw',  # 16-bit thermal image used for calibration
            self._image_16bit_callback,
            image_qos  # Use our custom QoS profile
        )
        
        self.image_8bit_sub = self._node.create_subscription(
            Image,
            'image_raw/mono8',  # 8-bit visualization stream
            self._image_8bit_callback,
            image_qos  # Use our custom QoS profile
        )
        
        # Initialize frame buffer variables
        self.last_frame_timestamp = 0
        self.frame_buffer_interval = 0.2  # Process frames at ~5 fps (200ms interval)
        self.buffered_16bit_image = None
        self.buffered_8bit_image = None
        
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
        
        # Set up periodic service check
        self.service_check_timer = QTimer(self._widget)
        self.service_check_timer.timeout.connect(self._try_reconnect_services)
        self.service_check_timer.start(5000)  # Check services every 5 seconds

    @Slot(object)
    def _update_image_display_from_signal(self, img_data):
        """Update image display from the main thread."""
        if not self._widget.isVisible():
            return
            
        try:
            with QMutexLocker(self.image_mutex):
                if isinstance(img_data, np.ndarray):
                    if len(img_data.shape) == 2:  # If it's a grayscale image
                        self._update_image_display(img_data)
                    elif len(img_data.shape) == 3:  # If it's already a colored image
                        # Just update directly
                        h, w, c = img_data.shape
                        q_img = QImage(img_data.data, w, h, w * c, QImage.Format_RGB888).rgbSwapped()
                        pixmap = QPixmap.fromImage(q_img)
                        
                        # Scale pixmap to fit the label while maintaining aspect ratio
                        # and update the overlay
                        self._set_scaled_pixmap(pixmap)
        except Exception as e:
            self._node.get_logger().error(f'Error updating image from signal: {e}')

    def _set_scaled_pixmap(self, pixmap):
        """Set a scaled pixmap that better fills the available space and update overlay."""
        if pixmap.isNull():
            return
            
        # Get the available size in the image view
        view_size = self.image_view.size()
        
        # Scale up the image to better fill the space
        # We'll scale it to 90% of the view size to leave a small margin
        scaled_pixmap = pixmap.scaled(
            int(view_size.width() * 0.9), 
            int(view_size.height() * 0.9),
            Qt.KeepAspectRatio, 
            Qt.SmoothTransformation
        )
        
        # Set the scaled pixmap - this will also update the overlay's dimensions
        # through the overridden setPixmap method in ThermalImageView
        self.image_view.setPixmap(scaled_pixmap)
        
        # Get the image rectangle for additional verification (if needed)
        image_rect = self.image_view._get_image_rect()
        if image_rect:
            # Ensure the overlay has the correct dimensions in case setPixmap didn't do it
            self.image_view.overlay.update_image_dimensions(
                pixmap.width(),
                pixmap.height(),
                image_rect
            )

    @Slot(str)
    def _update_raw_value_label(self, text):
        """Update raw value label from signal."""
        self.raw_value_label.setText(text)
    
    @Slot(str)
    def _update_temp_label(self, text):
        """Update temperature label from signal."""
        self.temp_label.setText(text)

    def _check_focus(self):
        """Check if the widget is in focus and adjust update frequency."""
        is_visible = self._widget.isVisible() and not self._widget.isMinimized()
        
        if is_visible != self.is_in_focus:
            self.is_in_focus = is_visible
            
            # Adjust image subscription QoS based on focus
            if is_visible:
                # If visible, use normal update frequency
                self._node.get_logger().debug("Plugin is visible, using normal update frequency")
            else:
                # If not visible, reduce update frequency to save resources
                self._node.get_logger().debug("Plugin is not visible, reducing update frequency")
        
    def _image_16bit_callback(self, msg):
        """Callback for 16-bit thermal image with rate limiting."""
        current_time = time.time()
        
        # Store this frame in the buffer regardless of timing
        try:
            with QMutexLocker(self.image_mutex):
                self.buffered_16bit_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
                # Make this the current image as well
                self.current_image = self.buffered_16bit_image.copy()
                self.has_valid_current_image = True
                self.last_16bit_timestamp = current_time
        except Exception as e:
            self._node.get_logger().error(f'Error buffering 16-bit image: {e}')
            return
            
        # Only process this frame for display if enough time has passed since the last one
        # This effectively reduces the processing rate regardless of incoming frame rate
        if current_time - self.last_frame_timestamp < self.frame_buffer_interval:
            return  # Skip processing this frame to maintain our lower rate
            
        # Update timestamp since we're processing this frame
        self.last_frame_timestamp = current_time
        
        # Process image for display if we don't have an 8-bit stream
        if not hasattr(self, 'has_8bit_stream') or not self.has_8bit_stream:
            try:
                with QMutexLocker(self.image_mutex):
                    display_img = cv2.normalize(self.current_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    colored_img = cv2.applyColorMap(display_img, cv2.COLORMAP_INFERNO)
                self.signal_helper.image_update_signal.emit(colored_img)
            except Exception as e:
                self._node.get_logger().error(f'Error processing image for display: {e}')
        
        # Process selected coordinates if any
        try:
            selected_coords = None
            current_raw_value = None
            with QMutexLocker(self.raw_value_mutex):
                if hasattr(self, 'selected_coords') and self.selected_coords:
                    selected_coords = self.selected_coords
                    current_raw_value = self.current_raw_value
                    
            # If we have selected coordinates but no raw value yet, try to get it from the current frame
            if selected_coords is not None and current_raw_value is None:
                x, y = selected_coords
                coord_key = f"{x},{y}"
                
                # Thread-safe check if we've processed this coordinate
                coord_processed = False
                with QMutexLocker(self.raw_value_mutex):
                    coord_processed = coord_key in self.last_raw_values
                    
                # Only update if we haven't processed this coordinate yet
                if not coord_processed:
                    with QMutexLocker(self.image_mutex):
                        if self.current_image is not None:
                            if 0 <= y < self.current_image.shape[0] and 0 <= x < self.current_image.shape[1]:
                                # Get raw value from current frame and store it
                                current_raw = int(self.current_image[y, x])
                                
                                with QMutexLocker(self.raw_value_mutex):
                                    self.last_raw_values[coord_key] = current_raw
                                    
                                    # Only update if we don't have a current raw value yet
                                    if self.current_raw_value is None:
                                        self.current_raw_value = current_raw
                                        # Use signal to update UI from main thread
                                        self.signal_helper.raw_value_update_signal.emit(f"Raw value: {current_raw}")
                                        
                                        # Also call the service to get a more reliable value
                                        self._call_get_raw_value(x, y)
        except Exception as e:
            self._node.get_logger().error(f'Error processing selected coordinates: {e}')

    def _image_8bit_callback(self, msg):
        """Callback for 8-bit thermal image with rate limiting."""
        current_time = time.time()
        
        # Set flag that we have 8-bit stream
        self.has_8bit_stream = True
        
        # Store this frame in the buffer regardless of timing
        try:
            # Convert ROS image message to OpenCV image and store in buffer
            self.buffered_8bit_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            # Store the timestamp
            self.last_8bit_timestamp = current_time
        except Exception as e:
            self._node.get_logger().error(f'Error buffering 8-bit image: {e}')
            return
            
        # Only process this frame for display if enough time has passed since the last one
        if current_time - self.last_frame_timestamp < self.frame_buffer_interval:
            return  # Skip processing this frame to maintain our lower rate
            
        # Update timestamp since we're processing this frame
        self.last_frame_timestamp = current_time
        
        try:
            # Store a copy for colormap changes
            img_8bit = self.buffered_8bit_image.copy()
            self.last_8bit_image = img_8bit
            
            # Apply colormap
            if not hasattr(self, 'current_colormap'):
                self.current_colormap = "Grayscale"
                
            if self.current_colormap == "Grayscale":
                # For grayscale, don't apply a colormap, just convert to RGB
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
            
            # Send the colored image via signal to the main thread for UI update
            self.signal_helper.image_update_signal.emit(colored_img)
        
        except Exception as e:
            self._node.get_logger().error(f'Error processing 8-bit image: {e}')

    def _get_current_raw_value_from_buffer(self, x, y):
        """Get the raw value at coordinates from the latest buffered image."""
        raw_value = None
        
        with QMutexLocker(self.image_mutex):
            # First try the current image
            if self.current_image is not None:
                if 0 <= y < self.current_image.shape[0] and 0 <= x < self.current_image.shape[1]:
                    raw_value = int(self.current_image[y, x])
                    self._node.get_logger().info(f'Got raw value from current image: {raw_value}')
                    return raw_value
                    
            # If that fails, try the buffer
            if self.buffered_16bit_image is not None:
                if 0 <= y < self.buffered_16bit_image.shape[0] and 0 <= x < self.buffered_16bit_image.shape[1]:
                    raw_value = int(self.buffered_16bit_image[y, x])
                    self._node.get_logger().info(f'Got raw value from buffered image: {raw_value}')
                    return raw_value
        
        # If we get here, we couldn't get a raw value
        self._node.get_logger().warn(f'Could not get raw value for coordinates ({x}, {y})')
        return None

    def _update_image_display(self, colored_img):
        """Update the image display with the given colored image."""
        try:
            # Only update if widget is visible
            if not self._widget.isVisible():
                return
                
            # Convert OpenCV image to QImage then QPixmap for display
            h, w, c = colored_img.shape
            q_img = QImage(colored_img.data, w, h, w * c, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img)
            
            # Update image view with scaled pixmap
            self._set_scaled_pixmap(pixmap)
        
        except Exception as e:
            self._node.get_logger().error(f'Error updating image display: {e}')
            self._node.get_logger().error(traceback.format_exc())
    
    def _update_display_from_16bit(self):
        """Update the display using the 16-bit image when 8-bit stream is not available."""
        if self.current_image is None:
            return
            
        try:
            # Convert to display image (apply colormap)
            display_img = cv2.normalize(self.current_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Apply current colormap
            if not hasattr(self, 'current_colormap'):
                self.current_colormap = "Grayscale"
                
            if self.current_colormap == "Grayscale":
                # For grayscale, we don't apply a colormap, but convert to RGB
                colored_img = cv2.cvtColor(display_img, cv2.COLOR_GRAY2BGR)
            else:
                # Map colormap name to OpenCV constant
                colormap_map = {
                    "Inferno": cv2.COLORMAP_INFERNO,
                    "Jet": cv2.COLORMAP_JET,
                    "Viridis": cv2.COLORMAP_VIRIDIS,
                    "Rainbow": cv2.COLORMAP_RAINBOW
                }
                colormap = colormap_map.get(self.current_colormap, cv2.COLORMAP_INFERNO)
                colored_img = cv2.applyColorMap(display_img, colormap)
            
            # Send via signal to the main thread
            self.signal_helper.image_update_signal.emit(colored_img)
        
        except Exception as e:
            self._node.get_logger().error(f'Error updating display from 16-bit: {e}')
    
    def _on_pixel_clicked(self, x, y):
        """Handle pixel selection in the image."""
        self._node.get_logger().info(f'Pixel clicked at ({x}, {y})')
        
        # Only process the click if we're not currently in temperature input mode
        if not self.temp_input_widget.isVisible():
            # Store coordinates persistently, regardless of frame updates
            self.selected_coords = (x, y)
            self.coords_label.setText(f"Coordinates: ({x}, {y})")
            
            # Enable the button to enter temperature
            self.enter_temp_btn.setEnabled(True)
            
            # Get the raw value from the current image (directly and via service)
            raw_value = self._get_current_raw_value_from_buffer(x, y)
            if raw_value is not None:
                with QMutexLocker(self.raw_value_mutex):
                    self.current_raw_value = raw_value
                    # Cache the value by coordinates
                    coord_key = f"{x},{y}"
                    self.last_raw_values[coord_key] = raw_value
                
                # Update the UI
                self.signal_helper.raw_value_update_signal.emit(f"Raw value: {raw_value}")
                
                # Update temperature if in radiometric mode
                if self.radiometric_mode and self.calibration_model:
                    self._update_temperature_display(raw_value)
            
            # Also call the service to get raw value for verification
            self._call_get_raw_value(x, y)
    
    def _update_temperature_display(self, raw_value):
        """Update the temperature display for a given raw value using the current calibration model."""
        if not self.radiometric_mode or not self.calibration_model or raw_value is None:
            return
        
        # Log that we're trying to update the temperature
        self._node.get_logger().info(f'Updating temperature display for raw value: {raw_value}')
        
        # First, attempt to calculate the temperature directly if we have the model
        try:
            model_params = self.calibration_model['parameters']
            if model_params and len(model_params) > 0:
                # Use numpy's poly1d to evaluate the polynomial
                import numpy as np
                p = np.poly1d(model_params)
                temp = float(p(raw_value))
                
                # Update the UI
                self.signal_helper.temp_update_signal.emit(f"Temperature: {temp:.1f}°C")
                self._node.get_logger().info(f'Direct calculation: Raw value {raw_value} -> {temp:.1f}°C')
        except Exception as e:
            self._node.get_logger().error(f'Error calculating temperature directly: {e}')
        
        # Also call service for verification (and to ensure backend is in sync)
        self._call_raw_to_temperature(raw_value)
    
    def _on_enter_temp_clicked(self):
        """Handle click on enter temperature button."""
        if not self.temp_input_widget.isVisible():
            # Switching to temperature input mode
            self.temp_input_widget.setVisible(True)
            
            # Disable the enter temp button while input controls are visible
            self.enter_temp_btn.setEnabled(False)
            
            # Set focus to the temperature input
            self.temp_input.setFocus()
        else:
            # This branch shouldn't be reachable now that we disable the button,
            # but just in case someone finds a way to activate it:
            self.temp_input_widget.setVisible(False)
            self.enter_temp_btn.setEnabled(True)

    def _save_temperature_value(self):
        """Save the entered temperature value with the current pixel.
        This is a method of the ThermalCalibrationPlugin class."""
        try:
            with QMutexLocker(self.raw_value_mutex):
                selected_coords = self.selected_coords
                current_raw_value = self.current_raw_value
            
            self._node.get_logger().info(f'Selected coordinates: {selected_coords}')
            self._node.get_logger().info(f'Current raw value: {current_raw_value}')
            
            if selected_coords is None:
                QMessageBox.warning(self._widget, "No Selection", 
                                "Please select a point on the image first.")
                return
                
            # Get reference temperature from input
            reference_temp = self.temp_input.value()
            
            # Check for raw value - try all possible sources
            raw_value = None
            x, y = selected_coords
            coord_key = f"{x},{y}"
            
            # Thread-safe access to raw values
            with QMutexLocker(self.raw_value_mutex):
                # First try current_raw_value
                if current_raw_value is not None:
                    raw_value = current_raw_value
                # Then check cached values
                elif coord_key in self.last_raw_values:
                    raw_value = self.last_raw_values[coord_key]
                    self._node.get_logger().info(f'Using cached raw value for {coord_key}: {raw_value}')
                else:
                    self._node.get_logger().warn(f'No cached raw value for {coord_key}')
            
            # If still no value, try reading from the current image directly
            if raw_value is None:
                try:
                    with QMutexLocker(self.image_mutex):
                        if self.current_image is not None:
                            if 0 <= y < self.current_image.shape[0] and 0 <= x < self.current_image.shape[1]:
                                raw_value = int(self.current_image[y, x])
                                self._node.get_logger().info(f'Read raw value directly from image: {raw_value}')
                        else:
                            self._node.get_logger().warn('No current image available')
                except Exception as e:
                    self._node.get_logger().error(f'Error reading from image: {e}')
            
            # Use 0 as a last resort if we still don't have a raw value
            if raw_value is None:
                raw_value = 0  # Default to 0 if no value could be retrieved
                self._node.get_logger().warn(f'Using default raw value 0 as a fallback')
            
            # Add calibration point using stored coordinates and raw value
            try:
                # Store point locally for immediate feedback
                point_id = len(self.calibration_points) + 1
                new_point = {
                    'id': point_id,
                    'x': x,
                    'y': y,
                    'raw_value': raw_value,
                    'reference_temp': reference_temp,
                    'timestamp': datetime.now().isoformat()
                }
                
                # Add to local model
                self.calibration_points.append(new_point)
                
                # Update UI
                self.signal_helper.points_table_update_signal.emit()
                
                # Add point to image view
                try:
                    self.image_view.add_calibration_point(x, y, reference_temp, raw_value)
                except Exception as e:
                    self._node.get_logger().error(f'Error adding point to image view: {e}')
                
                # Call service to add the calibration point
                self._call_add_calibration_point(x, y, raw_value, reference_temp)
                
                self._node.get_logger().info(f'Added calibration point: {new_point}')
            
                # Hide temperature input controls and re-enable enter button
                self.temp_input_widget.setVisible(False)
                self.enter_temp_btn.setEnabled(True)  
                
                # Enable remove last button if we have points
                self.remove_last_btn.setEnabled(len(self.calibration_points) > 0)
                
            except Exception as e:
                self._node.get_logger().error(f'Error saving temperature: {e}')
                self._node.get_logger().error(traceback.format_exc())
                QMessageBox.critical(self._widget, "Error", 
                                    f"Failed to save temperature: {str(e)}")
                
                # Ensure temperature input is hidden and enter button is re-enabled
                # even if an error occurs
                self.temp_input_widget.setVisible(False)
                self.enter_temp_btn.setEnabled(True)
        
        except Exception as e:
            self._node.get_logger().error(f'Unexpected error in save_temperature_value: {e}')
            self._node.get_logger().error(traceback.format_exc())
            
            # Ensure UI is in a good state
            self.temp_input_widget.setVisible(False)
            self.enter_temp_btn.setEnabled(True)

    def _on_save_temp_clicked(self):
        """Handle click on save temperature button.
        This is a method of the ThermalCalibrationPlugin class."""
        # Set a flag to prevent double-processing
        if hasattr(self, '_save_in_progress') and self._save_in_progress:
            self._node.get_logger().info("Save already in progress, ignoring")
            return
            
        self._save_in_progress = True
        
        try:
            # Commit any partial edits
            self.temp_input.interpretText()
            self.temp_input.clearFocus()
            
            self._node.get_logger().info("Save button clicked - saving temperature value")
            self._save_temperature_value()
        finally:
            # Reset the flag
            self._save_in_progress = False
    
    def _on_cancel_temp_clicked(self):
        """Handle click on cancel temperature button."""
        # Hide temperature input and re-enable the enter temperature button
        self.temp_input_widget.setVisible(False)
        self.enter_temp_btn.setEnabled(True)

    def _on_remove_last_clicked(self):
        """Handle click on remove last point button."""
        if not self.calibration_points:
            return
        
        try:
            # Get the last point
            last_point = self.calibration_points[-1]
            
            # Ask for confirmation
            reply = QMessageBox.question(
                self._widget, "Remove Point", 
                f"Remove calibration point at ({last_point['x']}, {last_point['y']}) with temperature {last_point['reference_temp']}°C?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                # Store point values before removal for logging
                point_x = last_point['x']
                point_y = last_point['y']
                point_temp = last_point['reference_temp']
                
                # Remove from our local list
                self.calibration_points.pop()
                
                # Update the table UI immediately
                self.signal_helper.points_table_update_signal.emit()
                
                # Remove from image view
                try:
                    # Clear and rebuild the overlay's points
                    self.image_view.clear_calibration_points()
                    
                    # Rebuild all points
                    for point in self.calibration_points:
                        self.image_view.add_calibration_point(
                            point['x'], 
                            point['y'], 
                            point['reference_temp'], 
                            point['raw_value']
                        )
                        
                    # Force update of the overlay
                    self.image_view.overlay.update()
                except Exception as e:
                    self._node.get_logger().error(f'Error updating overlay: {e}')
                    self._node.get_logger().error(traceback.format_exc())
                    
                # Use the clear-and-rebuild approach to keep backend and UI in sync
                self._node.get_logger().info("Synchronizing backend with UI after point removal")
                success = self._clear_and_rebuild_backend_points()
                
                if not success:
                    self._node.get_logger().warn(
                        "Remove last point: The backend couldn't be synchronized. " +
                        "Point was removed from UI only.")
                        
                    # Show an unobtrusive notification to the user
                    self.service_status_label.setText("Backend out of sync - point removed in UI only")
                    self.service_status_indicator.setStyleSheet("background-color: orange; border-radius: 8px;")
                    
                # Disable remove button if no more points
                if not self.calibration_points:
                    self.remove_last_btn.setEnabled(False)
                    
                # Log successful removal
                self._node.get_logger().info(
                    f"Removed point ({point_x}, {point_y}) with temperature {point_temp}°C")
            
        except Exception as e:
            self._node.get_logger().error(f'Error removing last point: {e}')
            self._node.get_logger().error(traceback.format_exc())
    
    def _on_calibrate_clicked(self):
        """Handle click on calibrate button."""
        if len(self.calibration_points) < 2:
            QMessageBox.warning(self._widget, "Insufficient Data", 
                            "At least 2 calibration points are required.")
            return
            
        # Get model type and degree
        model_type = self.model_type_combo.currentText().lower()
        degree = int(self.degree_spin.value())
        
        # Basic validation
        if degree < 1 or degree > 5:
            QMessageBox.warning(self._widget, "Invalid Degree", 
                            "Polynomial degree must be between 1 and 5.")
            return
        
        # Update UI to show we're calibrating
        self.calibrate_btn.setEnabled(False)
        self.calibrate_btn.setText("Calibrating...")
        
        # Perform calibration (without creating a progress dialog)
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
            # If we have a selected point and a model, update the temperature
            if hasattr(self, 'selected_coords') and self.selected_coords and self.calibration_model:
                with QMutexLocker(self.raw_value_mutex):
                    current_raw = self.current_raw_value
                if current_raw is not None:
                    self._update_temperature_display(current_raw)
        else:
            self.radio_toggle.setText("Enable Radiometric Mode")
            # Update temperature label to show it's disabled
            if self.calibration_model:
                self.signal_helper.temp_update_signal.emit("Temperature: (enable radiometric mode)")
            else:
                self.signal_helper.temp_update_signal.emit("Temperature: (calibration pending)")
    
    def _update_ui(self):
        """Periodically update UI elements."""
        # This method can be used for any UI updates that need to happen regularly
        pass
    
    @Slot()
    def _update_points_table(self):
        """Update the calibration points table."""
        try:
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
        except Exception as e:
            self._node.get_logger().error(f'Error updating points table: {e}')
    
    @Slot()
    def _update_calibration_results(self):
        """Update the calibration results display."""
        try:
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
                
                # Update temperature label format now that calibration is available
                if not self.radiometric_mode:
                    self.signal_helper.temp_update_signal.emit("Temperature: (enable radiometric mode)")
            else:
                self.cal_results_label.setText("No calibration performed yet")
                self.export_btn.setEnabled(False)
                self.radio_toggle.setEnabled(False)
                self.signal_helper.temp_update_signal.emit("Temperature: (calibration pending)")
        except Exception as e:
            self._node.get_logger().error(f'Error updating calibration results: {e}')

    def _on_colormap_changed(self, colormap_name):
        """Handle change in colormap selection."""
        self.current_colormap = colormap_name
        # Refresh the display if we have an image
        if hasattr(self, 'has_8bit_stream') and self.has_8bit_stream and hasattr(self, 'last_8bit_image'):
            try:
                # Apply colormap
                if colormap_name == "Grayscale":
                    # For grayscale, we don't apply a colormap, but convert to RGB
                    colored_img = cv2.cvtColor(self.last_8bit_image, cv2.COLOR_GRAY2BGR)
                else:
                    # Map colormap name to OpenCV constant
                    colormap_map = {
                        "Inferno": cv2.COLORMAP_INFERNO,
                        "Jet": cv2.COLORMAP_JET,
                        "Viridis": cv2.COLORMAP_VIRIDIS,
                        "Rainbow": cv2.COLORMAP_RAINBOW
                    }
                    colormap = colormap_map.get(colormap_name, cv2.COLORMAP_INFERNO)
                    colored_img = cv2.applyColorMap(self.last_8bit_image, colormap)
                
                # Send via signal to main thread
                self.signal_helper.image_update_signal.emit(colored_img)
            except Exception as e:
                self._node.get_logger().error(f'Error applying colormap: {e}')
        elif self.current_image is not None:
            self._update_display_from_16bit()

    # Service call methods

    def _track_service_call(self, service_name, identifier, future):
        """Track a service call to prevent race conditions and handle timeouts."""
        key = f"{service_name}:{identifier}"
        
        with QMutexLocker(self.service_mutex):
            self.pending_service_calls[key] = future
        
        def cleanup_callback(future_result):
            # Use signal to perform timer cleanup in main thread
            timer_key = key + "_timer"
            self.signal_helper.timer_cleanup_signal.emit(timer_key)
            
            # Remove service call from tracking
            with QMutexLocker(self.service_mutex):
                if key in self.pending_service_calls:
                    del self.pending_service_calls[key]
                    self._node.get_logger().debug(f'Service call {key} completed and removed from tracking')
        
        # Add cleanup callback
        future.add_done_callback(cleanup_callback)
        
        return key

    def _call_get_raw_value(self, x, y):
        """Call the get_raw_value service with tracking."""
        if not self.get_raw_value_client.service_is_ready():
            # Log warning
            self._node.get_logger().warn('get_raw_value service not available')
            
            # Try to use cached value if available
            coord_key = f"{x},{y}"
            if coord_key in self.last_raw_values:
                self.current_raw_value = self.last_raw_values[coord_key]
                self.signal_helper.raw_value_update_signal.emit(f"Raw value: {self.current_raw_value}")
                return
                
            # If no cached value and current image is available, use it temporarily
            if self.current_image is not None:
                try:
                    if 0 <= y < self.current_image.shape[0] and 0 <= x < self.current_image.shape[1]:
                        raw_value = int(self.current_image[y, x])
                        self.current_raw_value = raw_value
                        self.last_raw_values[coord_key] = raw_value
                        self.signal_helper.raw_value_update_signal.emit(f"Raw value: {raw_value}")
                        return
                except Exception as e:
                    self._node.get_logger().error(f'Error reading raw value from image: {e}')
                    
            # If all else fails, show a message that we couldn't get the raw value
            self.signal_helper.raw_value_update_signal.emit("Raw value: Service unavailable")
            return
        
        # Check if a call for these coordinates is already pending
        call_key = f"get_raw_value:{x},{y}"
        
        with QMutexLocker(self.service_mutex):
            if call_key in self.pending_service_calls:
                # Already pending, don't call again
                self._node.get_logger().debug(f'Service call for get_raw_value({x},{y}) already pending')
                return
        
        # Proceed with service call
        request = GetRawValue.Request()
        request.x = x
        request.y = y
        
        try:
            future = self.get_raw_value_client.call_async(request)
            # Track this call
            self._track_service_call("get_raw_value", f"{x},{y}", future)
            future.add_done_callback(self._get_raw_value_done)
        except Exception as e:
            self._node.get_logger().error(f'Error calling get_raw_value service: {e}')
            # Use direct image access as fallback
            coord_key = f"{x},{y}"
            if self.current_image is not None:
                try:
                    if 0 <= y < self.current_image.shape[0] and 0 <= x < self.current_image.shape[1]:
                        raw_value = int(self.current_image[y, x])
                        self.current_raw_value = raw_value
                        self.last_raw_values[coord_key] = raw_value
                        self.signal_helper.raw_value_update_signal.emit(f"Raw value: {raw_value}")
                except Exception as e2:
                    self._node.get_logger().error(f'Error reading raw value from image: {e2}')
    
    def _get_raw_value_done(self, future):
        """Callback for get_raw_value service response."""
        try:
            response = future.result()
            if response.success:
                # Thread-safe access to selected coordinates
                current_selected_coords = None
                with QMutexLocker(self.raw_value_mutex):
                    if hasattr(self, 'selected_coords'):
                        current_selected_coords = self.selected_coords
                
                # Only update if we have valid coordinates
                if current_selected_coords:
                    x, y = current_selected_coords
                    coord_key = f"{x},{y}"
                    
                    # Thread-safe update of raw values
                    with QMutexLocker(self.raw_value_mutex):
                        # Add to our cached values
                        self.last_raw_values[coord_key] = response.raw_value
                        
                        # Only update current_raw_value if it's for the currently selected point
                        if self.selected_coords == (x, y):
                            self.current_raw_value = response.raw_value
                            # Update UI
                            self.signal_helper.raw_value_update_signal.emit(f"Raw value: {response.raw_value}")
                            
                            # Update temperature if in radiometric mode
                            if self.radiometric_mode and self.calibration_model:
                                self._update_temperature_display(response.raw_value)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
    
    def _call_add_calibration_point(self, x, y, raw_value, reference_temp):
        """Call the add_calibration_point service with tracking."""
        if not self.add_calibration_point_client.service_is_ready():
            self._node.get_logger().warn('add_calibration_point service not available')
            self._update_service_status_indicators(False)
            return
        
        # Create a unique ID for this calibration point
        point_id = f"{x},{y},{reference_temp}"
        call_key = f"add_calibration_point:{point_id}"
        
        with QMutexLocker(self.service_mutex):
            if call_key in self.pending_service_calls:
                # Already pending, don't call again
                self._node.get_logger().debug(f'Service call for add_calibration_point({point_id}) already pending')
                return
        
        # Make sure the values are of the correct type
        x_int = int(x)
        y_int = int(y)
        
        # Ensure raw_value is a 16-bit integer (0-65535)
        try:
            raw_value_int = int(raw_value) if raw_value is not None else 0
            # Check if it's within 16-bit range
            if raw_value_int < 0 or raw_value_int > 65535:
                self._node.get_logger().warn(f'Raw value {raw_value_int} is outside 16-bit range (0-65535), clamping')
                raw_value_int = max(0, min(65535, raw_value_int))
        except (TypeError, ValueError) as e:
            self._node.get_logger().error(f'Error converting raw value to integer: {e}')
            raw_value_int = 0  # Default to 0 if conversion fails
        
        # Ensure reference_temp is a float with 1 decimal precision
        try:
            # Round to 1 decimal place
            reference_temp_float = round(float(reference_temp), 1)
        except (TypeError, ValueError) as e:
            self._node.get_logger().error(f'Error converting reference temperature to float: {e}')
            reference_temp_float = 0.0  # Default to 0.0 if conversion fails
        
        # Log the values for debugging
        self._node.get_logger().info(f'Adding calibration point: x={x_int}, y={y_int}, raw_value={raw_value_int}, temp={reference_temp_float}')
        
        # Proceed with service call
        request = AddCalibrationPoint.Request()
        request.x = x_int  # Pixel x-coordinate (integer)
        request.y = y_int  # Pixel y-coordinate (integer)
        request.raw_value = raw_value_int  # 16-bit integer (0-65535)
        request.reference_temp = reference_temp_float  # Float with 1 decimal precision
        
        try:
            future = self.add_calibration_point_client.call_async(request)
            # Track this call
            self._track_service_call("add_calibration_point", point_id, future)
            future.add_done_callback(self._add_calibration_point_done)
            
            # Set up a timeout timer
            point_timeout = QTimer(self._widget)
            point_timeout.setSingleShot(True)
            point_timeout.timeout.connect(lambda: self._handle_service_timeout(call_key, "add_point"))
            point_timeout.start(5000)  # 5 second timeout
            
            # Store the timer
            with QMutexLocker(self.service_mutex):
                self.pending_service_calls[call_key + "_timer"] = point_timeout
                
            # Update service status to show we're connecting
            self._update_service_status_indicators(True) 
                
        except Exception as e:
            self._node.get_logger().error(f'Error calling add_calibration_point service: {e}')
            self._update_service_status_indicators(False)
    
    def _add_calibration_point_done(self, future):
        """Callback for add_calibration_point service response."""
        try:
            response = future.result()
            
            # Update service status to indicate success
            self._update_service_status_indicators(True)
            
            if response.success:
                # Log the backend's response but don't modify UI - it's already been updated
                self._node.get_logger().info(f"Backend added point with ID: {response.point_id} - {response.message}")
            else:
                # If backend rejected the point, we should log it but not show a dialog
                # since we've already added it to the UI
                self._node.get_logger().warn(f"Backend rejected point: {response.message}")
                
                # If it's a duplicate point warning from the backend, let's log it but not bother the user
                if "duplicate" in response.message.lower():
                    self._node.get_logger().info("Backend detected duplicate point - this is expected behavior")
                    
                # Note: We intentionally don't remove the point from the UI even if backend rejected it
                # This keeps the UI responsive and consistent with what the user expects
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            self._update_service_status_indicators(False)

    def _update_service_status_indicators(self, connected=None):
        """Update UI indicators for service status."""
        if not hasattr(self, 'service_status_indicator'):
            return  # UI not initialized yet
            
        # If no specific state is provided, check all services
        if connected is None:
            services_ready = (
                self.get_raw_value_client.service_is_ready() and
                self.add_calibration_point_client.service_is_ready() and
                self.perform_calibration_client.service_is_ready() and
                self.clear_calibration_data_client.service_is_ready() and
                self.raw_to_temperature_client.service_is_ready()
            )
        else:
            services_ready = connected
            
        # Update indicators based on connection status
        if services_ready:
            self.service_status_indicator.setStyleSheet("background-color: green; border-radius: 8px;")
            self.service_status_label.setText("Services Connected")
        else:
            self.service_status_indicator.setStyleSheet("background-color: red; border-radius: 8px;")
            self.service_status_label.setText("Service Connection Issues")

    def _call_perform_calibration(self, model_type, degree):
        """Call the perform_calibration service with tracking."""
        if not self.perform_calibration_client.service_is_ready():
            self._node.get_logger().warn('perform_calibration service not available')
            self._update_service_status_indicators(False)
            QMessageBox.warning(self._widget, "Service Unavailable", 
                            "Calibration service is not available. Please try again later.")
            return
        
        # Create a unique ID for this calibration request
        call_id = f"{model_type}_{degree}"
        call_key = f"perform_calibration:{call_id}"
        
        with QMutexLocker(self.service_mutex):
            if call_key in self.pending_service_calls:
                # Already pending, don't call again
                self._node.get_logger().debug(f'Service call for perform_calibration({call_id}) already pending')
                QMessageBox.information(self._widget, "In Progress", 
                                    "Calibration is already in progress. Please wait.")
                return
        
        # Update UI directly in the main GUI thread to show process is happening
        self.calibrate_btn.setEnabled(False)
        self.calibrate_btn.setText("Calibrating...")
        
        # Proceed with service call
        request = PerformCalibration.Request()
        request.model_type = model_type
        request.degree = degree
        
        try:
            future = self.perform_calibration_client.call_async(request)
            
            # Use signal to handle completion in the main thread
            def completion_callback(future):
                self.signal_helper.calibration_complete_signal.emit(future)
            
            future.add_done_callback(completion_callback)
            
            # Track this call
            self._track_service_call("perform_calibration", call_id, future)
            
            # Set up a timeout timer in the main thread
            calibration_timeout = QTimer(self._widget)
            calibration_timeout.setSingleShot(True)
            calibration_timeout.timeout.connect(lambda: self._handle_service_timeout(call_key, "calibration"))
            calibration_timeout.start(15000)  # 15 second timeout
            
            # Store the timer
            with QMutexLocker(self.service_mutex):
                self.pending_service_calls[call_key + "_timer"] = calibration_timeout
                
            # Update service status
            self._update_service_status_indicators(True)
                
        except Exception as e:
            # Clean up UI in case of error
            self._node.get_logger().error(f'Error calling perform_calibration service: {e}')
            
            # Make sure the button is re-enabled in case of error
            self.calibrate_btn.setEnabled(True)
            self.calibrate_btn.setText("Calibrate")
            
            # Show error to user
            QMessageBox.warning(self._widget, "Service Error", 
                            f"Error performing calibration: {str(e)}")
            
            # Update service status
            self._update_service_status_indicators(False)
            
    def _perform_calibration_done(self, future):
        """Callback for perform_calibration service response."""
        # Re-enable the calibrate button
        self.calibrate_btn.setEnabled(True)
        self.calibrate_btn.setText("Calibrate")
        
        # Hide the progress dialog if it exists
        if hasattr(self, 'calibration_progress') and self.calibration_progress:
            self.calibration_progress.hide()
        
        try:
            response = future.result()
            
            # Update service status
            self._update_service_status_indicators(True)
            
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
                
                # Update UI via signal
                self.signal_helper.cal_results_update_signal.emit()
                QMessageBox.information(self._widget, "Success", response.message)
            else:
                QMessageBox.warning(self._widget, "Calibration Error", response.message)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            QMessageBox.critical(self._widget, "Error", f"Service call failed: {e}")
            
            # Update service status to indicate problem
            self._update_service_status_indicators(False)

    @Slot(object)
    def _handle_calibration_complete(self, future):
        """Handle calibration service completion in the main thread."""
        # Re-enable the calibrate button immediately
        self.calibrate_btn.setEnabled(True)
        self.calibrate_btn.setText("Calibrate")
        
        # Make sure any progress dialog is closed and cleaned up
        if hasattr(self, 'calibration_progress') and self.calibration_progress is not None:
            try:
                self.calibration_progress.close()
            except Exception as e:
                self._node.get_logger().error(f'Error closing calibration progress dialog: {e}')
            finally:
                self.calibration_progress = None
        
        try:
            response = future.result()
            
            # Update service status
            self._update_service_status_indicators(True)
            
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
                self.signal_helper.cal_results_update_signal.emit()
                
                # Clear calibration points from overlay after successful calibration
                self.image_view.clear_calibration_points()
                
                # Enable the radiometric mode toggle
                self.radio_toggle.setEnabled(True)
                
                # Show success message
                QMessageBox.information(self._widget, "Success", response.message)
            else:
                QMessageBox.warning(self._widget, "Calibration Error", response.message)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            QMessageBox.critical(self._widget, "Error", f"Service call failed: {e}")
            
            # Update service status to indicate problem
            self._update_service_status_indicators(False)
    
    def _call_clear_calibration_data(self):
        """Call the clear_calibration_data service with tracking."""
        if not self.clear_calibration_data_client.service_is_ready():
            self._node.get_logger().warn('clear_calibration_data service not available')
            
            # Even if service is unavailable, clear local data
            self.calibration_points = []
            self.calibration_model = None
            self.image_view.clear_calibration_points()
            self.signal_helper.points_table_update_signal.emit()
            self.signal_helper.cal_results_update_signal.emit()
            
            # Disable buttons
            self.remove_last_btn.setEnabled(False)
            if self.radiometric_mode:
                self.radio_toggle.setChecked(False)
            
            QMessageBox.warning(self._widget, "Service Unavailable", 
                            "Calibration service is not available. Local data was cleared but service data may persist.")
            return
        
        # Create a unique call ID
        call_key = "clear_calibration_data:confirm"
        
        with QMutexLocker(self.service_mutex):
            if call_key in self.pending_service_calls:
                # Already pending, don't call again
                self._node.get_logger().debug('Service call for clear_calibration_data already pending')
                return
        
        # Proceed with service call
        request = ClearCalibrationData.Request()
        request.confirm = True
        
        try:
            future = self.clear_calibration_data_client.call_async(request)
            # Track this call
            self._track_service_call("clear_calibration_data", "confirm", future)
            future.add_done_callback(self._clear_calibration_data_done)
            
            # Also clear local data immediately for responsive UI
            self.calibration_points = []
            self.calibration_model = None
            self.image_view.clear_calibration_points()
            self.signal_helper.points_table_update_signal.emit()
            self.signal_helper.cal_results_update_signal.emit()
            
            # Disable buttons
            self.remove_last_btn.setEnabled(False)
            if self.radiometric_mode:
                self.radio_toggle.setChecked(False)
                
        except Exception as e:
            self._node.get_logger().error(f'Error calling clear_calibration_data service: {e}')
            QMessageBox.warning(self._widget, "Service Error", 
                            f"Error clearing calibration data: {str(e)}")
    
    def _clear_calibration_data_done(self, future):
        """Callback for clear_calibration_data service response."""
        try:
            response = future.result()
            if response.success:
                # Clear local data
                self.calibration_points = []
                self.calibration_model = None
                
                # Clear image view points
                self.image_view.clear_calibration_points()
                
                # Update UI via signals
                self.signal_helper.points_table_update_signal.emit()
                self.signal_helper.cal_results_update_signal.emit()
                
                # Disable radiometric mode if it's enabled
                if self.radiometric_mode:
                    self.radio_toggle.setChecked(False)
                
                # Disable remove last button
                self.remove_last_btn.setEnabled(False)
                
                QMessageBox.information(self._widget, "Success", response.message)
            else:
                QMessageBox.warning(self._widget, "Error", response.message)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            QMessageBox.critical(self._widget, "Error", f"Service call failed: {e}")

    def _clear_and_rebuild_backend_points(self):
        """Clear all points on the backend and rebuild from UI points."""
        # First, verify the service is available
        if not self.clear_calibration_data_client.service_is_ready():
            self._node.get_logger().warn('clear_calibration_data service not available')
            self._update_service_status_indicators(False)
            return False

        # Create a unique call ID
        call_key = "clear_calibration_data:rebuild"
        
        # Proceed with service call to clear data
        request = ClearCalibrationData.Request()
        request.confirm = True
        
        try:
            # Call the service to clear all points
            future = self.clear_calibration_data_client.call_async(request)
            
            # Track this call with a timeout
            self._track_service_call("clear_calibration_data", "rebuild", future)
            
            # Set up a timeout handler
            clear_timeout = QTimer(self._widget)
            clear_timeout.setSingleShot(True)
            clear_timeout.timeout.connect(lambda: self._handle_service_timeout(call_key, "clear_points"))
            clear_timeout.start(5000)  # 5 second timeout
            
            # Store the timer
            with QMutexLocker(self.service_mutex):
                self.pending_service_calls[call_key + "_timer"] = clear_timeout
            
            # Define the callback to add points after clearing
            def clear_done_add_points(clear_future):
                try:
                    # Get clear response
                    clear_response = clear_future.result()
                    
                    if clear_response.success:
                        self._node.get_logger().info("Successfully cleared backend data, now rebuilding")
                        
                        # Add all points that remain in UI
                        for point in self.calibration_points:
                            self._call_add_calibration_point(
                                point['x'], 
                                point['y'], 
                                point['raw_value'], 
                                point['reference_temp']
                            )
                            # Small delay to prevent overwhelming the service
                            time.sleep(0.1)
                    else:
                        self._node.get_logger().error(f"Failed to clear backend data: {clear_response.message}")
                        self._update_service_status_indicators(False)
                except Exception as e:
                    self._node.get_logger().error(f"Error in clear_done_add_points: {e}")
                    self._update_service_status_indicators(False)
            
            # Add the callback
            future.add_done_callback(clear_done_add_points)
            
            # Update service status to show we're working
            self._update_service_status_indicators(True)
            return True
            
        except Exception as e:
            self._node.get_logger().error(f'Error calling clear_calibration_data service: {e}')
            self._update_service_status_indicators(False)
            return False

    def _handle_service_timeout(self, call_key, service_type):
        """Handle timeouts for service calls."""
        with QMutexLocker(self.service_mutex):
            if call_key in self.pending_service_calls:
                # Service call still pending after timeout
                self._node.get_logger().warn(f'Service call timed out: {call_key}')
                
                # Clean up the timer
                timer_key = call_key + "_timer"
                if timer_key in self.pending_service_calls:
                    # Stop the timer if it's still running
                    timer = self.pending_service_calls[timer_key]
                    if timer.isActive():
                        timer.stop()
                    # Remove from tracking
                    del self.pending_service_calls[timer_key]
                
                # Take appropriate action based on service type
                if service_type == "calibration":
                    # Re-enable the calibrate button
                    self.calibrate_btn.setEnabled(True)
                    self.calibrate_btn.setText("Calibrate")
                    
                    # Clean up any progress dialog that might exist
                    if hasattr(self, 'calibration_progress') and self.calibration_progress is not None:
                        try:
                            self.calibration_progress.close()
                        except Exception:
                            pass
                        self.calibration_progress = None
                    
                    # Show a message to the user
                    QMessageBox.warning(self._widget, "Calibration Timeout", 
                                    "The calibration operation is taking too long. Please try again.")
                # ... other service types ...
                
                # Remove the pending service call from tracking
                if call_key in self.pending_service_calls:
                    del self.pending_service_calls[call_key]


    def _call_raw_to_temperature(self, raw_value):
        """Call the raw_to_temperature service with tracking."""
        if not self.raw_to_temperature_client.service_is_ready():
            self._node.get_logger().warn('raw_to_temperature service not available')
            self.signal_helper.temp_update_signal.emit("Temperature: Service unavailable")
            return
        
        # Create a unique call ID
        call_key = f"raw_to_temperature:{raw_value}"
        
        with QMutexLocker(self.service_mutex):
            if call_key in self.pending_service_calls:
                # Already pending, don't call again
                self._node.get_logger().debug(f'Service call for raw_to_temperature({raw_value}) already pending')
                return
        
        # Proceed with service call
        request = RawToTemperature.Request()
        request.raw_value = raw_value
        
        try:
            future = self.raw_to_temperature_client.call_async(request)
            # Track this call
            self._track_service_call("raw_to_temperature", str(raw_value), future)
            future.add_done_callback(self._raw_to_temperature_done)
        except Exception as e:
            self._node.get_logger().error(f'Error calling raw_to_temperature service: {e}')
            self.signal_helper.temp_update_signal.emit(f"Temperature: Error ({str(e)})")
    
    def _raw_to_temperature_done(self, future):
        """Callback for raw_to_temperature service response."""
        try:
            response = future.result()
            if response.success:
                temp = response.temperature
                self.signal_helper.temp_update_signal.emit(f"Temperature: {temp:.1f}°C")
            else:
                self.signal_helper.temp_update_signal.emit("Temperature: -")
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            self.signal_helper.temp_update_signal.emit("Temperature: Error")
    
    def _call_save_calibration_model(self, filename):
        """Call the save_calibration_model service with tracking."""
        if not self.save_calibration_model_client.service_is_ready():
            self._node.get_logger().warn('save_calibration_model service not available')
            QMessageBox.warning(self._widget, "Service Unavailable", 
                            "Save calibration service is not available. Please try again later.")
            return
        
        # Create a unique ID for this save request
        base_filename = os.path.basename(filename)
        call_key = f"save_calibration_model:{base_filename}"
        
        with QMutexLocker(self.service_mutex):
            if call_key in self.pending_service_calls:
                # Already pending, don't call again
                self._node.get_logger().debug(f'Service call for save_calibration_model({base_filename}) already pending')
                QMessageBox.information(self._widget, "In Progress", 
                                    "Save operation is already in progress. Please wait.")
                return
        
        # Proceed with service call
        request = SaveCalibrationModel.Request()
        request.filename = base_filename
        
        try:
            future = self.save_calibration_model_client.call_async(request)
            # Track this call
            self._track_service_call("save_calibration_model", base_filename, future)
            future.add_done_callback(self._save_calibration_model_done)
            
            # Optionally, show a "saving" message
            self.export_btn.setEnabled(False)
            self.export_btn.setText("Saving...")
        except Exception as e:
            self._node.get_logger().error(f'Error calling save_calibration_model service: {e}')
            QMessageBox.warning(self._widget, "Service Error", 
                            f"Error saving calibration model: {str(e)}")
    
    def _save_calibration_model_done(self, future):
        """Callback for save_calibration_model service response."""
        # Reset the export button
        self.export_btn.setEnabled(True)
        self.export_btn.setText("Export Calibration")
        
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
        """Call the load_calibration_model service with tracking."""
        if not self.load_calibration_model_client.service_is_ready():
            self._node.get_logger().warn('load_calibration_model service not available')
            QMessageBox.warning(self._widget, "Service Unavailable", 
                            "Load calibration service is not available. Please try again later.")
            return
        
        # Create a unique ID for this load request
        base_path = os.path.basename(path)
        call_key = f"load_calibration_model:{base_path}"
        
        with QMutexLocker(self.service_mutex):
            if call_key in self.pending_service_calls:
                # Already pending, don't call again
                self._node.get_logger().debug(f'Service call for load_calibration_model({base_path}) already pending')
                QMessageBox.information(self._widget, "In Progress", 
                                    "Load operation is already in progress. Please wait.")
                return
        
        # Proceed with service call
        request = LoadCalibrationModel.Request()
        request.path = path
        
        try:
            future = self.load_calibration_model_client.call_async(request)
            # Track this call
            self._track_service_call("load_calibration_model", base_path, future)
            future.add_done_callback(self._load_calibration_model_done)
        except Exception as e:
            self._node.get_logger().error(f'Error calling load_calibration_model service: {e}')
            QMessageBox.warning(self._widget, "Service Error", 
                            f"Error loading calibration model: {str(e)}")
    
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
                
                # Update UI via signal
                self.signal_helper.cal_results_update_signal.emit()
                QMessageBox.information(self._widget, "Success", response.message)
            else:
                QMessageBox.warning(self._widget, "Error", response.message)
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')
            QMessageBox.critical(self._widget, "Error", f"Service call failed: {e}")
    
    def _try_reconnect_services(self):
        """Try to reconnect to any unavailable services and update UI indicators."""
        services_to_check = [
            (self.get_raw_value_client, 'get_raw_value'),
            (self.add_calibration_point_client, 'add_calibration_point'),
            (self.perform_calibration_client, 'perform_calibration'),
            (self.clear_calibration_data_client, 'clear_calibration_data'),
            (self.raw_to_temperature_client, 'raw_to_temperature'),
            (self.save_calibration_model_client, 'save_calibration_model'),
            (self.load_calibration_model_client, 'load_calibration_model')
        ]
        
        all_ready = True
        reconnected = False
        
        for client, name in services_to_check:
            if client is not None:
                if not client.service_is_ready():
                    all_ready = False
                    if client.wait_for_service(timeout_sec=0.1):
                        self._node.get_logger().info(f'Successfully reconnected to {name} service')
                        reconnected = True
                else:
                    # Service is already ready
                    pass
        
        # Update the UI status indicators based on service availability
        self._update_service_status_indicators(all_ready)
        
        # If any service was reconnected, update UI
        if reconnected:
            self._node.get_logger().info("Service reconnected - updating UI")

    def resizeEvent(self, event):
        """Handle resize events."""
        # Skip resizeEvent since this can cause issues with cross-thread pixel operations
        pass
        
    def save_settings(self, plugin_settings, instance_settings):
        """Save the intrinsic configuration of the plugin."""
        # You can save intrinsic configuration here
        pass
    
    def restore_settings(self, plugin_settings, instance_settings):
        """Restore the intrinsic configuration of the plugin."""
        # You can restore intrinsic configuration here
        pass

    def shutdown_plugin(self):
        """
        Callback when the plugin is closed.
        Unregisters all publishers and subscribers.
        """
        try:
            # Unregister subscribers
            self.image_16bit_sub = None
            self.image_8bit_sub = None
            
            # Stop all timers
            if hasattr(self, 'update_timer') and self.update_timer is not None:
                self.update_timer.stop()
            
            if hasattr(self, 'service_check_timer') and self.service_check_timer is not None:
                self.service_check_timer.stop()
                
            if hasattr(self, 'focus_check_timer') and self.focus_check_timer is not None:
                self.focus_check_timer.stop()
                
            # Clear any stored images to free memory
            self.current_image = None
            if hasattr(self, 'last_8bit_image'):
                self.last_8bit_image = None
                
            # Log shutdown
            if hasattr(self, '_node') and self._node is not None:
                self._node.get_logger().info("Thermal calibration plugin shutdown")
                
        except Exception as e:
            if hasattr(self, '_node') and self._node is not None:
                self._node.get_logger().error(f"Error during plugin shutdown: {e}")

    def _on_temp_input_editing_finished(self):
        """Handle when user presses Enter in the temperature input field.
        This is a method of the ThermalCalibrationPlugin class."""
        # Check if the editing finished was because Enter was pressed
        # We can tell if the widget no longer has focus and no other widget has focus
        # which indicates user pressed Enter to commit
        if not self.temp_input.hasFocus() and self._widget.focusWidget() is None:
            self._node.get_logger().info("Enter key pressed to commit temperature - triggering save")
            self._on_save_temp_clicked()
        else:
            # Just update internal state without creating points
            self._node.get_logger().debug("Temperature input focus changed - not saving")

    @Slot(str)
    def _cleanup_timer_main_thread(self, timer_key):
        """Handle timer cleanup in the main thread."""
        try:
            with QMutexLocker(self.service_mutex):
                if timer_key in self.pending_service_calls:
                    timer = self.pending_service_calls[timer_key]
                    if timer.isActive():
                        timer.stop()
                    del self.pending_service_calls[timer_key]
                    self._node.get_logger().debug(f'Timer {timer_key} cleaned up')
        except Exception as e:
            self._node.get_logger().error(f'Error cleaning up timer {timer_key}: {e}')


def main(args=None):
    """Main function to allow standalone operation of the plugin."""
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create a simple node for the plugin
    node = rclpy.create_node('thermal_calibration_rqt_node')
    
    # Create the plugin
    from qt_gui.main import Main
    main = Main()
    sys.exit(main.main(args=args, standalone='thermal_calibration_rqt.thermal_calibration_plugin:ThermalCalibrationPlugin'))

if __name__ == '__main__':
    main()