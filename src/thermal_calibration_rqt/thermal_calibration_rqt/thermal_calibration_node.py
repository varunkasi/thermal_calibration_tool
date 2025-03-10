#!/usr/bin/env python3

import os
import json
import numpy as np
from datetime import datetime
import cv2
import matplotlib.pyplot as plt
import io

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from thermal_calibration_interfaces.msg import CalibrationPoint, CalibrationModel
from thermal_calibration_interfaces.srv import (
    GetRawValue, AddCalibrationPoint, PerformCalibration, ClearCalibrationData, 
    RawToTemperature, SaveCalibrationModel, LoadCalibrationModel
)


class ThermalCalibrationNode(Node):
    """
    ROS node for thermal camera calibration.
    
    This node provides services for:
    1. Getting raw thermal values from coordinates
    2. Adding calibration points
    3. Performing calibration
    4. Converting raw values to temperatures
    5. Saving and loading calibration models
    """
    
    def __init__(self):
        super().__init__('thermal_calibration_node')
        
        # Initialize variables
        self.cv_bridge = CvBridge()
        self.thermal_image = None
        self.calibration_points = []
        self.calibration_model = None
        
        # Set up default data directory
        self.data_dir = os.path.expanduser('~/thermal_calibration_data')
        
        # Attempt to get data directory from parameter
        self.declare_parameter('data_dir', self.data_dir)
        self.data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        
        # Create data directory if it doesn't exist
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Create QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to thermal image topic
        self.declare_parameter('thermal_image_topic', '/mono16_converter/image')
        image_topic = self.get_parameter('thermal_image_topic').get_parameter_value().string_value
        
        self.thermal_sub = self.create_subscription(
            Image,
            image_topic,
            self._thermal_image_callback,
            qos
        )
        
        # Create services
        self.get_raw_value_srv = self.create_service(
            GetRawValue,
            'get_raw_value',
            self._get_raw_value_callback
        )
        
        self.add_calibration_point_srv = self.create_service(
            AddCalibrationPoint,
            'add_calibration_point',
            self._add_calibration_point_callback
        )
        
        self.perform_calibration_srv = self.create_service(
            PerformCalibration,
            'perform_calibration',
            self._perform_calibration_callback
        )
        
        self.clear_calibration_data_srv = self.create_service(
            ClearCalibrationData,
            'clear_calibration_data',
            self._clear_calibration_data_callback
        )
        
        self.raw_to_temperature_srv = self.create_service(
            RawToTemperature,
            'raw_to_temperature',
            self._raw_to_temperature_callback
        )
        
        self.save_calibration_model_srv = self.create_service(
            SaveCalibrationModel,
            'save_calibration_model',
            self._save_calibration_model_callback
        )
        
        self.load_calibration_model_srv = self.create_service(
            LoadCalibrationModel,
            'load_calibration_model',
            self._load_calibration_model_callback
        )
        
        self.get_logger().info(f'Thermal calibration node started')
        self.get_logger().info(f'Using data directory: {self.data_dir}')
        self.get_logger().info(f'Subscribed to thermal image topic: {image_topic}')
    
    def _thermal_image_callback(self, msg):
        """Callback for thermal image."""
        try:
            self.thermal_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
            self.get_logger().debug(f'Received thermal image: {self.thermal_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error converting thermal image: {e}')
    
    def _get_raw_value_callback(self, request, response):
        """Service callback to get raw value at coordinates."""
        if self.thermal_image is None:
            response.success = False
            response.message = 'No thermal image available'
            response.raw_value = 0
            return response
        
        try:
            x, y = request.x, request.y
            
            # Check if coordinates are within image bounds
            h, w = self.thermal_image.shape
            if 0 <= x < w and 0 <= y < h:
                raw_value = int(self.thermal_image[y, x])
                response.success = True
                response.message = f'Raw value at ({x}, {y}): {raw_value}'
                response.raw_value = raw_value
                
                self.get_logger().info(f'GetRawValue: ({x}, {y}) -> {raw_value}')
            else:
                response.success = False
                response.message = f'Coordinates ({x}, {y}) out of bounds. Image size: {w}x{h}'
                response.raw_value = 0
                
                self.get_logger().warn(f'GetRawValue: Coordinates ({x}, {y}) out of bounds. Image size: {w}x{h}')
        except Exception as e:
            response.success = False
            response.message = f'Error getting raw value: {str(e)}'
            response.raw_value = 0
            
            self.get_logger().error(f'GetRawValue error: {e}')
        
        return response
    
    def _add_calibration_point_callback(self, request, response):
        """Service callback to add a calibration point."""
        try:
            # Get raw value if not provided or zero
            raw_value = request.raw_value
            if raw_value == 0 and self.thermal_image is not None:
                # Try to read from the image
                try:
                    x, y = request.x, request.y
                    raw_value = int(self.thermal_image[y, x])
                    self.get_logger().info(f'Using raw value from image: {raw_value}')
                except IndexError:
                    response.success = False
                    response.message = f'Coordinates ({x}, {y}) out of bounds'
                    response.point_id = 0
                    return response
            
            # Create a new calibration point
            point_id = len(self.calibration_points) + 1
            point = {
                'id': point_id,
                'x': request.x,
                'y': request.y,
                'raw_value': raw_value,
                'reference_temp': request.reference_temp,
                'timestamp': datetime.now().isoformat()
            }
            
            # Add to list
            self.calibration_points.append(point)
            
            response.success = True
            response.message = f'Added calibration point with ID {point_id}'
            response.point_id = point_id
            
            self.get_logger().info(f'Added calibration point: {point}')
            
        except Exception as e:
            response.success = False
            response.message = f'Error adding calibration point: {str(e)}'
            response.point_id = 0
            
            self.get_logger().error(f'Error adding calibration point: {e}')
        
        return response
    
    def _perform_calibration_callback(self, request, response):
        """Service callback to perform calibration."""
        if len(self.calibration_points) < 2:
            response.success = False
            response.message = 'At least 2 calibration points are required'
            self.get_logger().warn('Calibration failed: Need at least 2 points')
            return response
        
        try:
            model_type = request.model_type
            degree = request.degree
            
            # Extract raw values and temperatures from calibration points
            raw_values = np.array([p['raw_value'] for p in self.calibration_points])
            temps = np.array([p['reference_temp'] for p in self.calibration_points])
            
            # Sort points by raw value for better visualization
            sort_idx = np.argsort(raw_values)
            raw_values = raw_values[sort_idx]
            temps = temps[sort_idx]
            
            self.get_logger().info(f'Performing {model_type} calibration (degree {degree}) with {len(raw_values)} points')
            
            # Perform calibration based on model type
            if model_type == "polynomial":
                # Fit polynomial
                coeffs = np.polyfit(raw_values, temps, degree)
                
                # Calculate predicted values
                p = np.poly1d(coeffs)
                predicted_temps = p(raw_values)
                
                # Calculate R-squared and RMSE
                residuals = temps - predicted_temps
                ss_res = np.sum(residuals**2)
                ss_tot = np.sum((temps - np.mean(temps))**2)
                
                r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0
                rmse = np.sqrt(np.mean(residuals**2))
                
                # Create model dictionary
                self.calibration_model = {
                    'model_type': model_type,
                    'degree': degree,
                    'parameters': coeffs.tolist(),
                    'r_squared': r_squared,
                    'rmse': rmse,
                    'points_count': len(self.calibration_points),
                    'raw_value_range': [int(min(raw_values)), int(max(raw_values))],
                    'timestamp': datetime.now().isoformat()
                }
                
                # Generate a scatter plot of the calibration
                self._generate_calibration_plot(raw_values, temps, predicted_temps)
                
                response.success = True
                response.message = f'Calibration successful: {model_type} (degree {degree}), R²={r_squared:.4f}, RMSE={rmse:.2f}°C'
                response.model_parameters = coeffs.tolist()
                response.r_squared = r_squared
                response.rmse = rmse
                
                self.get_logger().info(f'Calibration successful:')
                self.get_logger().info(f'  Model type: {model_type} (degree {degree})')
                self.get_logger().info(f'  Parameters: {coeffs.tolist()}')
                self.get_logger().info(f'  R²: {r_squared:.4f}')
                self.get_logger().info(f'  RMSE: {rmse:.2f}°C')
                
            else:
                response.success = False
                response.message = f'Unsupported model type: {model_type}'
                self.get_logger().error(f'Calibration failed: Unsupported model type: {model_type}')
        
        except Exception as e:
            response.success = False
            response.message = f'Error performing calibration: {str(e)}'
            
            self.get_logger().error(f'Calibration error: {e}')
        
        return response
    
    def _generate_calibration_plot(self, raw_values, temps, predicted_temps):
        """Generate a plot of the calibration and save it."""
        try:
            # Create figure
            plt.figure(figsize=(10, 6))
            
            # Plot measured points
            plt.scatter(raw_values, temps, color='blue', label='Measured points')
            
            # Plot calibration curve
            # Generate more points for smooth curve
            x_smooth = np.linspace(min(raw_values), max(raw_values), 100)
            p = np.poly1d(self.calibration_model['parameters'])
            y_smooth = p(x_smooth)
            plt.plot(x_smooth, y_smooth, 'r-', label='Calibration curve')
            
            # Add labels and title
            plt.xlabel('Raw sensor value')
            plt.ylabel('Temperature (°C)')
            plt.title('Thermal Camera Calibration')
            plt.grid(True)
            plt.legend()
            
            # Save plot
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            plot_filename = os.path.join(self.data_dir, f'calibration_plot_{timestamp}.png')
            plt.savefig(plot_filename)
            plt.close()
            
            self.get_logger().info(f'Calibration plot saved to: {plot_filename}')
            
        except Exception as e:
            self.get_logger().error(f'Error generating calibration plot: {e}')
    
    def _clear_calibration_data_callback(self, request, response):
        """Service callback to clear calibration data."""
        if not request.confirm:
            response.success = False
            response.message = 'Confirmation required to clear calibration data'
            return response
        
        try:
            # Clear calibration points and model
            self.calibration_points = []
            self.calibration_model = None
            
            response.success = True
            response.message = 'Calibration data cleared'
            
            self.get_logger().info('Calibration data cleared')
            
        except Exception as e:
            response.success = False
            response.message = f'Error clearing calibration data: {str(e)}'
            
            self.get_logger().error(f'Error clearing calibration data: {e}')
        
        return response
    
    def _raw_to_temperature_callback(self, request, response):
        """Service callback to convert raw value to temperature."""
        if self.calibration_model is None:
            response.success = False
            response.message = 'No calibration model available'
            response.temperature = 0.0
            return response
        
        try:
            raw_value = request.raw_value
            
            # Apply calibration model
            if self.calibration_model['model_type'] == "polynomial":
                coeffs = self.calibration_model['parameters']
                
                # Calculate temperature using polynomial
                p = np.poly1d(coeffs)
                temp = float(p(raw_value))
                
                response.success = True
                response.message = f'Raw value {raw_value} converted to {temp:.1f}°C'
                response.temperature = temp
                
                self.get_logger().debug(f'Raw value {raw_value} -> {temp:.1f}°C')
                
            else:
                response.success = False
                response.message = f'Unsupported model type: {self.calibration_model["model_type"]}'
                response.temperature = 0.0
                
        except Exception as e:
            response.success = False
            response.message = f'Error converting raw value to temperature: {str(e)}'
            response.temperature = 0.0
            
            self.get_logger().error(f'Error in raw_to_temperature: {e}')
        
        return response
    
    def _save_calibration_model_callback(self, request, response):
        """Service callback to save calibration model."""
        if self.calibration_model is None:
            response.success = False
            response.message = 'No calibration model available to save'
            response.path = ''
            return response
        
        try:
            # Generate filename if not provided
            filename = request.filename
            if not filename:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f'thermal_calibration_{timestamp}.json'
            
            # Ensure .json extension
            if not filename.endswith('.json'):
                filename += '.json'
            
            # Create full path
            file_path = os.path.join(self.data_dir, filename)
            
            # Create dict with calibration model and points
            save_data = {
                'model': self.calibration_model,
                'points': self.calibration_points,
                'saved_at': datetime.now().isoformat()
            }
            
            # Save to file
            with open(file_path, 'w') as f:
                json.dump(save_data, f, indent=2)
            
            response.success = True
            response.message = f'Calibration model saved to: {file_path}'
            response.path = file_path
            
            self.get_logger().info(f'Model saved to: {file_path}')
            
        except Exception as e:
            response.success = False
            response.message = f'Error saving calibration model: {str(e)}'
            response.path = ''
            
            self.get_logger().error(f'Error saving model: {e}')
        
        return response
    
    def _load_calibration_model_callback(self, request, response):
        """Service callback to load calibration model."""
        try:
            file_path = request.path
            
            # Check if file exists
            if not os.path.isfile(file_path):
                response.success = False
                response.message = f'File not found: {file_path}'
                return response
            
            # Load from file
            with open(file_path, 'r') as f:
                loaded_data = json.load(f)
            
            # Extract model and points
            if 'model' in loaded_data:
                self.calibration_model = loaded_data['model']
            else:
                # Backwards compatibility for older saved files that just contained the model
                self.calibration_model = loaded_data
            
            # Load points if available
            if 'points' in loaded_data:
                self.calibration_points = loaded_data['points']
            
            response.success = True
            response.message = f'Calibration model loaded from: {file_path}'
            response.model_type = self.calibration_model['model_type']
            response.model_parameters = self.calibration_model['parameters']
            
            self.get_logger().info(f'Model loaded from: {file_path}')
            self.get_logger().info(f'Model type: {self.calibration_model["model_type"]}')
            self.get_logger().info(f'Parameters: {self.calibration_model["parameters"]}')
            if 'r_squared' in self.calibration_model:
                self.get_logger().info(f'R²: {self.calibration_model["r_squared"]:.4f}')
            if 'rmse' in self.calibration_model:
                self.get_logger().info(f'RMSE: {self.calibration_model["rmse"]:.2f}°C')
            
        except Exception as e:
            response.success = False
            response.message = f'Error loading calibration model: {str(e)}'
            
            self.get_logger().error(f'Error loading model: {e}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    thermal_calibration_node = ThermalCalibrationNode()
    
    try:
        rclpy.spin(thermal_calibration_node)
    except KeyboardInterrupt:
        pass
    finally:
        thermal_calibration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()