#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Data management utilities for the Thermal Camera Calibration Tool.

This module provides functions for saving, loading, and managing calibration data
and other persistent application data.
"""

import os
import json
import time
import shutil
from typing import Optional, Dict, Any, List, Tuple, Union
from datetime import datetime

from loguru import logger


class DataManager:
    """Manages data storage and retrieval for the application."""
    
    def __init__(self, base_directory: str = "~/thermal_calibration_data"):
        """
        Initialize the data manager.
        
        Args:
            base_directory (str, optional): Base directory for data storage.
                Defaults to "~/thermal_calibration_data".
        """
        # Expand user directory if needed
        self.base_directory = os.path.expanduser(base_directory)
        
        # Ensure base directory exists
        os.makedirs(self.base_directory, exist_ok=True)
        
        # Create subdirectories
        self.calibration_dir = os.path.join(self.base_directory, "calibrations")
        self.images_dir = os.path.join(self.base_directory, "images")
        self.exports_dir = os.path.join(self.base_directory, "exports")
        self.logs_dir = os.path.join(self.base_directory, "logs")
        
        # Create subdirectories
        os.makedirs(self.calibration_dir, exist_ok=True)
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.exports_dir, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)
        
        logger.debug(f"Initialized data manager with base directory: {self.base_directory}")
    
    def save_calibration(
        self, 
        calibration_data: Dict[str, Any], 
        filename: Optional[str] = None,
        camera_info: Optional[Dict[str, Any]] = None
    ) -> Tuple[bool, str]:
        """
        Save calibration data to a file.
        
        Args:
            calibration_data (Dict[str, Any]): Calibration data to save.
            filename (Optional[str], optional): Filename to save to. If None, a timestamp-based
                filename will be generated. Defaults to None.
            camera_info (Optional[Dict[str, Any]], optional): Camera information to include
                in the saved data. Defaults to None.
        
        Returns:
            Tuple[bool, str]: Success flag and path to the saved file.
        """
        try:
            # Generate filename if not provided
            if not filename:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"calibration_{timestamp}.json"
            
            # Ensure .json extension
            if not filename.lower().endswith('.json'):
                filename += '.json'
            
            # Full path to calibration file
            filepath = os.path.join(self.calibration_dir, filename)
            
            # Prepare data for saving
            save_data = {
                "calibration": calibration_data,
                "camera_info": camera_info or {},
                "metadata": {
                    "created_at": time.time(),
                    "created_at_iso": datetime.now().isoformat(),
                    "version": "1.0.0"
                }
            }
            
            # Save to file
            with open(filepath, 'w') as f:
                json.dump(save_data, f, indent=2)
            
            logger.info(f"Saved calibration data to {filepath}")
            return True, filepath
            
        except Exception as e:
            logger.error(f"Error saving calibration data: {e}")
            return False, ""
    
    def load_calibration(self, filename: str) -> Tuple[bool, Dict[str, Any]]:
        """
        Load calibration data from a file.
        
        Args:
            filename (str): Filename or path to load from.
        
        Returns:
            Tuple[bool, Dict[str, Any]]: Success flag and loaded calibration data.
        """
        try:
            # Check if filename is a full path or just a filename
            if os.path.dirname(filename):
                filepath = filename
            else:
                # Ensure .json extension
                if not filename.lower().endswith('.json'):
                    filename += '.json'
                filepath = os.path.join(self.calibration_dir, filename)
            
            # Check if file exists
            if not os.path.isfile(filepath):
                logger.warning(f"Calibration file not found: {filepath}")
                return False, {}
            
            # Load from file
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # Check if data has the expected structure
            if "calibration" not in data:
                logger.warning(f"Invalid calibration file format: {filepath}")
                return False, {}
            
            logger.info(f"Loaded calibration data from {filepath}")
            return True, data["calibration"]
            
        except Exception as e:
            logger.error(f"Error loading calibration data: {e}")
            return False, {}
    
    def get_available_calibrations(self) -> List[Dict[str, Any]]:
        """
        Get a list of available calibration files with metadata.
        
        Returns:
            List[Dict[str, Any]]: List of calibration file information.
        """
        calibrations = []
        
        try:
            # List all JSON files in the calibration directory
            for filename in os.listdir(self.calibration_dir):
                if filename.lower().endswith('.json'):
                    filepath = os.path.join(self.calibration_dir, filename)
                    
                    try:
                        # Get basic file info
                        file_info = {
                            "filename": filename,
                            "filepath": filepath,
                            "size": os.path.getsize(filepath),
                            "modified": os.path.getmtime(filepath),
                            "modified_iso": datetime.fromtimestamp(
                                os.path.getmtime(filepath)
                            ).isoformat()
                        }
                        
                        # Try to load metadata
                        with open(filepath, 'r') as f:
                            data = json.load(f)
                            if "metadata" in data:
                                file_info["metadata"] = data["metadata"]
                            if "camera_info" in data:
                                file_info["camera_info"] = data["camera_info"]
                        
                        calibrations.append(file_info)
                    except Exception as e:
                        logger.warning(f"Error reading calibration file {filename}: {e}")
            
            # Sort by modification time (newest first)
            calibrations.sort(key=lambda x: x["modified"], reverse=True)
            
        except Exception as e:
            logger.error(f"Error listing calibration files: {e}")
        
        return calibrations
    
    def export_calibration(
        self, 
        calibration_data: Dict[str, Any], 
        export_path: str,
        camera_info: Optional[Dict[str, Any]] = None,
        include_metadata: bool = True
    ) -> bool:
        """
        Export calibration data to a file in the exports directory.
        
        Args:
            calibration_data (Dict[str, Any]): Calibration data to export.
            export_path (str): Path to export to.
            camera_info (Optional[Dict[str, Any]], optional): Camera information to include.
                Defaults to None.
            include_metadata (bool, optional): Whether to include metadata in the export.
                Defaults to True.
        
        Returns:
            bool: Success flag.
        """
        try:
            # Ensure exports directory exists
            os.makedirs(self.exports_dir, exist_ok=True)
            
            # If export_path is not an absolute path, make it relative to exports_dir
            if not os.path.isabs(export_path):
                export_path = os.path.join(self.exports_dir, export_path)
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(os.path.abspath(export_path)), exist_ok=True)
            
            # Prepare data for export
            export_data = {"calibration": calibration_data}
            
            if camera_info and include_metadata:
                export_data["camera_info"] = camera_info
            
            if include_metadata:
                export_data["metadata"] = {
                    "exported_at": time.time(),
                    "exported_at_iso": datetime.now().isoformat(),
                    "version": "1.0.0"
                }
            
            # Save to file
            with open(export_path, 'w') as f:
                json.dump(export_data, f, indent=2)
            
            logger.info(f"Exported calibration data to {export_path}")
            return True
            
        except Exception as e:
            logger.error(f"Error exporting calibration data: {e}")
            return False
    
    def save_thermal_image(
        self, 
        image_data: bytes,
        metadata: Optional[Dict[str, Any]] = None,
        filename: Optional[str] = None
    ) -> Tuple[bool, str]:
        """
        Save a thermal image to a file.
        
        Args:
            image_data (bytes): Raw image data.
            metadata (Optional[Dict[str, Any]], optional): Image metadata. Defaults to None.
            filename (Optional[str], optional): Filename to save to. Defaults to None.
        
        Returns:
            Tuple[bool, str]: Success flag and path to the saved file.
        """
        try:
            # Generate filename if not provided
            if not filename:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"thermal_{timestamp}.bin"
            
            # Ensure proper extension
            if not (filename.lower().endswith('.bin') or filename.lower().endswith('.raw')):
                filename += '.bin'
            
            # Full path to image file
            filepath = os.path.join(self.images_dir, filename)
            
            # Save image data
            with open(filepath, 'wb') as f:
                f.write(image_data)
            
            # Save metadata if provided
            if metadata:
                metadata_filepath = os.path.splitext(filepath)[0] + '.json'
                with open(metadata_filepath, 'w') as f:
                    json.dump(metadata, f, indent=2)
            
            logger.info(f"Saved thermal image to {filepath}")
            return True, filepath
            
        except Exception as e:
            logger.error(f"Error saving thermal image: {e}")
            return False, ""
    
    def load_thermal_image(self, filename: str) -> Tuple[bool, bytes, Optional[Dict[str, Any]]]:
        """
        Load a thermal image from a file.
        
        Args:
            filename (str): Filename or path to load from.
        
        Returns:
            Tuple[bool, bytes, Optional[Dict[str, Any]]]: Success flag, image data, and metadata.
        """
        try:
            # Check if filename is a full path or just a filename
            if os.path.dirname(filename):
                filepath = filename
            else:
                filepath = os.path.join(self.images_dir, filename)
            
            # Check if file exists
            if not os.path.isfile(filepath):
                logger.warning(f"Thermal image file not found: {filepath}")
                return False, b'', None
            
            # Load image data
            with open(filepath, 'rb') as f:
                image_data = f.read()
            
            # Try to load metadata if available
            metadata = None
            metadata_filepath = os.path.splitext(filepath)[0] + '.json'
            if os.path.isfile(metadata_filepath):
                with open(metadata_filepath, 'r') as f:
                    metadata = json.load(f)
            
            logger.info(f"Loaded thermal image from {filepath}")
            return True, image_data, metadata
            
        except Exception as e:
            logger.error(f"Error loading thermal image: {e}")
            return False, b'', None
    
    def backup_data(self, backup_path: Optional[str] = None) -> Tuple[bool, str]:
        """
        Create a backup of all data.
        
        Args:
            backup_path (Optional[str], optional): Path to save the backup to.
                Defaults to None.
        
        Returns:
            Tuple[bool, str]: Success flag and path to the backup file.
        """
        try:
            # Generate backup filename if not provided
            if not backup_path:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                backup_filename = f"thermal_calibration_backup_{timestamp}.zip"
                backup_path = os.path.join(self.base_directory, backup_filename)
            
            # Create backup archive
            shutil.make_archive(
                os.path.splitext(backup_path)[0],  # Remove .zip extension if present
                'zip',
                self.base_directory
            )
            
            backup_path = os.path.splitext(backup_path)[0] + '.zip'
            
            logger.info(f"Created data backup at {backup_path}")
            return True, backup_path
            
        except Exception as e:
            logger.error(f"Error creating data backup: {e}")
            return False, ""
    
    def restore_data(self, backup_path: str) -> bool:
        """
        Restore data from a backup.
        
        Args:
            backup_path (str): Path to the backup file.
        
        Returns:
            bool: Success flag.
        """
        try:
            # Check if backup file exists
            if not os.path.isfile(backup_path):
                logger.warning(f"Backup file not found: {backup_path}")
                return False
            
            # Create temp directory for restoration
            temp_dir = os.path.join(self.base_directory, "temp_restore")
            os.makedirs(temp_dir, exist_ok=True)
            
            # Extract backup to temp directory
            shutil.unpack_archive(backup_path, temp_dir)
            
            # Check for calibration data in the backup
            temp_calibration_dir = os.path.join(temp_dir, "calibrations")
            if os.path.isdir(temp_calibration_dir):
                # Copy calibration files
                for filename in os.listdir(temp_calibration_dir):
                    src = os.path.join(temp_calibration_dir, filename)
                    dst = os.path.join(self.calibration_dir, filename)
                    if os.path.isfile(src):
                        shutil.copy2(src, dst)
            
            # Check for image data in the backup
            temp_images_dir = os.path.join(temp_dir, "images")
            if os.path.isdir(temp_images_dir):
                # Copy image files
                for filename in os.listdir(temp_images_dir):
                    src = os.path.join(temp_images_dir, filename)
                    dst = os.path.join(self.images_dir, filename)
                    if os.path.isfile(src):
                        shutil.copy2(src, dst)
            
            # Clean up temp directory
            shutil.rmtree(temp_dir)
            
            logger.info(f"Restored data from backup at {backup_path}")
            return True
            
        except Exception as e:
            logger.error(f"Error restoring data from backup: {e}")
            return False