#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Image utility functions for the Thermal Camera Calibration Tool.

This module provides utility functions for image processing and manipulation,
such as filtering, smoothing, and format conversion.
"""

from typing import Optional, Tuple, Union

import numpy as np
from loguru import logger


def apply_gaussian_filter(image: np.ndarray, sigma: float = 1.0) -> np.ndarray:
    """
    Apply a Gaussian filter to an image for noise reduction.
    
    Args:
        image (np.ndarray): Input image.
        sigma (float, optional): Standard deviation of the Gaussian kernel. 
            Defaults to 1.0.
    
    Returns:
        np.ndarray: Filtered image.
    """
    try:
        from scipy.ndimage import gaussian_filter
        return gaussian_filter(image, sigma=sigma)
    except ImportError:
        logger.warning("scipy.ndimage not available. Using simple box filter instead.")
        return simple_box_filter(image, kernel_size=int(sigma * 3) | 1)


def simple_box_filter(image: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Apply a simple box filter to an image.
    
    Args:
        image (np.ndarray): Input image.
        kernel_size (int, optional): Size of the kernel. Must be odd. Defaults to 3.
    
    Returns:
        np.ndarray: Filtered image.
    """
    # Ensure kernel size is odd
    if kernel_size % 2 == 0:
        kernel_size += 1
    
    # Create a copy of the image to avoid modifying the original
    result = np.copy(image).astype(np.float32)
    
    # Simple box filter implementation
    half_size = kernel_size // 2
    for y in range(half_size, image.shape[0] - half_size):
        for x in range(half_size, image.shape[1] - half_size):
            # Extract neighborhood
            neighborhood = image[y-half_size:y+half_size+1, x-half_size:x+half_size+1]
            # Calculate mean
            result[y, x] = np.mean(neighborhood)
    
    return result.astype(image.dtype)


def apply_median_filter(image: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Apply a median filter to an image for noise reduction.
    
    Args:
        image (np.ndarray): Input image.
        kernel_size (int, optional): Size of the kernel. Must be odd. Defaults to 3.
    
    Returns:
        np.ndarray: Filtered image.
    """
    try:
        from scipy.ndimage import median_filter
        return median_filter(image, size=kernel_size)
    except ImportError:
        logger.warning("scipy.ndimage not available. Using simple median filter instead.")
        return simple_median_filter(image, kernel_size=kernel_size)


def simple_median_filter(image: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    """
    Apply a simple median filter to an image.
    
    Args:
        image (np.ndarray): Input image.
        kernel_size (int, optional): Size of the kernel. Must be odd. Defaults to 3.
    
    Returns:
        np.ndarray: Filtered image.
    """
    # Ensure kernel size is odd
    if kernel_size % 2 == 0:
        kernel_size += 1
    
    # Create a copy of the image to avoid modifying the original
    result = np.copy(image)
    
    # Simple median filter implementation
    half_size = kernel_size // 2
    for y in range(half_size, image.shape[0] - half_size):
        for x in range(half_size, image.shape[1] - half_size):
            # Extract neighborhood
            neighborhood = image[y-half_size:y+half_size+1, x-half_size:x+half_size+1]
            # Calculate median
            result[y, x] = np.median(neighborhood)
    
    return result


def scale_image(
    image: np.ndarray, 
    scale_factor: float
) -> np.ndarray:
    """
    Scale an image by a given factor.
    
    Args:
        image (np.ndarray): Input image.
        scale_factor (float): Scale factor. Values > 1 enlarge, values < 1 shrink.
    
    Returns:
        np.ndarray: Scaled image.
    """
    try:
        # Use OpenCV if available
        import cv2
        new_height = int(image.shape[0] * scale_factor)
        new_width = int(image.shape[1] * scale_factor)
        
        # Determine interpolation method based on scale factor
        if scale_factor > 1.0:
            # Enlarging: use bicubic interpolation
            interpolation = cv2.INTER_CUBIC
        else:
            # Shrinking: use area interpolation
            interpolation = cv2.INTER_AREA
        
        return cv2.resize(image, (new_width, new_height), interpolation=interpolation)
    except ImportError:
        logger.warning("OpenCV (cv2) not available. Using simple scaling.")
        return simple_scale_image(image, scale_factor)


def simple_scale_image(
    image: np.ndarray, 
    scale_factor: float
) -> np.ndarray:
    """
    Scale an image by a given factor using a simple nearest-neighbor approach.
    
    Args:
        image (np.ndarray): Input image.
        scale_factor (float): Scale factor. Values > 1 enlarge, values < 1 shrink.
    
    Returns:
        np.ndarray: Scaled image.
    """
    # Determine new dimensions
    new_height = int(image.shape[0] * scale_factor)
    new_width = int(image.shape[1] * scale_factor)
    
    # Create output image
    if len(image.shape) == 3:
        # Color image (3D array)
        result = np.zeros((new_height, new_width, image.shape[2]), dtype=image.dtype)
    else:
        # Grayscale image (2D array)
        result = np.zeros((new_height, new_width), dtype=image.dtype)
    
    # Map each pixel in the result to the source image
    for y in range(new_height):
        src_y = min(int(y / scale_factor), image.shape[0] - 1)
        for x in range(new_width):
            src_x = min(int(x / scale_factor), image.shape[1] - 1)
            result[y, x] = image[src_y, src_x]
    
    return result


def apply_threshold(
    image: np.ndarray, 
    threshold: float, 
    max_value: Optional[float] = None
) -> np.ndarray:
    """
    Apply a threshold to an image.
    
    Args:
        image (np.ndarray): Input image.
        threshold (float): Threshold value.
        max_value (Optional[float], optional): Value to set for pixels above threshold.
            If None, pixels above threshold are set to their original value.
            Defaults to None.
    
    Returns:
        np.ndarray: Thresholded image.
    """
    # Create a copy of the image to avoid modifying the original
    result = np.copy(image)
    
    # Apply threshold
    if max_value is None:
        # Binary threshold
        result[result < threshold] = 0
    else:
        # Binary threshold with max value
        result[result < threshold] = 0
        result[result >= threshold] = max_value
    
    return result


def normalize_image(
    image: np.ndarray, 
    min_val: Optional[float] = None, 
    max_val: Optional[float] = None,
    target_type: np.dtype = np.uint8
) -> np.ndarray:
    """
    Normalize an image to a specified range.
    
    Args:
        image (np.ndarray): Input image.
        min_val (Optional[float], optional): Minimum value in the source range.
            If None, uses the minimum value in the image. Defaults to None.
        max_val (Optional[float], optional): Maximum value in the source range.
            If None, uses the maximum value in the image. Defaults to None.
        target_type (np.dtype, optional): Target data type. Defaults to np.uint8 (0-255 range).
    
    Returns:
        np.ndarray: Normalized image.
    """
    # Determine source range
    if min_val is None:
        min_val = np.min(image)
    if max_val is None:
        max_val = np.max(image)
    
    # Check for valid range
    if min_val >= max_val:
        min_val = max_val - 1
    
    # Normalize to 0-1 range
    normalized = (image.astype(np.float32) - min_val) / (max_val - min_val)
    
    # Clip to ensure values are within range
    normalized = np.clip(normalized, 0.0, 1.0)
    
    # Convert to target type
    if target_type == np.uint8:
        return (normalized * 255).astype(np.uint8)
    elif target_type == np.uint16:
        return (normalized * 65535).astype(np.uint16)
    else:
        return normalized.astype(target_type)


def equalize_histogram(image: np.ndarray) -> np.ndarray:
    """
    Equalize the histogram of an image to enhance contrast.
    
    Args:
        image (np.ndarray): Input grayscale image.
    
    Returns:
        np.ndarray: Equalized image.
    """
    try:
        # Use OpenCV if available
        import cv2
        return cv2.equalizeHist(image.astype(np.uint8))
    except ImportError:
        logger.warning("OpenCV (cv2) not available. Using simple histogram equalization.")
        return simple_equalize_histogram(image)


def simple_equalize_histogram(image: np.ndarray) -> np.ndarray:
    """
    Simple implementation of histogram equalization.
    
    Args:
        image (np.ndarray): Input grayscale image.
    
    Returns:
        np.ndarray: Equalized image.
    """
    # Convert to uint8 if needed
    if image.dtype != np.uint8:
        # Normalize to 0-255 range
        image = normalize_image(image, target_type=np.uint8)
    
    # Compute histogram
    hist, _ = np.histogram(image.flatten(), 256, [0, 256])
    
    # Compute cumulative distribution function (CDF)
    cdf = hist.cumsum()
    
    # Normalize CDF to 0-255 range
    cdf_normalized = cdf * 255 / cdf[-1]
    
    # Use CDF as lookup table
    result = np.interp(image.flatten(), np.arange(256), cdf_normalized)
    
    # Reshape back to original image shape
    return result.reshape(image.shape).astype(np.uint8)


def create_pseudo_color_image(
    grayscale_image: np.ndarray,
    colormap_name: str = "inferno"
) -> np.ndarray:
    """
    Create a pseudo-color image from a grayscale image using a colormap.
    
    Args:
        grayscale_image (np.ndarray): Input grayscale image.
        colormap_name (str, optional): Name of the colormap to use.
            Options: "inferno", "jet", "viridis", "hot", "cool", "rainbow".
            Defaults to "inferno".
    
    Returns:
        np.ndarray: Pseudo-color image (RGB).
    """
    try:
        # Use Matplotlib if available
        import matplotlib.pyplot as plt
        from matplotlib import cm
        
        # Normalize input
        normalized = normalize_image(grayscale_image)
        
        # Apply colormap
        colormap = cm.get_cmap(colormap_name)
        colored = colormap(normalized)
        
        # Convert to uint8 RGB
        colored_rgb = (colored[:, :, :3] * 255).astype(np.uint8)
        return colored_rgb
    
    except ImportError:
        logger.warning("Matplotlib not available. Using simple colormap.")
        
        # Normalize input
        normalized = normalize_image(grayscale_image, target_type=np.uint8)
        
        # Create a simple RGB image using the grayscale value
        height, width = normalized.shape
        result = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Simple color mapping
        if colormap_name == "jet":
            # Simple jet colormap approximation
            for y in range(height):
                for x in range(width):
                    val = normalized[y, x]
                    # Red component
                    if val < 64:
                        r = 0
                    elif val < 128:
                        r = 4 * (val - 64)
                    elif val < 192:
                        r = 255
                    else:
                        r = 255 - 4 * (val - 192)
                    
                    # Green component
                    if val < 64:
                        g = 4 * val
                    elif val < 128:
                        g = 255
                    elif val < 192:
                        g = 255 - 4 * (val - 128)
                    else:
                        g = 0
                    
                    # Blue component
                    if val < 64:
                        b = 255
                    elif val < 128:
                        b = 255 - 4 * (val - 64)
                    elif val < 192:
                        b = 0
                    else:
                        b = 4 * (val - 192)
                    
                    result[y, x] = [r, g, b]
        
        elif colormap_name == "inferno":
            # Simple inferno colormap approximation
            for y in range(height):
                for x in range(width):
                    val = normalized[y, x]
                    # Black to purple to orange to yellow
                    if val < 64:  # Black to purple
                        r = val * 2
                        g = 0
                        b = val * 4
                    elif val < 128:  # Purple to deep red
                        r = 128 + (val - 64) * 2
                        g = 0
                        b = 255 - (val - 64) * 4
                    elif val < 192:  # Deep red to orange
                        r = 255
                        g = (val - 128) * 3
                        b = 0
                    else:  # Orange to yellow
                        r = 255
                        g = 192 + (val - 192) * 2
                        b = 0
                    
                    result[y, x] = [r, g, b]
        
        else:
            # Default to grayscale
            for y in range(height):
                for x in range(width):
                    val = normalized[y, x]
                    result[y, x] = [val, val, val]
        
        return result


def detect_hot_spots(
    thermal_image: np.ndarray,
    threshold_percentile: float = 95.0
) -> Tuple[np.ndarray, list]:
    """
    Detect hot spots in a thermal image.
    
    Args:
        thermal_image (np.ndarray): Input thermal image.
        threshold_percentile (float, optional): Percentile to use for thresholding.
            Defaults to 95.0.
    
    Returns:
        Tuple[np.ndarray, list]: Binary mask of hot spots and list of hot spot coordinates.
    """
    # Determine threshold value
    threshold = np.percentile(thermal_image, threshold_percentile)
    
    # Create binary mask
    hot_spots_mask = thermal_image > threshold
    
    # Find connected components
    try:
        from scipy import ndimage
        labeled_array, num_features = ndimage.label(hot_spots_mask)
        
        # Get coordinates of hot spots
        hot_spots = []
        for i in range(1, num_features + 1):
            # Get coordinates of the current hot spot
            y, x = np.where(labeled_array == i)
            
            # Calculate centroid
            if len(y) > 0 and len(x) > 0:
                centroid_y = int(np.mean(y))
                centroid_x = int(np.mean(x))
                area = len(y)
                max_temp = np.max(thermal_image[labeled_array == i])
                
                hot_spots.append({
                    'centroid': (centroid_x, centroid_y),
                    'area': area,
                    'max_temp': max_temp
                })
    
    except ImportError:
        logger.warning("scipy.ndimage not available. Hot spot detection limited.")
        # Simple hot spot detection
        hot_spots = []
        y, x = np.where(hot_spots_mask)
        if len(y) > 0 and len(x) > 0:
            centroid_y = int(np.mean(y))
            centroid_x = int(np.mean(x))
            area = len(y)
            max_temp = np.max(thermal_image[hot_spots_mask])
            
            hot_spots.append({
                'centroid': (centroid_x, centroid_y),
                'area': area,
                'max_temp': max_temp
            })
    
    return hot_spots_mask, hot_spots


def create_temperature_overlay(
    visible_image: np.ndarray,
    thermal_image: np.ndarray,
    alpha: float = 0.5
) -> np.ndarray:
    """
    Create an overlay of a thermal image on a visible image.
    
    Args:
        visible_image (np.ndarray): Visible light image (RGB).
        thermal_image (np.ndarray): Thermal image (grayscale).
        alpha (float, optional): Opacity of the thermal overlay. Defaults to 0.5.
    
    Returns:
        np.ndarray: Overlay image (RGB).
    """
    # Check if images have the same shape
    if visible_image.shape[:2] != thermal_image.shape:
        logger.warning("Visible and thermal images have different shapes. Resizing thermal image.")
        try:
            import cv2
            thermal_image = cv2.resize(thermal_image, (visible_image.shape[1], visible_image.shape[0]))
        except ImportError:
            thermal_image = simple_scale_image(thermal_image, 
                                        visible_image.shape[0] / thermal_image.shape[0])
    
    # Create a pseudo-color thermal image
    thermal_color = create_pseudo_color_image(thermal_image)
    
    # Blend images
    return blend_images(visible_image, thermal_color, alpha)


def blend_images(
    image1: np.ndarray,
    image2: np.ndarray,
    alpha: float = 0.5
) -> np.ndarray:
    """
    Blend two images with the specified alpha value.
    
    Args:
        image1 (np.ndarray): First image (RGB).
        image2 (np.ndarray): Second image (RGB).
        alpha (float, optional): Weight of the second image. Defaults to 0.5.
    
    Returns:
        np.ndarray: Blended image (RGB).
    """
    # Ensure alpha is in [0, 1]
    alpha = max(0.0, min(1.0, alpha))
    
    # Ensure both images are RGB (3 channels)
    if len(image1.shape) == 2:
        image1 = np.stack([image1] * 3, axis=2)
    if len(image2.shape) == 2:
        image2 = np.stack([image2] * 3, axis=2)
    
    # Ensure both images have the same data type
    dtype = np.result_type(image1.dtype, image2.dtype)
    image1 = image1.astype(dtype)
    image2 = image2.astype(dtype)
    
    # Blend images
    blended = (1 - alpha) * image1 + alpha * image2
    
    # Ensure result is in the valid range for the data type
    if dtype == np.uint8:
        blended = np.clip(blended, 0, 255).astype(np.uint8)
    elif dtype == np.uint16:
        blended = np.clip(blended, 0, 65535).astype(np.uint16)
    else:
        blended = np.clip(blended, 0.0, 1.0)
    
    return blended