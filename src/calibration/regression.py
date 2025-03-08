#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Regression analysis for thermal camera calibration.

This module implements various regression models to create a mapping function 
between raw thermal camera values and temperature values.
"""

from typing import Dict, Any, Tuple, Callable, Optional, List

import numpy as np
from scipy import optimize
from loguru import logger


def create_calibration_function(
    raw_values: np.ndarray, 
    reference_temps: np.ndarray, 
    model_type: str = "polynomial", 
    degree: int = 2
) -> Tuple[Dict[str, Any], Callable[[Any], Any]]:
    """
    Create a calibration function mapping raw values to temperatures.
    
    Args:
        raw_values (np.ndarray): Array of raw thermal camera values.
        reference_temps (np.ndarray): Array of reference temperature values.
        model_type (str, optional): Type of regression model. 
            Options: "polynomial", "exponential", "logarithmic". Defaults to "polynomial".
        degree (int, optional): Degree of polynomial if model_type is "polynomial". 
            Defaults to 2.
    
    Returns:
        Tuple[Dict[str, Any], Callable[[Any], Any]]: 
            - Dictionary of model parameters
            - Calibration function that converts raw values to temperatures
    """
    if len(raw_values) != len(reference_temps):
        raise ValueError("Input arrays must have the same length")
    
    if len(raw_values) < 3:
        raise ValueError("Need at least 3 calibration points")
    
    # Normalize raw values to range [0, 1] for numerical stability
    min_raw = np.min(raw_values)
    max_raw = np.max(raw_values)
    range_raw = max_raw - min_raw
    
    if range_raw == 0:
        raise ValueError("All raw values are identical")
    
    raw_normalized = (raw_values - min_raw) / range_raw
    
    # Apply selected regression model
    if model_type == "polynomial":
        return _create_polynomial_calibration(
            raw_normalized, reference_temps, min_raw, range_raw, degree
        )
    elif model_type == "exponential":
        return _create_exponential_calibration(
            raw_normalized, reference_temps, min_raw, range_raw
        )
    elif model_type == "logarithmic":
        return _create_logarithmic_calibration(
            raw_normalized, reference_temps, min_raw, range_raw
        )
    else:
        logger.warning(f"Unknown model type: {model_type}, using polynomial")
        return _create_polynomial_calibration(
            raw_normalized, reference_temps, min_raw, range_raw, degree
        )


def _create_polynomial_calibration(
    raw_normalized: np.ndarray, 
    reference_temps: np.ndarray, 
    min_raw: float, 
    range_raw: float, 
    degree: int
) -> Tuple[Dict[str, Any], Callable[[Any], Any]]:
    """
    Create a polynomial calibration function.
    
    Args:
        raw_normalized (np.ndarray): Normalized raw values.
        reference_temps (np.ndarray): Reference temperature values.
        min_raw (float): Minimum raw value for normalization.
        range_raw (float): Range of raw values for normalization.
        degree (int): Degree of polynomial.
    
    Returns:
        Tuple[Dict[str, Any], Callable[[Any], Any]]: Model parameters and calibration function.
    """
    # Fit polynomial of specified degree
    coeffs = np.polyfit(raw_normalized, reference_temps, degree)
    
    # Convert to list for JSON serialization (in reverse order to match powers)
    coeffs_list = coeffs[::-1].tolist()
    
    # Create model parameters dictionary
    model_params = {
        "coefficients": coeffs_list,
        "degree": degree,
        "min_raw": min_raw,
        "range_raw": range_raw
    }
    
    # Create calibration function
    def calibration_func(raw_value):
        # Normalize input
        x_norm = (raw_value - min_raw) / range_raw
        
        # Handle array input
        if isinstance(raw_value, np.ndarray):
            result = np.zeros_like(raw_value, dtype=float)
            x_norm_clipped = np.clip(x_norm, 0, 1)  # Clip to [0, 1] to avoid extrapolation errors
            for i in range(degree + 1):
                result += coeffs_list[i] * (x_norm_clipped ** i)
            return result
        else:
            # Handle scalar input
            x_norm_clipped = max(0, min(1, x_norm))  # Clip to [0, 1]
            return sum(coeffs_list[i] * (x_norm_clipped ** i) for i in range(degree + 1))
    
    return model_params, calibration_func


def _create_exponential_calibration(
    raw_normalized: np.ndarray, 
    reference_temps: np.ndarray, 
    min_raw: float, 
    range_raw: float
) -> Tuple[Dict[str, Any], Callable[[Any], Any]]:
    """
    Create an exponential calibration function: temp = a * exp(b * raw) + c
    
    Args:
        raw_normalized (np.ndarray): Normalized raw values.
        reference_temps (np.ndarray): Reference temperature values.
        min_raw (float): Minimum raw value for normalization.
        range_raw (float): Range of raw values for normalization.
    
    Returns:
        Tuple[Dict[str, Any], Callable[[Any], Any]]: Model parameters and calibration function.
    """
    # Define the exponential function to fit
    def exp_func(x, a, b, c):
        return a * np.exp(b * x) + c
    
    # Initial parameter guess
    p0 = [1.0, 1.0, min(reference_temps)]
    
    # Fit the exponential function
    try:
        params, _ = optimize.curve_fit(exp_func, raw_normalized, reference_temps, p0=p0)
        a, b, c = params
    except RuntimeError as e:
        logger.warning(f"Exponential fit failed: {e}. Using fallback parameters.")
        # Fallback to simple linear approximation
        a, b, c = 0, 0, 0
        linear_coeffs = np.polyfit(raw_normalized, reference_temps, 1)
        a = linear_coeffs[0]
        c = linear_coeffs[1]
    
    # Create model parameters dictionary
    model_params = {
        "a": float(a),
        "b": float(b),
        "c": float(c),
        "min_raw": min_raw,
        "range_raw": range_raw
    }
    
    # Create calibration function
    def calibration_func(raw_value):
        # Normalize input
        x_norm = (raw_value - min_raw) / range_raw
        
        # Handle array input
        if isinstance(raw_value, np.ndarray):
            x_norm_clipped = np.clip(x_norm, 0, 1)
            return a * np.exp(b * x_norm_clipped) + c
        else:
            # Handle scalar input
            x_norm_clipped = max(0, min(1, x_norm))
            return a * np.exp(b * x_norm_clipped) + c
    
    return model_params, calibration_func


def _create_logarithmic_calibration(
    raw_normalized: np.ndarray, 
    reference_temps: np.ndarray, 
    min_raw: float, 
    range_raw: float
) -> Tuple[Dict[str, Any], Callable[[Any], Any]]:
    """
    Create a logarithmic calibration function: temp = a * log(b * raw + 1) + c
    
    Args:
        raw_normalized (np.ndarray): Normalized raw values.
        reference_temps (np.ndarray): Reference temperature values.
        min_raw (float): Minimum raw value for normalization.
        range_raw (float): Range of raw values for normalization.
    
    Returns:
        Tuple[Dict[str, Any], Callable[[Any], Any]]: Model parameters and calibration function.
    """
    # Define the logarithmic function to fit
    def log_func(x, a, b, c):
        return a * np.log(b * x + 1) + c
    
    # Initial parameter guess
    p0 = [1.0, 1.0, min(reference_temps)]
    
    # Fit the logarithmic function
    try:
        params, _ = optimize.curve_fit(log_func, raw_normalized, reference_temps, p0=p0)
        a, b, c = params
    except RuntimeError as e:
        logger.warning(f"Logarithmic fit failed: {e}. Using fallback parameters.")
        # Fallback to simple linear approximation
        a, b, c = 0, 0, 0
        linear_coeffs = np.polyfit(raw_normalized, reference_temps, 1)
        a = linear_coeffs[0]
        c = linear_coeffs[1]
        b = 1.0
    
    # Create model parameters dictionary
    model_params = {
        "a": float(a),
        "b": float(b),
        "c": float(c),
        "min_raw": min_raw,
        "range_raw": range_raw
    }
    
    # Create calibration function
    def calibration_func(raw_value):
        # Normalize input
        x_norm = (raw_value - min_raw) / range_raw
        
        # Handle array input
        if isinstance(raw_value, np.ndarray):
            x_norm_clipped = np.clip(x_norm, 0, 1)
            return a * np.log(b * x_norm_clipped + 1) + c
        else:
            # Handle scalar input
            x_norm_clipped = max(0, min(1, x_norm))
            return a * np.log(b * x_norm_clipped + 1) + c
    
    return model_params, calibration_func


def evaluate_calibration(
    raw_values: np.ndarray, 
    reference_temps: np.ndarray, 
    calibration_func: Callable[[Any], Any]
) -> Dict[str, float]:
    """
    Evaluate the quality of a calibration model.
    
    Args:
        raw_values (np.ndarray): Raw thermal camera values.
        reference_temps (np.ndarray): Reference temperature values.
        calibration_func (Callable[[Any], Any]): Calibration function to evaluate.
    
    Returns:
        Dict[str, float]: Dictionary of evaluation metrics.
    """
    # Predict temperatures using the calibration function
    predicted_temps = calibration_func(raw_values)
    
    # Calculate residuals
    residuals = reference_temps - predicted_temps
    
    # Calculate statistics
    mean_error = np.mean(residuals)
    mean_abs_error = np.mean(np.abs(residuals))
    rmse = np.sqrt(np.mean(residuals**2))
    
    # Calculate R-squared
    ss_total = np.sum((reference_temps - np.mean(reference_temps))**2)
    ss_residual = np.sum(residuals**2)
    r_squared = 1 - (ss_residual / ss_total) if ss_total != 0 else 0
    
    # Calculate max error and its location
    max_error_idx = np.argmax(np.abs(residuals))
    max_error = residuals[max_error_idx]
    max_error_raw = raw_values[max_error_idx]
    max_error_temp = reference_temps[max_error_idx]
    
    # Return evaluation metrics
    return {
        "mean_error": float(mean_error),
        "mean_abs_error": float(mean_abs_error),
        "rmse": float(rmse),
        "r_squared": float(r_squared),
        "max_error": float(max_error),
        "max_error_raw": float(max_error_raw),
        "max_error_temp": float(max_error_temp)
    }


def suggest_model_improvements(
    raw_values: np.ndarray, 
    reference_temps: np.ndarray, 
    current_stats: Dict[str, float]
) -> List[str]:
    """
    Suggest improvements to the calibration model.
    
    Args:
        raw_values (np.ndarray): Raw thermal camera values.
        reference_temps (np.ndarray): Reference temperature values.
        current_stats (Dict[str, float]): Current evaluation metrics.
    
    Returns:
        List[str]: List of suggested improvements.
    """
    suggestions = []
    
    # Check if we have enough data points
    if len(raw_values) < 5:
        suggestions.append("Collect more calibration points for better accuracy")
    
    # Check if the range of temperatures is sufficient
    temp_range = np.max(reference_temps) - np.min(reference_temps)
    if temp_range < 30:
        suggestions.append(f"Current temperature range is only {temp_range:.1f}°C. "
                         f"Try to include points with wider temperature range")
    
    # Check the quality of the fit
    rmse = current_stats.get("rmse", float('inf'))
    r_squared = current_stats.get("r_squared", 0)
    
    if rmse > 2.0:
        suggestions.append(f"RMSE is high ({rmse:.2f}°C). Consider using a different model type "
                         f"or adding more calibration points")
    
    if r_squared < 0.95:
        suggestions.append(f"R-squared is low ({r_squared:.3f}). The model may not be capturing "
                         f"the relationship well. Try a higher degree polynomial or different model type")
    
    # Check for outliers
    z_scores = np.abs((reference_temps - np.mean(reference_temps)) / np.std(reference_temps))
    outliers = np.where(z_scores > 2.5)[0]
    
    if len(outliers) > 0:
        outlier_temps = reference_temps[outliers]
        suggestions.append(f"Possible outliers detected at temperatures: "
                         f"{', '.join([f'{t:.1f}°C' for t in outlier_temps])}")
    
    # Check distribution of calibration points
    sorted_idx = np.argsort(reference_temps)
    sorted_temps = reference_temps[sorted_idx]
    
    gaps = sorted_temps[1:] - sorted_temps[:-1]
    max_gap_idx = np.argmax(gaps)
    max_gap = gaps[max_gap_idx]
    
    if max_gap > 10 and len(raw_values) > 3:
        gap_temp_low = sorted_temps[max_gap_idx]
        gap_temp_high = sorted_temps[max_gap_idx + 1]
        suggestions.append(f"Large gap of {max_gap:.1f}°C between {gap_temp_low:.1f}°C and "
                         f"{gap_temp_high:.1f}°C. Consider adding points in this range")
    
    return suggestions


def test_multiple_models(
    raw_values: np.ndarray, 
    reference_temps: np.ndarray
) -> Dict[str, Dict[str, Any]]:
    """
    Test multiple regression models and compare their performance.
    
    Args:
        raw_values (np.ndarray): Raw thermal camera values.
        reference_temps (np.ndarray): Reference temperature values.
    
    Returns:
        Dict[str, Dict[str, Any]]: Dictionary of model results with statistics.
    """
    results = {}
    
    # Test polynomial models of different degrees
    for degree in [1, 2, 3, 4]:
        model_name = f"polynomial_degree_{degree}"
        try:
            model_params, calibration_func = create_calibration_function(
                raw_values, reference_temps, "polynomial", degree
            )
            stats = evaluate_calibration(raw_values, reference_temps, calibration_func)
            results[model_name] = {
                "model_params": model_params,
                "stats": stats,
                "calibration_func": calibration_func
            }
        except Exception as e:
            logger.warning(f"Failed to create {model_name} model: {e}")
    
    # Test exponential model
    try:
        model_params, calibration_func = create_calibration_function(
            raw_values, reference_temps, "exponential"
        )
        stats = evaluate_calibration(raw_values, reference_temps, calibration_func)
        results["exponential"] = {
            "model_params": model_params,
            "stats": stats,
            "calibration_func": calibration_func
        }
    except Exception as e:
        logger.warning(f"Failed to create exponential model: {e}")
    
    # Test logarithmic model
    try:
        model_params, calibration_func = create_calibration_function(
            raw_values, reference_temps, "logarithmic"
        )
        stats = evaluate_calibration(raw_values, reference_temps, calibration_func)
        results["logarithmic"] = {
            "model_params": model_params,
            "stats": stats,
            "calibration_func": calibration_func
        }
    except Exception as e:
        logger.warning(f"Failed to create logarithmic model: {e}")
    
    return results


def find_best_model(model_results: Dict[str, Dict[str, Any]]) -> Tuple[str, Dict[str, Any]]:
    """
    Find the best model based on RMSE and R-squared.
    
    Args:
        model_results (Dict[str, Dict[str, Any]]): Dictionary of model results.
    
    Returns:
        Tuple[str, Dict[str, Any]]: Name of the best model and its results.
    """
    if not model_results:
        raise ValueError("No model results provided")
    
    # Define a scoring function that combines RMSE and R-squared
    def score_model(stats):
        rmse = stats.get("rmse", float('inf'))
        r_squared = stats.get("r_squared", 0)
        # Lower RMSE is better, higher R-squared is better
        # Normalize RMSE to 0-1 range (assuming max RMSE of 10°C)
        rmse_score = max(0, 1 - rmse/10)
        # Combine with 70% weight on RMSE, 30% on R-squared
        return 0.7 * rmse_score + 0.3 * r_squared
    
    best_model_name = None
    best_score = -float('inf')
    
    for model_name, model_data in model_results.items():
        stats = model_data.get("stats", {})
        score = score_model(stats)
        
        if score > best_score:
            best_score = score
            best_model_name = model_name
    
    return best_model_name, model_results[best_model_name]