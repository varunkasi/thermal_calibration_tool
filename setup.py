#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Setup script for the Thermal Camera Calibration Tool.

This script is used to install the Thermal Camera Calibration Tool package.
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="thermal_calibration_tool",
    version="1.0.0",
    author="Thermal Tools Inc.",
    author_email="info@example.com",
    description="A GUI application for radiometric calibration of a FLIR Boson+ thermal camera",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/thermal-tools/thermal-calibration-tool",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Topic :: Scientific/Engineering :: Image Processing",
        "Topic :: Scientific/Engineering :: Visualization",
    ],
    python_requires=">=3.10",
    install_requires=[
        "numpy==1.24.2",
        "scipy>=1.8.0",
        "matplotlib>=3.5.0",
        "opencv-python>=4.5.5",
        "PyQt5>=5.15.6",
        "pyqtgraph>=0.12.4",
        "pyflir>=0.3.0",  # Replace with actual FLIR SDK Python package if different
        "pyyaml>=6.0",
        "loguru>=0.6.0",
        "typing-extensions>=4.1.0",
        "dataclasses-json>=0.5.7",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=3.0.0",
            "black>=22.1.0",
            "mypy>=0.931",
            "flake8>=4.0.1",
            "isort>=5.10.1",
            "pylint>=2.12.2",
            "ipython>=8.0.0",
            "ipdb>=0.13.9",
            "sphinx>=4.4.0",
            "sphinx-rtd-theme>=1.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "thermal-calibration=src.main:main",
        ],
    },
)