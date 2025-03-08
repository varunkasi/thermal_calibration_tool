# Thermal Camera Calibration Tool

A GUI application for radiometric calibration of a FLIR Boson+ thermal camera using reference measurements from a thermal gun.

## Project Overview

This tool provides a user-friendly interface for calibrating a FLIR Boson+ thermal camera, allowing you to map raw thermal camera values to actual temperature measurements. The calibration process involves:

1. Viewing the live thermal camera feed
2. Selecting pixels of interest in the thermal image
3. Entering reference temperature measurements from a thermal gun
4. Running regression analysis to create a calibration mapping function
5. Exporting the calibration model for future use
6. Using the calibrated camera in radiometric mode for temperature visualization

## Features

- Live thermal camera stream with interactive pixel selection
- Support for multiple color palettes (Inferno, Jet, Viridis, Grayscale, IronBow)
- Zooming and panning capabilities for detailed examination
- Collection and management of calibration points
- Multiple regression models (polynomial, exponential, logarithmic)
- Quality indicators and recommendations for calibration improvement
- Export and import of calibration data
- Radiometric mode for temperature visualization
- Containerized deployment using Docker

## Installation

### System Requirements

- Ubuntu 22.04
- Docker
- X11 server for GUI display
- USB access for camera connection

### Installation Steps

1. Clone the repository:
   ```
   git clone https://github.com/varunkasi/thermal-calibration-tool.git
   cd thermal-calibration-tool
   ```

2. Ensure you have the proper permissions for USB devices. Create a udev rule for the FLIR camera:
   ```
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="09cb", ATTRS{idProduct}=="*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-flir-boson.rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

3. Run the application using the provided script:
   ```
   chmod +x scripts/run_thermal_calibration.sh
   ./scripts/run_thermal_calibration.sh
   ```

## Usage

### Starting the Application

Run the application using the provided shell script:
```
./scripts/run_thermal_calibration.sh
```

This will:
- Build the Docker container if necessary
- Set up device permissions
- Launch the application
- Gracefully shut down when closed

### Calibration Workflow

1. Connect your FLIR Boson+ thermal camera
2. Point the camera at a target with varying temperatures
3. Click on a pixel in the thermal image
4. Click "Enter temperature value"
5. Input the temperature reading from your thermal gun
6. Repeat steps 3-5 for multiple points with different temperatures
7. Once you have at least 3 points, click "Calibrate"
8. Review the calibration quality and add more points if needed
9. Export the calibration for future use

### Development

For development purposes, use the development script:
```
./scripts/dev.sh
```

Additional options:
- `--run`: Run the application in development mode
- `--test`: Run unit tests
- `--format`: Format code according to style guidelines
- `--lint`: Lint the code
- `--docs`: Build documentation
- `--cmd "command"`: Run a custom command in the development container

## Project Structure

```
thermal_calibration_tool/
├── docker/                 # Docker configuration files
├── scripts/                # Shell scripts for running and development
├── src/                    # Source code
│   ├── camera/             # Camera interface modules
│   ├── calibration/        # Calibration algorithms and models
│   ├── gui/                # GUI components
│   ├── processing/         # Image processing utilities
│   ├── utils/              # Utility modules
│   └── main.py             # Application entry point
├── tests/                  # Unit tests
├── .gitignore
├── README.md
└── setup.py                # Python package configuration
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

- FLIR Systems for the Boson+ thermal camera
- PyQt5 team for the GUI framework
- NumPy, SciPy, and other scientific computing libraries