#!/bin/bash

# Thermal Camera Calibration Tool Runner
# This script sets up and runs the thermal calibration tool in a Docker container

# Exit on error
set -e

# Print colorful messages
function echo_info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}

function echo_success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}

function echo_error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1"
}

function echo_warning() {
    echo -e "\033[1;33m[WARNING]\033[0m $1"
}

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo_error "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if X server is running
if [ -z "$DISPLAY" ]; then
    echo_error "X server not detected. Please make sure X server is running."
    exit 1
fi

# Directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Create data directory if it doesn't exist
DATA_DIR="${HOME}/thermal_calibration_data"
mkdir -p "$DATA_DIR"
echo_info "Data directory: $DATA_DIR"

# Setup udev rules for FLIR camera if needed
UDEV_RULE="/etc/udev/rules.d/99-flir-boson.rules"
if [ ! -f "$UDEV_RULE" ]; then
    echo_warning "FLIR camera udev rules not found. Setting up udev rules..."
    echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"09cb\", ATTRS{idProduct}==\"*\", MODE=\"0666\"" | sudo tee "$UDEV_RULE"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo_success "Udev rules set up for FLIR camera"
fi

# Function to clean up on exit
function cleanup {
    echo_info "Shutting down container..."
    docker-compose -f "$PROJECT_DIR/docker/docker-compose.yml" down
    echo_success "Container stopped. Exiting."
}

# Register the cleanup function to be called on exit
trap cleanup EXIT

# Navigate to project directory
cd "$PROJECT_DIR"

# Build/pull the Docker image
echo_info "Building Docker image..."
docker-compose -f "$PROJECT_DIR/docker/docker-compose.yml" build

# Run the container
echo_info "Starting thermal calibration tool..."
docker-compose -f "$PROJECT_DIR/docker/docker-compose.yml" up

# Script will exit here but cleanup function will be called due to the trap