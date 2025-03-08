#!/bin/bash

# Thermal Camera Calibration Tool Development Environment Setup
# This script sets up a development environment for the thermal calibration tool

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

# Parse command line arguments
MODE="shell"  # Default mode is interactive shell
COMMAND=""

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --run) MODE="run"; shift ;;
        --test) MODE="test"; shift ;;
        --format) MODE="format"; shift ;;
        --lint) MODE="lint"; shift ;;
        --docs) MODE="docs"; shift ;;
        --cmd) MODE="custom"; COMMAND="$2"; shift 2 ;;
        *) echo_error "Unknown parameter: $1"; exit 1 ;;
    esac
done

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo_error "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if X server is running (needed for GUI)
if [ -z "$DISPLAY" ]; then
    echo_warning "X server not detected. GUI may not work properly."
fi

# Directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Navigate to project directory
cd "$PROJECT_DIR"

# Build/update the development Docker image
echo_info "Building development Docker image..."
docker-compose -f "$PROJECT_DIR/docker/docker-compose.dev.yml" build

# Run container based on mode
case $MODE in
    shell)
        echo_info "Starting development shell..."
        docker-compose -f "$PROJECT_DIR/docker/docker-compose.dev.yml" run --rm thermal-calibration-dev bash
        ;;
    run)
        echo_info "Running application in development mode..."
        docker-compose -f "$PROJECT_DIR/docker/docker-compose.dev.yml" run --rm thermal-calibration-dev python -m src.main
        ;;
    test)
        echo_info "Running tests..."
        docker-compose -f "$PROJECT_DIR/docker/docker-compose.dev.yml" run --rm thermal-calibration-dev pytest
        ;;
    format)
        echo_info "Formatting code..."
        docker-compose -f "$PROJECT_DIR/docker/docker-compose.dev.yml" run --rm thermal-calibration-dev bash -c "black src tests && isort src tests"
        ;;
    lint)
        echo_info "Linting code..."
        docker-compose -f "$PROJECT_DIR/docker/docker-compose.dev.yml" run --rm thermal-calibration-dev bash -c "flake8 src tests && mypy src tests && pylint src tests"
        ;;
    docs)
        echo_info "Building documentation..."
        docker-compose -f "$PROJECT_DIR/docker/docker-compose.dev.yml" run --rm thermal-calibration-dev bash -c "cd docs && sphinx-build -b html source build"
        ;;
    custom)
        echo_info "Running custom command: $COMMAND"
        docker-compose -f "$PROJECT_DIR/docker/docker-compose.dev.yml" run --rm thermal-calibration-dev bash -c "$COMMAND"
        ;;
esac

echo_success "Development task completed successfully!"