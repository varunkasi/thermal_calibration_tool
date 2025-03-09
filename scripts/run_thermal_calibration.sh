#!/bin/bash
# Integrated Thermal Camera Calibration Tool Runner Script
# This script automates the process of starting the usb_cam node, 
# mono16_converter, and the thermal calibration rqt plugin

set -e  # Exit immediately if any command exits with non-zero status

# Define colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration variables
CONTAINER_NAME="x86_container"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TOOL_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"
WORKSPACE_DIR="$( cd "${TOOL_DIR}/../.." && pwd )"
PARAMS_FILE="${WORKSPACE_DIR}/src/usb_cam/config/flir.yaml"
DATA_DIR="${TOOL_DIR}/data"

# Function to display progress messages
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help             Show this help message and exit"
    echo "  -d, --data-dir DIR     Specify custom data directory (default: $DATA_DIR)"
    echo "  -p, --params-file FILE Specify custom usb_cam params file (default: $PARAMS_FILE)"
    echo "  -n, --no-cam           Skip starting the usb_cam node (assume it's already running)"
    echo ""
    exit 0
}

# Parse command line arguments
SKIP_CAM=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            ;;
        -d|--data-dir)
            DATA_DIR="$2"
            shift 2
            ;;
        -p|--params-file)
            PARAMS_FILE="$2"
            shift 2
            ;;
        -n|--no-cam)
            SKIP_CAM=true
            shift
            ;;
        *)
            log_error "Unknown option: $1"
            show_usage
            ;;
    esac
done

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    log_error "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if the container is running
if ! docker ps | grep -q $CONTAINER_NAME; then
    log_info "Container $CONTAINER_NAME is not running. Starting it..."
    
    # Run the container using the provided script
    if [ -f "${WORKSPACE_DIR}/run-compose.sh" ]; then
        (cd "${WORKSPACE_DIR}" && ./run-compose.sh)
    else
        log_error "run-compose.sh not found. Please start the container manually."
        exit 1
    fi
    
    # Wait a moment for the container to start
    sleep 5
    
    # Check again if the container is running
    if ! docker ps | grep -q $CONTAINER_NAME; then
        log_error "Failed to start the container. Please check the Docker logs."
        exit 1
    fi
    
    log_success "Container $CONTAINER_NAME is now running."
else
    log_info "Container $CONTAINER_NAME is already running."
fi

# Convert local paths to container paths
CONTAINER_WORKSPACE_DIR=$(docker exec $CONTAINER_NAME bash -c "cd ${WORKSPACE_DIR} && pwd" | tr -d '\r')
CONTAINER_TOOL_DIR=$(docker exec $CONTAINER_NAME bash -c "cd ${TOOL_DIR} && pwd" | tr -d '\r')
CONTAINER_DATA_DIR=$(docker exec $CONTAINER_NAME bash -c "cd ${DATA_DIR} 2>/dev/null || echo '/tmp/thermal_calibration_data'" | tr -d '\r')
CONTAINER_PARAMS_FILE=$(docker exec $CONTAINER_NAME bash -c "cd $(dirname ${PARAMS_FILE}) && pwd")/$(basename ${PARAMS_FILE})

# Create data directory if it doesn't exist inside the container
log_info "Ensuring data directory exists: ${CONTAINER_DATA_DIR}"
docker exec $CONTAINER_NAME bash -c "mkdir -p ${CONTAINER_DATA_DIR}"

# Configure the Docker container for thermal calibration
log_info "Configuring the container for thermal calibration..."

# Install numpy 1.24.2 if not already installed
docker exec $CONTAINER_NAME bash -c "pip3 list | grep -q 'numpy.*1.24.2' || pip3 install numpy==1.24.2"

# Build the thermal calibration package
log_info "Building thermal calibration package..."
docker exec $CONTAINER_NAME bash -c "cd ${CONTAINER_WORKSPACE_DIR} && . /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select thermal_calibration_interfaces thermal_calibration_rqt"

# Create the tmux session for running multiple commands
log_info "Setting up tmux session for thermal calibration..."
docker exec $CONTAINER_NAME bash -c "tmux new-session -d -s thermal_calibration || true"

# Start usb_cam node if not skipped
if [ "$SKIP_CAM" = false ]; then
    log_info "Starting usb_cam node..."
    docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:0 '. /opt/ros/humble/setup.bash && ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ${CONTAINER_PARAMS_FILE}' C-m"
    
    # Wait for usb_cam to start
    sleep 5
    
    # Check if usb_cam node is running
    if ! docker exec $CONTAINER_NAME bash -c "ros2 node list | grep -q '/usb_cam'"; then
        log_warning "usb_cam node may not have started properly. Continuing anyway..."
    else
        log_success "usb_cam node is running."
    fi
else
    log_info "Skipping usb_cam node startup as requested."
fi

# Start mono16_converter
log_info "Starting mono16_converter node..."
docker exec $CONTAINER_NAME bash -c "tmux new-window -t thermal_calibration:1 -n mono16_converter"
docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:1 '. /opt/ros/humble/setup.bash && ros2 run mono16_converter mono16_converter' C-m"

# Wait for mono16_converter to start
sleep 3

# Start the thermal calibration rqt plugin
log_info "Starting thermal calibration rqt plugin..."
docker exec $CONTAINER_NAME bash -c "tmux new-window -t thermal_calibration:2 -n thermal_calibration"
docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:2 '. /opt/ros/humble/setup.bash && . ${CONTAINER_WORKSPACE_DIR}/install/setup.bash && rqt --force-discover --perspective-file ${CONTAINER_TOOL_DIR}/src/thermal_calibration_rqt/resource/thermal_calibration.perspective' C-m"

# Attach to the tmux session
log_info "Attaching to tmux session..."
docker exec -it $CONTAINER_NAME bash -c "tmux attach-session -t thermal_calibration"

log_success "Thermal calibration tool closed."
log_info "To reattach to the session later, run: docker exec -it $CONTAINER_NAME bash -c \"tmux attach-session -t thermal_calibration\""
