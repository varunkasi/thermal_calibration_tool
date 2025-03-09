#!/bin/bash
# Self-contained Thermal Camera Calibration Tool Runner Script
# This script handles Docker container management via docker-compose,
# usb_cam node, mono16_converter, and the thermal calibration rqt plugin

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

# Get AIRLAB_PATH from environment
if [ -z "${AIRLAB_PATH}" ]; then
    log_error "AIRLAB_PATH environment variable is not set. Please set it before running this script."
    log_info "Example: export AIRLAB_PATH=/path/to/airlab"
    exit 1
fi

DOCKER_COMPOSE_FILE="${AIRLAB_PATH}/dtc-dockerfiles/container-examples/docker-compose-x86.yaml"

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
    echo "  -c, --compose FILE     Specify custom docker-compose file (default: $DOCKER_COMPOSE_FILE)"
    echo "  -f, --force-new        Force creation of a new container even if one exists"
    echo ""
    exit 0
}

# Function to clean up resources
cleanup() {
    log_info "Container will continue running. To stop it, use:"
    log_info "docker compose -f ${DOCKER_COMPOSE_FILE} down"
    exit 0
}

# Parse command line arguments
SKIP_CAM=false
FORCE_NEW=false

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
        -c|--compose)
            DOCKER_COMPOSE_FILE="$2"
            shift 2
            ;;
        -f|--force-new)
            FORCE_NEW=true
            shift
            ;;
        *)
            log_error "Unknown option: $1"
            show_usage
            ;;
    esac
done

# Trap signals to ensure clean exit
trap cleanup SIGINT SIGTERM

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    log_error "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker &> /dev/null; then
    log_error "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker daemon is running
if ! docker info &> /dev/null; then
    log_error "Docker daemon is not running. Please start Docker first."
    exit 1
fi

# Check if docker-compose file exists
if [ ! -f "${DOCKER_COMPOSE_FILE}" ]; then
    log_error "Docker Compose file not found: ${DOCKER_COMPOSE_FILE}"
    exit 1
fi

# Check if the container is already running
if docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
    if [ "$FORCE_NEW" = true ]; then
        log_info "Force new container requested. Stopping existing container..."
        (cd "$(dirname "${DOCKER_COMPOSE_FILE}")" && docker compose -f "$(basename "${DOCKER_COMPOSE_FILE}")" down)
    else
        log_info "Container $CONTAINER_NAME is already running. Will use it."
    fi
fi

# Start container if it's not already running
if ! docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
    log_info "Container $CONTAINER_NAME is not running. Starting it with docker-compose..."
    
    # Get directory of docker-compose file
    COMPOSE_DIR="$(dirname "${DOCKER_COMPOSE_FILE}")"
    COMPOSE_FILENAME="$(basename "${DOCKER_COMPOSE_FILE}")"
    
    # Export necessary environment variables for docker-compose
    export USER_ID=$(id -u)
    export GROUP_ID=$(id -g)
    export USER_NAME=$(id -un)
    export GROUP_NAME=$(id -gn)
    export DEVICE_TYPE="x86"
    export AIRLAB_PATH="${AIRLAB_PATH}"
    
    # Start container using docker compose
    (cd "${COMPOSE_DIR}" && docker compose -f "${COMPOSE_FILENAME}" up -d)
    
    if [ $? -ne 0 ]; then
        log_error "Failed to start container. Check Docker Compose logs for details."
        exit 1
    fi
    
    # Wait a moment for the container to start
    sleep 5
    
    log_success "Container $CONTAINER_NAME is now running."
else
    log_info "Using existing container $CONTAINER_NAME."
fi

# Convert local paths to container paths
CONTAINER_WORKSPACE_DIR=$(docker exec $CONTAINER_NAME bash -c "cd ${WORKSPACE_DIR} 2>/dev/null || echo '/workspace'" | tr -d '\r')
CONTAINER_TOOL_DIR=$(docker exec $CONTAINER_NAME bash -c "cd ${TOOL_DIR} 2>/dev/null || echo '/workspace/src/thermal_calibration_tool'" | tr -d '\r')
CONTAINER_DATA_DIR=$(docker exec $CONTAINER_NAME bash -c "cd ${DATA_DIR} 2>/dev/null || echo '/workspace/src/thermal_calibration_tool/data'" | tr -d '\r')

# Check if params file exists
if [ ! -f "${PARAMS_FILE}" ]; then
    log_warning "Params file ${PARAMS_FILE} not found. Using default path inside container."
    CONTAINER_PARAMS_FILE="/workspace/src/usb_cam/config/flir.yaml"
else
    CONTAINER_PARAMS_FILE=$(docker exec $CONTAINER_NAME bash -c "cd $(dirname ${PARAMS_FILE}) 2>/dev/null && pwd")/$(basename ${PARAMS_FILE})
    CONTAINER_PARAMS_FILE=${CONTAINER_PARAMS_FILE:-"/workspace/src/usb_cam/config/flir.yaml"}
fi

# Create data directory if it doesn't exist inside the container
log_info "Ensuring data directory exists: ${CONTAINER_DATA_DIR}"
docker exec $CONTAINER_NAME bash -c "mkdir -p ${CONTAINER_DATA_DIR}"

# Configure the Docker container for thermal calibration
log_info "Configuring the container for thermal calibration..."

# Install numpy 1.24.2 if not already installed
docker exec $CONTAINER_NAME bash -c "pip3 list | grep -q 'numpy.*1.24.2' || pip3 install numpy==1.24.2"

# Install tmux if not already installed
docker exec $CONTAINER_NAME bash -c "command -v tmux >/dev/null 2>&1 || apt-get update && apt-get install -y tmux"

# Ensure workspace directory exists
docker exec $CONTAINER_NAME bash -c "mkdir -p ${CONTAINER_WORKSPACE_DIR}/src"

# Check if thermal calibration packages exist and build if they do
if docker exec $CONTAINER_NAME bash -c "[ -d ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_interfaces ] && [ -d ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_rqt ]"; then
    log_info "Building thermal calibration packages..."
    docker exec $CONTAINER_NAME bash -c "cd ${CONTAINER_WORKSPACE_DIR} && . /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select thermal_calibration_interfaces thermal_calibration_rqt"
else
    log_warning "Thermal calibration packages not found in the container. The rqt plugin will not be available."
fi

# Create the tmux session for running multiple commands
log_info "Setting up tmux session for thermal calibration..."
docker exec $CONTAINER_NAME bash -c "tmux new-session -d -s thermal_calibration || true"

# Start usb_cam node if not skipped
if [ "$SKIP_CAM" = false ]; then
    log_info "Starting usb_cam node..."
    docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:0 '. /opt/ros/humble/setup.bash && ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ${CONTAINER_PARAMS_FILE} || echo \"Failed to start usb_cam node. Check if the package is installed.\"' C-m"
    
    # Wait for usb_cam to start
    sleep 5
    
    # Check if usb_cam node is running
    if ! docker exec $CONTAINER_NAME bash -c "ros2 node list 2>/dev/null | grep -q '/usb_cam'"; then
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
docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:1 '. /opt/ros/humble/setup.bash && ros2 run mono16_converter mono16_converter || echo \"Failed to start mono16_converter. Check if the package is installed.\"' C-m"

# Wait for mono16_converter to start
sleep 3

# Start the thermal calibration rqt plugin if available
log_info "Starting rqt..."
docker exec $CONTAINER_NAME bash -c "tmux new-window -t thermal_calibration:2 -n thermal_calibration"

# Check if thermal_calibration_rqt is installed
if docker exec $CONTAINER_NAME bash -c "[ -d ${CONTAINER_WORKSPACE_DIR}/install/thermal_calibration_rqt ]"; then
    log_info "Starting thermal calibration rqt plugin..."
    docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:2 '. /opt/ros/humble/setup.bash && . ${CONTAINER_WORKSPACE_DIR}/install/setup.bash && rqt --force-discover --perspective-file ${CONTAINER_TOOL_DIR}/src/thermal_calibration_rqt/resource/thermal_calibration.perspective' C-m"
else
    log_info "Starting standard rqt (thermal calibration plugin not installed)..."
    docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:2 '. /opt/ros/humble/setup.bash && rqt' C-m"
fi

# Attach to the tmux session
log_info "Attaching to tmux session..."
docker exec -it $CONTAINER_NAME bash -c "tmux attach-session -t thermal_calibration"

log_success "Thermal calibration tool closed."
log_info "To reattach to the session later, run: docker exec -it $CONTAINER_NAME bash -c \"tmux attach-session -t thermal_calibration\""
log_info "The container is still running. To stop it, run: cd $(dirname \"${DOCKER_COMPOSE_FILE}\") && docker compose -f $(basename \"${DOCKER_COMPOSE_FILE}\") down"