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
BUILD_PACKAGES=true
USE_LAUNCH=true # Flag to use the launch file instead of starting components manually

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

# Get AIRLAB_PATH from environment
if [ -z "${AIRLAB_PATH}" ]; then
    log_error "AIRLAB_PATH environment variable is not set. Please set it before running this script."
    log_info "Example: export AIRLAB_PATH=/path/to/airlab"
    exit 1
fi

DOCKER_COMPOSE_FILE="${AIRLAB_PATH}/dtc-dockerfiles/container-examples/docker-compose-x86.yaml"

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
    echo "  -s, --skip-build       Skip building the ROS packages"
    echo "  -m, --manual           Start components manually instead of using the launch file"
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
        -s|--skip-build)
            BUILD_PACKAGES=false
            shift
            ;;
        -m|--manual)
            USE_LAUNCH=false
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
    log_info "Container $CONTAINER_NAME is not running. Starting it with docker compose..."
    
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

# Find correct workspace path inside container
CONTAINER_AIRLAB_PATH=$(docker exec $CONTAINER_NAME bash -c "echo \${AIRLAB_PATH}")
CONTAINER_WORKSPACE_DIR="${CONTAINER_AIRLAB_PATH}/$(basename ${WORKSPACE_DIR})"

# Determine paths inside the container
CONTAINER_TOOL_DIR="${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_tool"
CONTAINER_DATA_DIR="${CONTAINER_TOOL_DIR}/data"

log_info "Container workspace directory: ${CONTAINER_WORKSPACE_DIR}"
log_info "Container tool directory: ${CONTAINER_TOOL_DIR}"

# Check if params file exists
if [ ! -f "${PARAMS_FILE}" ]; then
    log_warning "Params file ${PARAMS_FILE} not found. Using default path inside container."
    CONTAINER_PARAMS_FILE="${CONTAINER_WORKSPACE_DIR}/src/usb_cam/config/flir.yaml"
else
    # Calculate the relative path from workspace to params file
    RELATIVE_PATH=$(realpath --relative-to="${WORKSPACE_DIR}" "${PARAMS_FILE}")
    CONTAINER_PARAMS_FILE="${CONTAINER_WORKSPACE_DIR}/${RELATIVE_PATH}"
fi

log_info "Using params file: ${CONTAINER_PARAMS_FILE}"

# Create data directory if it doesn't exist inside the container
log_info "Ensuring data directory exists: ${CONTAINER_DATA_DIR}"
docker exec $CONTAINER_NAME bash -c "mkdir -p ${CONTAINER_DATA_DIR}"

# Configure the Docker container for thermal calibration
log_info "Configuring the container for thermal calibration..."

# Install RQT dependencies if not already installed
log_info "Installing required RQT packages..."
docker exec -u 0 $CONTAINER_NAME bash -c "apt-get update && apt-get install -y \
    ros-humble-rqt \
    ros-humble-rqt-gui \
    ros-humble-rqt-gui-py"

# Install numpy 1.24.2 if not already installed
docker exec $CONTAINER_NAME bash -c "pip3 list | grep -q 'numpy.*1.24.2' || pip3 install numpy==1.24.2"

# Install additional Python packages needed for the thermal calibration tool
log_info "Installing required Python packages..."
docker exec $CONTAINER_NAME bash -c "pip3 install matplotlib scipy"

# Fix packaging and setuptools to handle common build errors
log_info "Setting up Python package environment to prevent common build errors..."
docker exec $CONTAINER_NAME bash -c "pip3 install setuptools==58.2.0 wheel==0.37.1 packaging==21.3"

# Fix setuptools tests_require issue in setup.py files
log_info "Checking for potential package build issues..."
if docker exec $CONTAINER_NAME bash -c "[ -f ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_tool/src/thermal_calibration_rqt/setup.py ]"; then
    log_info "Checking thermal_calibration_rqt setup.py file for potential issues..."
    # Check if setup.py contains tests_require
    if docker exec $CONTAINER_NAME bash -c "grep -q 'tests_require' ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_tool/src/thermal_calibration_rqt/setup.py"; then
        log_info "Detected 'tests_require' in setup.py, fixing compatibility issue..."
        # Create a backup
        docker exec $CONTAINER_NAME bash -c "cp ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_tool/src/thermal_calibration_rqt/setup.py ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_tool/src/thermal_calibration_rqt/setup.py.bak"
        # Replace tests_require with extras_require
        docker exec $CONTAINER_NAME bash -c "sed -i 's/tests_require=\[/extras_require={\"test\": \[/g' ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_tool/src/thermal_calibration_rqt/setup.py"
        docker exec $CONTAINER_NAME bash -c "sed -i 's/\],  # Add pytest and any other test dependencies here/\]},  # Add pytest and any other test dependencies here/g' ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_tool/src/thermal_calibration_rqt/setup.py"
        log_success "Fixed setup.py file."
    fi
fi

# Install tmux if not already installed - use docker exec with root user
log_info "Checking if tmux is installed in the container..."
if ! docker exec $CONTAINER_NAME bash -c "command -v tmux >/dev/null 2>&1"; then
    log_info "Installing tmux in the container..."
    docker exec -u 0 $CONTAINER_NAME bash -c "apt-get update && apt-get install -y tmux"
fi

# Ensure workspace directory exists
docker exec $CONTAINER_NAME bash -c "mkdir -p ${CONTAINER_WORKSPACE_DIR}/src"

# Ensure workspace setup files exist and are sourced properly
log_info "Checking workspace setup files..."
WORKSPACE_SETUP_CMD=""
if docker exec $CONTAINER_NAME bash -c "[ -f ${CONTAINER_WORKSPACE_DIR}/install/setup.bash ]"; then
    log_info "Workspace setup file found. Will source it in all ROS commands."
    WORKSPACE_SETUP_CMD=". ${CONTAINER_WORKSPACE_DIR}/install/setup.bash && "
else
    log_warning "Workspace setup file not found at ${CONTAINER_WORKSPACE_DIR}/install/setup.bash"
    log_warning "Only ROS core packages will be available. Building workspace first..."
    
    # Try to build the workspace to create the setup files
    docker exec $CONTAINER_NAME bash -c "cd ${CONTAINER_WORKSPACE_DIR} && . /opt/ros/humble/setup.bash && colcon build --symlink-install"
    
    # Check again if setup file exists now
    if docker exec $CONTAINER_NAME bash -c "[ -f ${CONTAINER_WORKSPACE_DIR}/install/setup.bash ]"; then
        log_success "Successfully built workspace and created setup files."
        WORKSPACE_SETUP_CMD=". ${CONTAINER_WORKSPACE_DIR}/install/setup.bash && "
    else
        log_warning "Could not create workspace setup files. Some packages may not be available."
    fi
fi

# Build the thermal calibration packages if needed and if they exist
if [ "$BUILD_PACKAGES" = true ]; then
    # Check if thermal calibration packages exist
    INTERFACES_EXISTS=$(docker exec $CONTAINER_NAME bash -c "[ -d ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_interfaces ] && echo 1 || echo 0")
    RQT_EXISTS=$(docker exec $CONTAINER_NAME bash -c "[ -d ${CONTAINER_WORKSPACE_DIR}/src/thermal_calibration_rqt ] && echo 1 || echo 0")
    
    if [ "$INTERFACES_EXISTS" = "1" ] || [ "$RQT_EXISTS" = "1" ]; then
        log_info "Building thermal calibration packages..."
        
        # Build interfaces package first if it exists
        if [ "$INTERFACES_EXISTS" = "1" ]; then
            log_info "Building thermal_calibration_interfaces package..."
            docker exec $CONTAINER_NAME bash -c "cd ${CONTAINER_WORKSPACE_DIR} && . /opt/ros/humble/setup.bash && ${WORKSPACE_SETUP_CMD} PYTHONPATH= colcon build --symlink-install --packages-select thermal_calibration_interfaces"
            
            if [ $? -ne 0 ]; then
                log_error "Failed to build thermal_calibration_interfaces package."
                
                # Attempt to fix by rebuilding with the current setup
                log_info "Attempting to fix build issues..."
                docker exec $CONTAINER_NAME bash -c "cd ${CONTAINER_WORKSPACE_DIR} && rm -rf build/thermal_calibration_interfaces install/thermal_calibration_interfaces log/latest_build/thermal_calibration_interfaces"
                docker exec $CONTAINER_NAME bash -c "cd ${CONTAINER_WORKSPACE_DIR} && . /opt/ros/humble/setup.bash && ${WORKSPACE_SETUP_CMD} PYTHONPATH= colcon build --symlink-install --packages-select thermal_calibration_interfaces"
            else
                log_success "Successfully built thermal_calibration_interfaces package."
                # Update workspace setup command to include newly built packages
                WORKSPACE_SETUP_CMD=". ${CONTAINER_WORKSPACE_DIR}/install/setup.bash && "
            fi
        else
            log_warning "thermal_calibration_interfaces package not found in workspace."
        fi
        
        # Build RQT plugin package if it exists
        if [ "$RQT_EXISTS" = "1" ]; then
            log_info "Building thermal_calibration_rqt package..."
            docker exec $CONTAINER_NAME bash -c "cd ${CONTAINER_WORKSPACE_DIR} && . /opt/ros/humble/setup.bash && ${WORKSPACE_SETUP_CMD} PYTHONPATH= colcon build --symlink-install --packages-select thermal_calibration_rqt"
            
            if [ $? -ne 0 ]; then
                log_error "Failed to build thermal_calibration_rqt package."
                
                # Attempt to fix by rebuilding with the current setup
                log_info "Attempting to fix build issues..."
                docker exec $CONTAINER_NAME bash -c "cd ${CONTAINER_WORKSPACE_DIR} && rm -rf build/thermal_calibration_rqt install/thermal_calibration_rqt log/latest_build/thermal_calibration_rqt"
                docker exec $CONTAINER_NAME bash -c "cd ${CONTAINER_WORKSPACE_DIR} && . /opt/ros/humble/setup.bash && ${WORKSPACE_SETUP_CMD} PYTHONPATH= colcon build --symlink-install --packages-select thermal_calibration_rqt"
            else
                log_success "Successfully built thermal_calibration_rqt package."
                # Update workspace setup command to include newly built packages
                WORKSPACE_SETUP_CMD=". ${CONTAINER_WORKSPACE_DIR}/install/setup.bash && "
            fi
        else
            log_warning "thermal_calibration_rqt package not found in workspace."
        fi
        
        # Check if the packages were successfully built
        log_info "Checking if packages were successfully built..."
        if docker exec $CONTAINER_NAME bash -c "${ROS_SOURCE_CMD} ros2 pkg list | grep -q thermal_calibration_rqt"; then
            log_success "thermal_calibration_rqt package is available in ROS environment."
        else
            log_warning "thermal_calibration_rqt package is NOT available in ROS environment."
        fi
        
        if docker exec $CONTAINER_NAME bash -c "${ROS_SOURCE_CMD} ros2 pkg list | grep -q thermal_calibration_interfaces"; then
            log_success "thermal_calibration_interfaces package is available in ROS environment."
        else
            log_warning "thermal_calibration_interfaces package is NOT available in ROS environment."
        fi
    else
        log_warning "No thermal calibration packages found in the workspace."
    fi
else
    log_info "Skipping package build as requested."
fi

# Create the tmux session for running multiple commands
log_info "Setting up tmux session for thermal calibration..."
docker exec $CONTAINER_NAME bash -c "tmux new-session -d -s thermal_calibration || true"

# Define ROS source command that ensures both ROS and workspace are sourced
ROS_SOURCE_CMD=". /opt/ros/humble/setup.bash && ${WORKSPACE_SETUP_CMD}"

# Start usb_cam node if not skipped
if [ "$SKIP_CAM" = false ]; then
    log_info "Starting usb_cam node..."
    docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:0 '${ROS_SOURCE_CMD} ros2 run usb_cam usb_cam_node_exe --ros-args --params-file ${CONTAINER_PARAMS_FILE} || echo \"Failed to start usb_cam node. Check if the package is installed.\"' C-m"
    
    # Wait for usb_cam to start
    sleep 5
    
    # Check if usb_cam node is running
    if ! docker exec $CONTAINER_NAME bash -c "${ROS_SOURCE_CMD} ros2 node list 2>/dev/null | grep -q '/usb_cam'"; then
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
docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:1 '${ROS_SOURCE_CMD} ros2 run mono16_converter mono16_converter || echo \"Failed to start mono16_converter. Check if the package is installed.\"' C-m"

# Wait for mono16_converter to start
sleep 3

# Start thermal calibration
log_info "Starting thermal calibration system..."

if [ "$USE_LAUNCH" = true ]; then
    # Start using launch file
    docker exec $CONTAINER_NAME bash -c "tmux new-window -t thermal_calibration:2 -n thermal_calibration"
    docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:2 '${ROS_SOURCE_CMD} ros2 launch thermal_calibration_rqt thermal_calibration.launch.py' C-m"
else
    # Start thermal calibration node
    docker exec $CONTAINER_NAME bash -c "tmux new-window -t thermal_calibration:2 -n calibration_node"
    docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:2 '${ROS_SOURCE_CMD} ros2 run thermal_calibration_rqt thermal_calibration_node' C-m"
    
    # Start the thermal calibration rqt plugin
    docker exec $CONTAINER_NAME bash -c "tmux new-window -t thermal_calibration:3 -n rqt"
    
    # Check if thermal_calibration_rqt is installed
    if docker exec $CONTAINER_NAME bash -c "[ -d ${CONTAINER_WORKSPACE_DIR}/install/thermal_calibration_rqt ]"; then
        log_info "Starting thermal calibration rqt plugin..."
        PERSPECTIVE_PATH="${CONTAINER_WORKSPACE_DIR}/install/thermal_calibration_rqt/share/thermal_calibration_rqt/resource/thermal_calibration.perspective"
        # Check if perspective file exists
        if docker exec $CONTAINER_NAME bash -c "[ -f ${PERSPECTIVE_PATH} ]"; then
            docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:3 '${ROS_SOURCE_CMD} rqt --force-discover --perspective-file ${PERSPECTIVE_PATH}' C-m"
        else
            log_warning "Perspective file not found. Starting rqt without perspective file."
            docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:3 '${ROS_SOURCE_CMD} rqt --force-discover' C-m"
        fi
    else
        log_info "Starting standard rqt (thermal calibration plugin not installed)..."
        docker exec $CONTAINER_NAME bash -c "tmux send-keys -t thermal_calibration:3 '${ROS_SOURCE_CMD} rqt' C-m"
    fi
fi

# Attach to the tmux session
log_info "Attaching to tmux session..."
docker exec -it $CONTAINER_NAME bash -c "tmux attach-session -t thermal_calibration"

log_success "Thermal calibration tool closed."
log_info "To reattach to the session later, run: docker exec -it $CONTAINER_NAME bash -c \"tmux attach-session -t thermal_calibration\""
log_info "The container is still running. To stop it, run: cd $(dirname \"${DOCKER_COMPOSE_FILE}\") && docker compose -f $(basename \"${DOCKER_COMPOSE_FILE}\") down"