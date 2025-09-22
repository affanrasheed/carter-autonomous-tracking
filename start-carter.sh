#!/bin/bash

# Carter Startup Script
# This script helps you start the carter services with various configurations

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -a, --arch ARCH      Set architecture (x86|arm64) [default: x86]"
    echo "  -p, --profile PROF   Set profile (all|ess-depth|nanoowl-detection|robot-path-planning) [default: all]"
    echo "  -d, --detached      Run in detached mode"
    echo "  -s, --stop          Stop all services"
    echo "  -l, --logs          Show logs"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                           # Start all services with x86 architecture"
    echo "  $0 --arch arm64              # Start all services with arm64 architecture"
    echo "  $0 --profile ess-depth       # Start only ESS depth estimation service"
    echo "  $0 --profile robot-path-planning  # Start only robot path planning service"
    echo "  $0 --stop                    # Stop all services"
    echo "  $0 --logs                    # Show logs for all services"
}

# Default values
ARCH="x86"
PROFILE="all"
DETACHED=false
STOP=false
LOGS=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -a|--arch)
            ARCH="$2"
            shift 2
            ;;
        -p|--profile)
            PROFILE="$2"
            shift 2
            ;;
        -d|--detached)
            DETACHED=true
            shift
            ;;
        -s|--stop)
            STOP=true
            shift
            ;;
        -l|--logs)
            LOGS=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Validate architecture
if [[ "$ARCH" != "x86" && "$ARCH" != "arm64" ]]; then
    print_error "Invalid architecture: $ARCH. Must be 'x86' or 'arm64'"
    exit 1
fi

# Validate profile
if [[ "$PROFILE" != "all" && "$PROFILE" != "ess-depth" && "$PROFILE" != "robot-path-planning" ]]; then
    print_error "Invalid profile: $PROFILE. Must be 'all', 'ess-depth' or 'robot-path-planning'"
    exit 1
fi

# Check if Docker is running
if ! docker info >/dev/null 2>&1; then
    print_error "Docker is not running. Please start Docker first."
    exit 1
fi

# Check if required directories exist
check_directories() {
    local missing_dirs=()
    
    if [[ ! -d "$HOME/ess_model" ]]; then
        missing_dirs+=("$HOME/ess_model")
    fi
    
    if [[ ${#missing_dirs[@]} -gt 0 ]]; then
        print_warning "The following required directories are missing:"
        for dir in "${missing_dirs[@]}"; do
            echo "  - $dir"
        done
        print_info "Creating missing directories..."
        for dir in "${missing_dirs[@]}"; do
            mkdir -p "$dir"
            print_success "Created: $dir"
        done
    fi
}

# Stop services
stop_services() {
    print_info "Stopping carter services..."
    docker compose down
    print_success "Services stopped"
}

# Show logs
show_logs() {
    print_info "Showing logs for carter services..."
    docker compose logs -f
}

# Start services
start_services() {
    print_info "Starting carter services..."
    print_info "Architecture: $ARCH"
    print_info "Profile: $PROFILE"
    
    # Check directories
    check_directories
    
    # Set environment variables
    export ARCH_TAG="$ARCH"
    
    # Build command
    local cmd="docker compose --env-file carter.env --profile $PROFILE up"
    if [[ "$DETACHED" == true ]]; then
        cmd="$cmd -d"
    fi
    
    print_info "Running: $cmd"
    eval "$cmd"
    
    if [[ "$DETACHED" == true ]]; then
        print_success "Services started in detached mode"
        print_info "Use '$0 --logs' to view logs"
        print_info "Use '$0 --stop' to stop services"
    fi
}

# Main execution
if [[ "$STOP" == true ]]; then
    stop_services
elif [[ "$LOGS" == true ]]; then
    show_logs
else
    start_services
fi
