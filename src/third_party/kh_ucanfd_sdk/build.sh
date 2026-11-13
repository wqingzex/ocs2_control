#!/bin/bash

set -e  # Exit on error

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# udev rule configuration
UDEV_RULE_PATH="/etc/udev/rules.d/100-kcan.rules"
UDEV_RULE_CONTENT='SUBSYSTEM=="net", ACTION=="add", DRIVERS=="kcan", NAME="kcan%n"'
RUNTIME_KERNEL_LOCATION=""
CLI_KERNEL_LOCATION=""

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1" >&2
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" >&2
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

KERNEL_HELPER="$SCRIPT_DIR/kernel_location.sh"
if [[ ! -f "$KERNEL_HELPER" ]]; then
    print_error "Kernel location helper not found: $KERNEL_HELPER"
    exit 1
fi
source "$KERNEL_HELPER"

sanitize_kernel_location_for_sudo_make() {
    local kernel_location="$1"
    local sanitized_location=""

    sanitized_location=$(printf '%s\n' "$kernel_location" | tr -d ';&|`$()')
    if [[ "$sanitized_location" != "$kernel_location" ]]; then
        print_error "Kernel path contains unsafe shell metacharacters: $kernel_location" >&2
        return 1
    fi

    echo "$sanitized_location"
}

resolve_runtime_kernel_location() {
    local kernel_version="${1:-$(uname -r)}"
    local required="${2:-1}"
    local kernel_location=""

    if kernel_location=$(kcan_find_kernel_location "$kernel_version" "$CLI_KERNEL_LOCATION"); then
        if ! kcan_require_prepared_kernel_tree "$kernel_location"; then
            exit 1
        fi
        echo "$kernel_location"
        return 0
    fi

    if [[ "$required" -eq 1 ]]; then
        kcan_print_kernel_location_guidance "$kernel_version"
        exit 1
    fi

    return 1
}

# Check DKMS installation
check_dkms() {
    print_info "Checking DKMS installation..."
    if ! command -v dkms &> /dev/null; then
        print_warning "DKMS is not installed, installing automatically..."
        if ! (sudo apt-get update; sudo apt-get install -y dkms); then
            print_error "Failed to install DKMS automatically"
            print_info "Please install DKMS manually: sudo apt-get install dkms"
            exit 1
        fi
        print_success "DKMS installed successfully"
    fi
    print_success "DKMS is installed: $(dkms --version 2>/dev/null | head -1)"
}

# Check sudo permissions
check_sudo() {
    print_info "Checking sudo permissions..."
    if ! sudo -v 2>/dev/null; then
        print_error "Unable to obtain sudo permissions, please check user permission settings"
        exit 1
    fi
    print_success "Sudo permissions check passed"
}

# Check dependencies
check_dependencies() {
    local cross_compile=$1
    local cc_cmd=${2:-gcc}
    print_info "Checking build dependencies..."
    
    # Check necessary commands
    local commands=("make" "sudo")
    if [[ $cross_compile -eq 1 ]]; then
        commands+=("$cc_cmd")  # Only need host gcc for compiling tools
    else
        commands+=("$cc_cmd")
    fi
    
    for cmd in "${commands[@]}"; do
        if ! command -v "$cmd" &> /dev/null; then
            print_error "Missing dependency: $cmd"
            print_info "Please install necessary build tools: sudo apt-get install build-essential"
            exit 1
        fi
    done
    
    # Check development packages
    local packages=()
    for pkg in "${packages[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            print_warning "Missing development package: $pkg"
            print_info "Please install: sudo apt-get install $pkg"
        fi
    done
    
    print_success "Dependency check completed"
}

# Detect kernel GCC version from compile.h (Kbuild style)
detect_kernel_gcc_version() {
    local kernel_version=$(uname -r)
    local headers_dir=""
    local compile_h=""
    local compiler_info=""
    local gcc_version=""

    headers_dir=$(kcan_find_kernel_location "$kernel_version" "$CLI_KERNEL_LOCATION" 2>/dev/null || true)
    if [[ -n "$headers_dir" ]]; then
        compile_h="$headers_dir/include/generated/compile.h"
    else
        compile_h="/lib/modules/$kernel_version/build/include/generated/compile.h"
    fi

    if [[ -f "$compile_h" ]]; then
        compiler_info=$(grep "LINUX_COMPILER" "$compile_h" 2>/dev/null | \
                       sed 's/.*"\(.*\)".*/\1/')

        # Try to match full version (e.g., "13.3.0")
        if [[ $compiler_info =~ ([0-9]+\.[0-9]+\.[0-9]+) ]]; then
            gcc_version="${BASH_REMATCH[1]}"
        # Fallback to major version (e.g., "13" from "gcc-13")
        elif [[ $compiler_info =~ gcc-([0-9]+) ]]; then
            gcc_version="${BASH_REMATCH[1]}"
        fi
    fi

    # Fallback to /proc/version if compile.h not available
    if [[ -z "$gcc_version" ]]; then
        local version_str=""
        if [[ -r /proc/version ]]; then
            version_str=$(cat /proc/version)
        else
            version_str=$(uname -v)
        fi

        if [[ $version_str =~ gcc[-[:space:]]*([0-9]+(\.[0-9]+(\.[0-9]+)?)?) ]]; then
            gcc_version="${BASH_REMATCH[1]}"
        fi
    fi

    echo "$gcc_version"
}

# Resolve host compiler for local build
resolve_host_cc() {
    local kernel_gcc=""
    local cc_cmd=""

    if [[ -n "${CC:-}" ]]; then
        cc_cmd="$CC"
        print_info "Using CC from environment: $cc_cmd"
        echo "$cc_cmd"
        return 0
    fi

    kernel_gcc=$(detect_kernel_gcc_version)
    if [[ -n "$kernel_gcc" ]]; then
        local kernel_gcc_major="${kernel_gcc%%.*}"

        if command -v "gcc-$kernel_gcc_major" &> /dev/null; then
            cc_cmd="gcc-$kernel_gcc_major"
        elif command -v "gcc-$kernel_gcc" &> /dev/null; then
            cc_cmd="gcc-$kernel_gcc"
        fi

        if [[ -n "$cc_cmd" ]]; then
            print_info "Kernel built with GCC $kernel_gcc; using $cc_cmd"
            echo "$cc_cmd"
            return 0
        fi

        print_warning "Kernel GCC $kernel_gcc detected, but matching gcc not found"
    else
        print_warning "Unable to detect kernel GCC version"
    fi

    echo "gcc"
}

# install udev rule
install_udev_rule() {
    print_info "Installing udev rule..."
    
    if [[ -f "$UDEV_RULE_PATH" ]]; then
        if grep -Fxq "$UDEV_RULE_CONTENT" "$UDEV_RULE_PATH"; then
            print_info "Udev rule already exists"
            return 0
        else
            sudo cp "$UDEV_RULE_PATH" "${UDEV_RULE_PATH}.backup.$(date +%Y%m%d%H%M%S)"
        fi
    fi
    
    echo "$UDEV_RULE_CONTENT" | sudo tee "$UDEV_RULE_PATH" > /dev/null
    
    if [[ $? -eq 0 ]]; then
        sudo chmod 644 "$UDEV_RULE_PATH"
        
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        print_success "Udev rule installed successfully"
    else
        print_error "Failed to install udev rule"
        exit 1
    fi
}

# uninstall udev rule
uninstall_udev_rule() {
    print_info "Checking and removing udev rule..."
    
    if [[ -f "$UDEV_RULE_PATH" ]]; then
        if grep -Fxq "$UDEV_RULE_CONTENT" "$UDEV_RULE_PATH"; then
            sudo rm -f "$UDEV_RULE_PATH"
            
            if [[ $? -eq 0 ]]; then
                sudo udevadm control --reload-rules
                sudo udevadm trigger
                print_success "Udev rule removed successfully"
            else
                print_error "Failed to remove udev rule"
                exit 1
            fi
        else
            print_info "Udev rule not found in $UDEV_RULE_PATH"
        fi
    else
        print_info "No udev rule file found"
    fi
}

# Cleanup function
cleanup() {
    if [[ $? -ne 0 ]]; then
        print_error "Error occurred during build process, cleaning up..."
        if [[ -n "$RUNTIME_KERNEL_LOCATION" ]]; then
            local kernel_path=""
            if ! kernel_path=$(sanitize_kernel_location_for_sudo_make "$RUNTIME_KERNEL_LOCATION"); then
                exit 1
            fi
            sudo make clean KERNEL_LOCATION="$kernel_path" > /dev/null 2>&1 || true
        else
            sudo make clean > /dev/null 2>&1 || true
        fi
    fi
}

# Trap cleanup function on exit
trap cleanup EXIT

# Cross-compilation configuration function
CROSS_COMPILE_CONFIG() {
    print_info "Configuring cross-compilation parameters..."
    
    # Check if external configuration file exists
    local config_file="$(dirname "$0")/cross_compile_config.sh"
    if [[ -f "$config_file" ]]; then
        print_info "Loading external configuration file: $config_file"
        source "$config_file"
    else
        print_error "External configuration file not found: $config_file"
        print_info "Please create cross_compile_config.sh file and configure the following parameters:"
        print_info "  export ARCH=<target_architecture>"
        print_info "  export CROSS_COMPILE=<cross_compilation_toolchain_path>"
        print_info "  export KERNEL_LOCATION=<kernel_source_path>"
        print_info "  export EXTRA_CFLAGS=<additional_compilation_parameters>(optional)"
        exit 1
    fi

    if [[ -n "$CLI_KERNEL_LOCATION" ]]; then
        KERNEL_LOCATION="$CLI_KERNEL_LOCATION"
    fi
    
    print_success "Cross-compilation parameters configuration completed"
    print_info "Architecture: $ARCH"
    print_info "Cross-compiler: $CROSS_COMPILE"
    print_info "Kernel path: $KERNEL_LOCATION"
}

# Check cross-compilation environment
check_cross_compile_env() {
    print_info "Checking cross-compilation environment..."
    
    # Check if cross-compiler exists
    if [[ ! -f "${CROSS_COMPILE}gcc" ]]; then
        print_error "Cross-compiler not found: ${CROSS_COMPILE}gcc"
        print_info "Please check CROSS_COMPILE environment variable setting"
        exit 1
    fi
    
    local resolved_kernel_location=""
    if ! resolved_kernel_location=$(kcan_validate_kernel_location "$KERNEL_LOCATION"); then
        kcan_print_kernel_location_guidance "$(uname -r)"
        exit 1
    fi
    KERNEL_LOCATION="$resolved_kernel_location"
    
    # Check if kernel is configured
    if [[ ! -f "$KERNEL_LOCATION/.config" ]]; then
        print_warning "Kernel not yet configured, executing default configuration..."
        make -C "$KERNEL_LOCATION" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" defconfig
        make -C "$KERNEL_LOCATION" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" modules_prepare
    fi
    
    # Check if kernel module compilation environment is ready
    if [[ ! -f "$KERNEL_LOCATION/scripts/module.lds" ]]; then
        print_info "Preparing kernel module compilation environment..."
        make -C "$KERNEL_LOCATION" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" modules_prepare
    fi
    
    print_success "Cross-compilation environment check passed"
}

# Cross-compile driver
cross_compile_driver() {
    print_info "Starting cross-compilation of KH-UCANFD driver..."
    
    # Configure cross-compilation parameters
    CROSS_COMPILE_CONFIG
    
    # Check cross-compilation environment
    check_cross_compile_env
    
    # Check dependencies
    check_dependencies 1 "gcc"
    
    # Clean previous build
    print_info "Cleaning previous build files..."
    cd driver && make clean
    
    # Cross-compile netdev mode driver
    print_info "Cross-compiling netdev mode driver..."
    make netdev ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" KERNEL_LOCATION="$KERNEL_LOCATION" $EXTRA_CFLAGS
    
    print_success "KH-UCANFD driver cross-compilation completed!"
    print_info "Generated kernel modules: $(find . -name '*.ko' -type f)"
    print_info "You can use file command to verify architecture: file driver/kcan.ko"
}

# Install driver
install_driver() {
    local install_udev=$1
    print_info "Starting installation of KH-UCANFD driver..."
    local host_cc
    local kernel_location=""
    local kernel_path=""

    host_cc=$(resolve_host_cc)
    check_dependencies 0 "$host_cc"
    check_sudo

    kernel_location=$(resolve_runtime_kernel_location "$(uname -r)" 1)
    if ! kernel_path=$(sanitize_kernel_location_for_sudo_make "$kernel_location"); then
        exit 1
    fi
    RUNTIME_KERNEL_LOCATION="$kernel_path"
    print_info "Using kernel headers from: $kernel_path"
    
    # Clean previous build
    print_info "Cleaning previous build files..."
    sudo make clean CC="$host_cc" HOSTCC="$host_cc" KERNEL_LOCATION="$kernel_path"
    
    # Build netdev mode driver
    print_info "Building netdev mode driver..."
    sudo make netdev CC="$host_cc" HOSTCC="$host_cc" KERNEL_LOCATION="$kernel_path"
    
    # Install driver
    print_info "Installing driver to system..."
    sudo make install CC="$host_cc" HOSTCC="$host_cc" KERNEL_LOCATION="$kernel_path"
    
    # Install udev rule only if -rules parameter is provided
    if [[ $install_udev -eq 1 ]]; then
        install_udev_rule
    else
        print_info "Skipping udev rule installation (use -rules parameter to enable)"
    fi
    
    print_success "KH-UCANFD driver installation completed!"
    print_info "You can use the following commands to verify installation:"
    print_info "  lsmod | grep kcan"
    print_info "  ls /sys/class/kcan/"
}

# Install driver with DKMS
install_driver_with_dkms() {
    local install_udev=$1
    print_info "Starting installation of KH-UCANFD driver with DKMS..."
    local host_cc
    local kernel_location=""
    local kernel_path=""

    host_cc=$(resolve_host_cc)
    check_dependencies 0 "$host_cc"
    check_dkms
    check_sudo

    kernel_location=$(resolve_runtime_kernel_location "$(uname -r)" 1)
    if ! kernel_path=$(sanitize_kernel_location_for_sudo_make "$kernel_location"); then
        exit 1
    fi
    RUNTIME_KERNEL_LOCATION="$kernel_path"
    print_info "Using kernel headers from: $kernel_path"

    # Clean previous build
    print_info "Cleaning previous build files..."
    sudo make clean CC="$host_cc" HOSTCC="$host_cc" KERNEL_LOCATION="$kernel_path"

    # Build netdev mode driver
    print_info "Building netdev mode driver..."
    sudo make netdev CC="$host_cc" HOSTCC="$host_cc" KERNEL_LOCATION="$kernel_path"

    # Install driver with DKMS
    print_info "Installing driver with DKMS..."
    sudo make install_with_dkms CC="$host_cc" HOSTCC="$host_cc" KERNEL_LOCATION="$kernel_path"

    # Install udev rule only if -d-rules parameter is provided
    if [[ $install_udev -eq 1 ]]; then
        install_udev_rule
    else
        print_info "Skipping udev rule installation (use -d-rules parameter to enable)"
    fi

    print_success "KH-UCANFD driver installation with DKMS completed!"
    print_info "The driver will be automatically rebuilt when kernel is upgraded"
    print_info "You can use the following commands to verify installation:"
    print_info "  dkms status"
    print_info "  lsmod | grep kcan"
    print_info "  ls /sys/class/kcan/"
}

# Uninstall driver
uninstall_driver() {
    print_info "Starting uninstallation of KH-UCANFD driver..."
    local kernel_location=""
    local kernel_path=""

    check_sudo
    kernel_location=$(resolve_runtime_kernel_location "$(uname -r)" 0 2>/dev/null || true)
    if [[ -n "$kernel_location" ]]; then
        if ! kernel_path=$(sanitize_kernel_location_for_sudo_make "$kernel_location"); then
            exit 1
        fi
    fi
    RUNTIME_KERNEL_LOCATION="$kernel_path"

    # Always check and remove udev rule if it exists
    uninstall_udev_rule

    # Uninstall driver
    print_info "Uninstalling driver..."
    if [[ -n "$kernel_path" ]]; then
        sudo make uninstall KERNEL_LOCATION="$kernel_path"
    else
        sudo make uninstall
    fi

    # Clean compilation files
    print_info "Cleaning compilation files..."
    if [[ -n "$kernel_path" ]]; then
        sudo make clean KERNEL_LOCATION="$kernel_path"
    else
        sudo make clean
    fi

    print_success "KH-UCANFD driver uninstallation completed!"
}

# Install prerequisites
install_prerequisites() {
    local preqs_script="$(dirname "$0")/preqs.sh"
    
    if [[ ! -f "$preqs_script" ]]; then
        print_error "Prerequisites script not found: $preqs_script"
        exit 1
    fi
    
    print_info "Running prerequisites installation script..."
    if [[ -n "$CLI_KERNEL_LOCATION" ]]; then
        bash "$preqs_script" --kernel-location "$CLI_KERNEL_LOCATION"
    else
        bash "$preqs_script"
    fi
}

# Show help information
show_help() {
    # Define ANSI color codes
    local BOLD='\033[1m'
    local GREEN='\033[0;32m'
    local YELLOW='\033[0;33m'
    local BLUE='\033[0;34m'
    local NC='\033[0m' # No Color

    echo -e "${BOLD}KH-UCANFD Linux SDK Install/Uninstall Script${NC}"
    echo "==============================================================="
    
    echo -e "${YELLOW}Usage:${NC}"
    echo "  sudo $0 [Options]"
    echo ""

    echo -e "${YELLOW}Installation Options:${NC}"
    printf "  %-18s %s\n" "-p"          "Install prerequisites"
    printf "  %-18s %s\n" "-rules"     "Install driver with udev rules"
    printf "  %-18s %s\n" "-dkms"       "Install with DKMS (auto-rebuild on kernel update)"
    printf "  %-18s %s\n" "-dkms-rules" "Install with DKMS and udev rules"
    printf "  %-18s %s\n" "-c"         "Cross-compile driver (requires manual config)"
    echo ""

    echo -e "${YELLOW}Uninstall Options:${NC}"
    printf "  %-18s %s\n" "-u, -uninstall" "Remove driver"
    echo ""

    echo -e "${YELLOW}General Options:${NC}"
    printf "  %-18s %s\n" "--kernel-location" "Use an explicit kernel build tree"
    printf "  %-18s %s\n" "-h, --help"  "Show this help information"
    echo ""

    echo -e "${BLUE}Examples:${NC}"
    echo -e "  sudo $0 ${GREEN}-dkms-rules${NC}      # Recommended: DKMS mode with udev rules"
    echo "  sudo $0 -u               # uninstall the driver"
    echo "  CC=gcc-12 sudo $0        # Use specific compiler version"
    echo "  sudo $0 --kernel-location /path/to/kernel/tree"
    echo "  KERNEL_LOCATION=/path/to/kernel/tree sudo $0"
    
    echo ""
    echo -e "${YELLOW}Technical Notes:${NC}"
    echo -e "  ${BOLD}• DKMS:${NC} Ensures driver persistence after system kernel updates."
    echo -e "  ${BOLD}• Udev:${NC} Rules are only applied with ${BLUE}-rules${NC} or ${BLUE}-d-rules${NC}."
    echo -e "  ${BOLD}• Cross-Compile:${NC} Edit ${BLUE}CROSS_COMPILE_CONFIG()${NC} in the script before using -c."
    echo "==============================================================="
}

parse_args() {
    ACTION="install"
    INSTALL_UDEV=0
    USE_DKMS=0

    while [[ $# -gt 0 ]]; do
        case "$1" in
            "--kernel-location")
                if [[ $# -lt 2 ]]; then
                    print_error "Missing value for --kernel-location"
                    exit 1
                fi
                CLI_KERNEL_LOCATION="$2"
                shift 2
                ;;
            "--kernel-location="*)
                CLI_KERNEL_LOCATION="${1#*=}"
                shift
                ;;
            "-h"|"--help")
                ACTION="help"
                shift
                ;;
            "-p"|"--prereqs")
                ACTION="prereqs"
                shift
                ;;
            "-c"|"--cross")
                ACTION="cross"
                shift
                ;;
            "-u"|"--uninstall"|"-uninstall")
                ACTION="uninstall"
                shift
                ;;
            "-dkms")
                USE_DKMS=1
                shift
                ;;
            "-dkms-rules")
                USE_DKMS=1
                INSTALL_UDEV=1
                shift
                ;;
            "-rules")
                INSTALL_UDEV=1
                shift
                ;;
            *)
                print_error "Unknown parameter: $1"
                show_help
                exit 1
                ;;
        esac
    done
}

# Main function
main() {
    print_info "KH-UCANFD Linux SDK Build Script"
    print_info "Working directory: $SCRIPT_DIR"
    parse_args "$@"

    case "$ACTION" in
        "help")
            show_help
            exit 0
            ;;
        "prereqs")
            install_prerequisites
            ;;
        "cross")
            cross_compile_driver
            ;;
        "uninstall")
            uninstall_driver
            ;;
        "install")
            if [[ "$USE_DKMS" -eq 1 ]]; then
                install_driver_with_dkms "$INSTALL_UDEV"
            else
                install_driver "$INSTALL_UDEV"
            fi
            ;;
    esac
}

# Execute main function
main "$@"
