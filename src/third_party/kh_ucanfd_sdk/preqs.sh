#!/bin/bash

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLI_KERNEL_LOCATION=""

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

# Check if a package is installed
check_package_installed() {
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q "install ok installed"
}

# Check if a command exists
check_command_exists() {
    command -v "$1" &> /dev/null
}

KERNEL_HELPER="$SCRIPT_DIR/kernel_location.sh"
if [[ ! -f "$KERNEL_HELPER" ]]; then
    print_error "Kernel location helper not found: $KERNEL_HELPER"
    exit 1
fi
source "$KERNEL_HELPER"

# Detect kernel GCC version from compile.h (Kbuild style)
detect_kernel_gcc_version() {
    local kernel_version=$(uname -r)
    local kernel_location=""
    local compile_h=""
    local compiler_info=""
    local gcc_version=""

    kernel_location=$(kcan_find_kernel_location "$kernel_version" "$CLI_KERNEL_LOCATION" 2>/dev/null || true)
    if [[ -n "$kernel_location" ]]; then
        compile_h="$kernel_location/include/generated/compile.h"
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

# Check and install build-essential
check_and_install_build_essential() {
    print_info "Checking build-essential..."
    if check_package_installed "build-essential"; then
        print_success "build-essential is already installed"
    else
        print_warning "build-essential is not installed, installing..."
        sudo apt-get install -y build-essential
        print_success "build-essential installed successfully"
    fi
}

# Check and install GCC and G++ matching kernel version
check_and_install_matching_gcc_gplusplus() {
    print_info "Detecting kernel GCC version..."

    local kernel_gcc=$(detect_kernel_gcc_version)
    if [[ -z "$kernel_gcc" ]]; then
        print_warning "Unable to detect kernel GCC version, installing default GCC/G++"
        if check_command_exists "g++"; then
            print_success "g++ is already installed"
        else
            print_warning "g++ is not installed, installing..."
            sudo apt-get install -y g++
            print_success "g++ installed successfully"
        fi
        return 0
    fi

    print_info "Kernel was compiled with GCC $kernel_gcc"

    local kernel_gcc_major="${kernel_gcc%%.*}"
    local gcc_package="gcc-$kernel_gcc_major"
    local gpp_package="g++-$kernel_gcc_major"

    # Check if matching gcc and g++ are already installed
    if check_command_exists "gcc-$kernel_gcc_major" && check_command_exists "g++-$kernel_gcc_major"; then
        print_success "GCC $kernel_gcc_major and G++ $kernel_gcc_major are already installed"
        return 0
    fi

    # Install the matching GCC and G++ versions
    print_warning "Installing matching compiler suite: $gcc_package and $gpp_package..."

    if apt-cache show "$gcc_package" &> /dev/null && apt-cache show "$gpp_package" &> /dev/null; then
        sudo apt-get install -y "$gcc_package" "$gpp_package"
        print_success "$gcc_package and $gpp_package installed successfully"
    else
        print_warning "Package $gcc_package or $gpp_package not found in repository"
        print_info "Trying to install default gcc and g++ instead..."
        sudo apt-get install -y gcc g++
        print_success "Default GCC and G++ installed successfully"
    fi
}

# Check and install kernel headers
check_and_install_kernel_headers() {
    local kernel_version=$(uname -r)
    local headers_dir=""
    
    print_info "Checking kernel headers for $kernel_version..."

    if headers_dir=$(kcan_find_kernel_location "$kernel_version" "$CLI_KERNEL_LOCATION"); then
        if ! kcan_require_prepared_kernel_tree "$headers_dir"; then
            exit 1
        fi
        print_success "Kernel headers are already available: $headers_dir"
        return 0
    fi

    if [[ -n "$CLI_KERNEL_LOCATION" ]] || [[ -n "${KERNEL_LOCATION:-}" ]]; then
        kcan_print_kernel_location_guidance "$kernel_version"
        exit 1
    fi

    if check_package_installed "linux-headers-$kernel_version"; then
        print_warning "linux-headers-$kernel_version is installed, but /lib/modules/$kernel_version/build is not usable"
    else
        print_warning "Kernel headers are not installed, installing linux-headers-$kernel_version..."
        if ! sudo apt-get install -y "linux-headers-$kernel_version"; then
            kcan_print_kernel_location_guidance "$kernel_version"
            exit 1
        fi
    fi

    if headers_dir=$(kcan_find_kernel_location "$kernel_version"); then
        if ! kcan_require_prepared_kernel_tree "$headers_dir"; then
            exit 1
        fi
        print_success "Kernel headers installed successfully: $headers_dir"
    else
        kcan_print_kernel_location_guidance "$kernel_version"
        exit 1
    fi
}

# Check and install libpopt-dev
check_and_install_libpopt() {
    print_info "Checking libpopt-dev..."
    if check_package_installed "libpopt-dev"; then
        print_success "libpopt-dev is already installed"
    else
        print_warning "libpopt-dev is not installed, installing..."
        sudo apt-get install -y libpopt-dev
        print_success "libpopt-dev installed successfully"
    fi
}

# Check and install can-utils
check_and_install_can_utils() {
    print_info "Checking can-utils..."
    if check_package_installed "can-utils"; then
        print_success "can-utils is already installed"
    else
        print_warning "can-utils is not installed, installing..."
        sudo apt-get install -y can-utils
        print_success "can-utils installed successfully"
    fi
}

parse_args() {
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
            *)
                print_error "Unknown parameter: $1"
                exit 1
                ;;
        esac
    done
}


# Main function
main() {
    parse_args "$@"
    print_info "KH-UCANFD Linux SDK Prerequisites Installation Script"
    print_info "======================================================"
    
    # Check sudo permissions
    print_info "Checking sudo permissions..."
    if ! sudo -v 2>/dev/null; then
        print_error "Unable to obtain sudo permissions, please check user permission settings"
        exit 1
    fi
    print_success "Sudo permissions check passed"
    echo
    
    # Update package list
    print_info "Updating package list..."
    sudo apt-get update || print_warning "apt-get update failed, continuing anyway..."
    echo
    
    # Check and install dependencies
    check_and_install_build_essential
    echo
    
    check_and_install_matching_gcc_gplusplus
    echo

    check_and_install_kernel_headers
    echo

    check_and_install_libpopt
    echo

    check_and_install_can_utils
    echo
    
    print_success "All prerequisites check and installation completed!"
    print_info "You can now run './build.sh' to build the driver"
}

# Execute main function
main "$@"
