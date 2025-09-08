#!/bin/bash

# Aurora SDK System-Wide Installation Script
# This script installs the Aurora SDK similar to how Orbbec SDK is typically installed

set -e  # Exit on any error

# Configuration
AURORA_SDK_VERSION="1.1.22"
SDK_NAME="deptrum-stream-aurora900"
INSTALL_PREFIX="/usr/local"
PKG_CONFIG_PATH="/usr/local/lib/pkgconfig"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Aurora SDK System-Wide Installation ===${NC}"
echo -e "${BLUE}Version: ${AURORA_SDK_VERSION}${NC}"
echo -e "${BLUE}Install prefix: ${INSTALL_PREFIX}${NC}"

# Check if running as root
if [[ $EUID -eq 0 ]]; then
    echo -e "${YELLOW}Warning: Running as root. Installation will proceed to system directories.${NC}"
else
    echo -e "${YELLOW}Note: You may need to run with sudo for system-wide installation.${NC}"
fi

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check dependencies
echo -e "\n${BLUE}Checking dependencies...${NC}"
MISSING_DEPS=()

if ! command_exists cmake; then
    MISSING_DEPS+=("cmake")
fi

if ! command_exists pkg-config; then
    MISSING_DEPS+=("pkg-config")
fi

# Check for libusb development files
if ! pkg-config --exists libusb-1.0; then
    MISSING_DEPS+=("libusb-1.0-dev")
fi

if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo -e "${RED}Missing dependencies: ${MISSING_DEPS[*]}${NC}"
    echo -e "${YELLOW}Please install them first:${NC}"
    echo "sudo apt-get update"
    echo "sudo apt-get install cmake pkg-config libusb-1.0-0-dev build-essential"
    exit 1
else
    echo -e "${GREEN}All dependencies satisfied${NC}"
fi

# Get the current directory (assuming script is run from SDK root)
CURRENT_DIR=$(pwd)
SDK_ROOT=""

# Try to find the SDK root directory
if [ -d "deptrum-stream-aurora900-linux-x86_64-v1.1.22-18.04" ]; then
    SDK_ROOT="$CURRENT_DIR/deptrum-stream-aurora900-linux-x86_64-v1.1.22-18.04"
elif [ -d "include/deptrum" ] && [ -d "lib" ]; then
    SDK_ROOT="$CURRENT_DIR"
elif [ -d "../deptrum-stream-aurora900-linux-x86_64-v1.1.22-18.04" ]; then
    SDK_ROOT="$CURRENT_DIR/../deptrum-stream-aurora900-linux-x86_64-v1.1.22-18.04"
else
    echo -e "${RED}Could not find Aurora SDK directory!${NC}"
    echo "Please run this script from the SDK directory or ensure the SDK is properly extracted."
    echo "Expected structure: deptrum-stream-aurora900-linux-x86_64-v1.1.22-18.04/"
    exit 1
fi

echo -e "${GREEN}Found SDK at: ${SDK_ROOT}${NC}"

# Verify SDK structure
if [ ! -d "$SDK_ROOT/include" ] || [ ! -d "$SDK_ROOT/lib" ]; then
    echo -e "${RED}Invalid SDK structure! Missing include or lib directories.${NC}"
    exit 1
fi

# Create installation directories
echo -e "\n${BLUE}Creating installation directories...${NC}"
sudo mkdir -p "$INSTALL_PREFIX/include"
sudo mkdir -p "$INSTALL_PREFIX/lib"
sudo mkdir -p "$PKG_CONFIG_PATH"
sudo mkdir -p "$INSTALL_PREFIX/lib/cmake/AuroraSDK"

# Install headers
echo -e "\n${BLUE}Installing headers...${NC}"
sudo cp -r "$SDK_ROOT/include/"* "$INSTALL_PREFIX/include/"
echo -e "${GREEN}Headers installed to $INSTALL_PREFIX/include/${NC}"

# Install libraries
echo -e "\n${BLUE}Installing libraries...${NC}"
sudo cp -r "$SDK_ROOT/lib/"* "$INSTALL_PREFIX/lib/"

# Set proper permissions and create symlinks for libraries
cd "$INSTALL_PREFIX/lib"
for lib in libdeptrum_stream_*.so*; do
    if [ -f "$lib" ]; then
        sudo chmod 755 "$lib"
        # Create version-less symlinks if they don't exist
        if [[ "$lib" =~ (.+)\.so\.[0-9.]+$ ]]; then
            base_name="${BASH_REMATCH[1]}.so"
            if [ ! -e "$base_name" ]; then
                sudo ln -sf "$lib" "$base_name"
            fi
        fi
    fi
done

echo -e "${GREEN}Libraries installed to $INSTALL_PREFIX/lib/${NC}"

# Create pkg-config file
echo -e "\n${BLUE}Creating pkg-config file...${NC}"
sudo tee "$PKG_CONFIG_PATH/aurora-sdk.pc" > /dev/null << EOF
prefix=$INSTALL_PREFIX
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib
includedir=\${prefix}/include

Name: Aurora SDK
Description: Deptrum Aurora900 Series Camera SDK
Version: $AURORA_SDK_VERSION
Libs: -L\${libdir} -ldeptrum_stream_aurora900
Libs.private: -lusb-1.0 -lpthread
Cflags: -I\${includedir}
EOF

echo -e "${GREEN}pkg-config file created: $PKG_CONFIG_PATH/aurora-sdk.pc${NC}"

# Create CMake find module
echo -e "\n${BLUE}Creating CMake find module...${NC}"
sudo tee "$INSTALL_PREFIX/lib/cmake/AuroraSDK/AuroraSDKConfig.cmake" > /dev/null << 'EOF'
# AuroraSDK CMake configuration file

@PACKAGE_INIT@

# Set the version
set(AuroraSDK_VERSION "1.1.22")

# Find the includes and libraries
find_path(AuroraSDK_INCLUDE_DIR
    NAMES deptrum/device.h deptrum/aurora900_series.h
    PATHS @CMAKE_INSTALL_PREFIX@/include
    NO_DEFAULT_PATH
)

find_library(AuroraSDK_LIBRARY
    NAMES deptrum_stream_aurora900
    PATHS @CMAKE_INSTALL_PREFIX@/lib
    NO_DEFAULT_PATH
)

# Find additional libraries that might be needed
find_library(AuroraSDK_STELLAR400_LIBRARY
    NAMES deptrum_stream_stellar400
    PATHS @CMAKE_INSTALL_PREFIX@/lib
    NO_DEFAULT_PATH
)

find_library(AuroraSDK_AURORA300_LIBRARY
    NAMES deptrum_stream_aurora300
    PATHS @CMAKE_INSTALL_PREFIX@/lib
    NO_DEFAULT_PATH
)

# Set the include directories and libraries
set(AuroraSDK_INCLUDE_DIRS ${AuroraSDK_INCLUDE_DIR})
set(AuroraSDK_LIBRARIES ${AuroraSDK_LIBRARY})

# Add additional libraries if found
if(AuroraSDK_STELLAR400_LIBRARY)
    list(APPEND AuroraSDK_LIBRARIES ${AuroraSDK_STELLAR400_LIBRARY})
endif()

if(AuroraSDK_AURORA300_LIBRARY)
    list(APPEND AuroraSDK_LIBRARIES ${AuroraSDK_AURORA300_LIBRARY})
endif()

# Add system dependencies
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
    list(APPEND AuroraSDK_LIBRARIES ${LIBUSB_LIBRARIES})
    list(APPEND AuroraSDK_INCLUDE_DIRS ${LIBUSB_INCLUDE_DIRS})
endif()

# Check if everything was found
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(AuroraSDK
    REQUIRED_VARS AuroraSDK_LIBRARY AuroraSDK_INCLUDE_DIR
    VERSION_VAR AuroraSDK_VERSION
)

# Create imported target
if(AuroraSDK_FOUND AND NOT TARGET AuroraSDK::AuroraSDK)
    add_library(AuroraSDK::AuroraSDK SHARED IMPORTED)
    set_target_properties(AuroraSDK::AuroraSDK PROPERTIES
        IMPORTED_LOCATION ${AuroraSDK_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES "${AuroraSDK_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "usb-1.0;pthread"
        INTERFACE_COMPILE_DEFINITIONS "DEVICE_TYPE_AURORA900"
    )
endif()

mark_as_advanced(AuroraSDK_INCLUDE_DIR AuroraSDK_LIBRARY)
EOF

# Substitute the install prefix in the CMake config
sudo sed -i "s|@CMAKE_INSTALL_PREFIX@|$INSTALL_PREFIX|g" "$INSTALL_PREFIX/lib/cmake/AuroraSDK/AuroraSDKConfig.cmake"
sudo sed -i "s|@PACKAGE_INIT@||g" "$INSTALL_PREFIX/lib/cmake/AuroraSDK/AuroraSDKConfig.cmake"

echo -e "${GREEN}CMake configuration created: $INSTALL_PREFIX/lib/cmake/AuroraSDK/AuroraSDKConfig.cmake${NC}"

# Update library cache
echo -e "\n${BLUE}Updating library cache...${NC}"
sudo ldconfig

# Create udev rules for Aurora cameras
echo -e "\n${BLUE}Creating udev rules for Aurora cameras...${NC}"
sudo tee "/etc/udev/rules.d/99-aurora-camera.rules" > /dev/null << EOF
# Aurora camera udev rules
# PID 1900 for Aurora900 series
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="1900", MODE="0666", GROUP="plugdev"
# Add other Aurora camera PIDs as needed
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo -e "${GREEN}Udev rules created: /etc/udev/rules.d/99-aurora-camera.rules${NC}"

# Add current user to plugdev group if not already there
if ! groups $USER | grep -q "\bplugdev\b"; then
    echo -e "\n${BLUE}Adding user to plugdev group...${NC}"
    sudo usermod -a -G plugdev $USER
    echo -e "${YELLOW}Note: You may need to log out and log back in for group changes to take effect.${NC}"
fi

# Create a simple test program
echo -e "\n${BLUE}Creating test program...${NC}"
sudo tee "$INSTALL_PREFIX/bin/aurora-test" > /dev/null << 'EOF'
#!/bin/bash
echo "Testing Aurora SDK installation..."

# Test pkg-config
if pkg-config --exists aurora-sdk; then
    echo "✓ pkg-config configuration found"
    echo "  Version: $(pkg-config --modversion aurora-sdk)"
    echo "  Cflags: $(pkg-config --cflags aurora-sdk)"
    echo "  Libs: $(pkg-config --libs aurora-sdk)"
else
    echo "✗ pkg-config configuration not found"
fi

# Test CMake configuration
if [ -f "/usr/local/lib/cmake/AuroraSDK/AuroraSDKConfig.cmake" ]; then
    echo "✓ CMake configuration found"
else
    echo "✗ CMake configuration not found"
fi

# Test library presence
if ldconfig -p | grep -q deptrum_stream; then
    echo "✓ Aurora libraries found in system library path"
else
    echo "✗ Aurora libraries not found in system library path"
fi

# Test headers
if [ -f "/usr/local/include/deptrum/device.h" ]; then
    echo "✓ Aurora headers found"
else
    echo "✗ Aurora headers not found"
fi

echo "Test completed."
EOF

sudo chmod +x "$INSTALL_PREFIX/bin/aurora-test"

# Installation complete
echo -e "\n${GREEN}=== Aurora SDK Installation Complete! ===${NC}"
echo -e "\n${BLUE}Installation Summary:${NC}"
echo -e "  Headers: $INSTALL_PREFIX/include/deptrum/"
echo -e "  Libraries: $INSTALL_PREFIX/lib/libdeptrum_stream_*"
echo -e "  pkg-config: $PKG_CONFIG_PATH/aurora-sdk.pc"
echo -e "  CMake config: $INSTALL_PREFIX/lib/cmake/AuroraSDK/"
echo -e "  Udev rules: /etc/udev/rules.d/99-aurora-camera.rules"

echo -e "\n${BLUE}Usage Examples:${NC}"
echo -e "\n${YELLOW}1. Using pkg-config:${NC}"
echo -e "   g++ -o myapp myapp.cpp \$(pkg-config --cflags --libs aurora-sdk)"

echo -e "\n${YELLOW}2. Using CMake:${NC}"
echo -e "   find_package(AuroraSDK REQUIRED)"
echo -e "   target_link_libraries(myapp AuroraSDK::AuroraSDK)"

echo -e "\n${YELLOW}3. Manual compilation:${NC}"
echo -e "   g++ -o myapp myapp.cpp -I$INSTALL_PREFIX/include -L$INSTALL_PREFIX/lib -ldeptrum_stream_aurora900 -lusb-1.0 -lpthread"

echo -e "\n${BLUE}Test the installation:${NC}"
echo -e "   aurora-test"

echo -e "\n${GREEN}Installation successful!${NC}"

# Check if user needs to log out for group changes
if ! groups $USER | grep -q "\bplugdev\b"; then
    echo -e "\n${YELLOW}Important: Please log out and log back in for USB permissions to take effect.${NC}"
fi