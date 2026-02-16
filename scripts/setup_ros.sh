#!/bin/bash
set -e

export ROS_DISTRO="jazzy"

# -----------------------------------------------------------------------------
# Locale Setup
# -----------------------------------------------------------------------------
echo "Setting up locale..."
if ! locale | grep -q "LANG=en_US.UTF-8"; then
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    echo "Locale set to en_US.UTF-8"
else
    echo "Locale already set to en_US.UTF-8"
fi

# -----------------------------------------------------------------------------
# ROS 2 Apt Repository
# -----------------------------------------------------------------------------
echo "Adding ROS 2 apt repository..."
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo apt update && sudo apt install -y software-properties-common curl
    sudo add-apt-repository -y universe
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
else
    echo "ROS 2 repository already configured."
fi

# -----------------------------------------------------------------------------
# Install ROS 2 Base & Dev Tools
# -----------------------------------------------------------------------------
echo "Installing ROS 2 development tools and base packages..."
sudo apt update && sudo apt install -y ros-dev-tools

# -----------------------------------------------------------------------------
# Install ROS 2 Desktop
# -----------------------------------------------------------------------------
echo "Installing ROS 2 ${ROS_DISTRO} Desktop..."
sudo apt install -y ros-${ROS_DISTRO}-desktop

# -----------------------------------------------------------------------------
# Environment Setup (.bashrc)
# -----------------------------------------------------------------------------
echo "Configuring .bashrc..."
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    echo "Added ROS source to .bashrc"
fi

# Aliases
grep -q "alias sp=" ~/.bashrc || echo "alias sp='source install/setup.bash'" >> ~/.bashrc
grep -q "alias sr=" ~/.bashrc || echo "alias sr='source /opt/ros/$ROS_DISTRO/setup.bash'" >> ~/.bashrc
grep -q "alias sb=" ~/.bashrc || echo "alias sb='source ~/.bashrc'" >> ~/.bashrc

# -----------------------------------------------------------------------------
# Rosdep Initialization
# -----------------------------------------------------------------------------
echo "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
else
    echo "rosdep already initialized."
fi
rosdep update

# -----------------------------------------------------------------------------
# Install Dependencies from Source
# -----------------------------------------------------------------------------
echo "Installing dependencies from source..."
# Check if we are in a workspace
if [ -d "src" ]; then
    rosdep install --from-paths src --ignore-src -r -y
else
    echo "WARNING: 'src' directory not found. Skipping dependency installation from source."
fi

# -----------------------------------------------------------------------------
# Install Additional Packages
# -----------------------------------------------------------------------------
echo "Installing additional packages..."
# Using the correct package name for Zenoh RMW
sudo apt install -y ros-${ROS_DISTRO}-rmw-zenoh-cpp ros-${ROS_DISTRO}-nav2* ros-${ROS_DISTRO}-slam-toolbox

# -----------------------------------------------------------------------------
# Build
# -----------------------------------------------------------------------------
if [ -f "package.xml" ] || [ -d "src" ]; then
    echo "Building workspace..."
    colcon build --symlink-install
else
    echo "No workspace to build."
fi

echo "ROS 2 Setup Completed Successfully!"
