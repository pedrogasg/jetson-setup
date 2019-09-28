# JETSON setup

Quick (actually very long) remainder how to setup a jetson nano

## Hardware
---

What you actually need as material

+ [Jetson Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)

+ [Intel® Dual Band Wireless-AC 8265](https://ark.intel.com/content/www/us/en/ark/products/94150/intel-dual-band-wireless-ac-8265.html)

+ [5V 4A (4000mA) switching power supply ](https://www.adafruit.com/product/1466)(Any powersuply will do as is 5V, 4A, 5.5mm OD and 2.1mm ID)

+ [Micro SD](https://www.sandisk.com/home/memory-cards/microsd-cards/extreme-microsd) (At least 32G)

+ Standard Computer Jumper Caps

+ A micro USB cable

## Preparing the kit
---

You only need to put the kit together if you want you can follow the tutorials

+ [For the WIFI](https://www.jetsonhacks.com/2019/04/08/jetson-nano-intel-wifi-and-bluetooth/)

+ [For use the barrel adapter](https://www.jetsonhacks.com/2019/04/10/jetson-nano-use-more-power/)


### Step 1 - Flash micro SD card for the latest version

1. Download the image in the [nvida site](https://developer.nvidia.com/embedded/downloads) not use the sdk manager, that is currently not working right

2. Insert a 32GB+ SD card into the desktop machine

3. Using [Etcher](https://www.balena.io/etcher/) unzip and select the image downloaded before


### Step 2 - Power on and connect

1. Insert the configured SD card into the Jetson Nano module

2. Power on by plugging the powersuply to the jetson

3. Connect the device to a screan and create an user account

### Step 3 - Connect the Jetson to WiFi

1. Open a terminal

2. In the terminal, type the following command to list available WiFi networks, and find the ``ssid_name`` of your network.

    ```bash
    sudo nmcli device wifi list
    ```
3. Connect to  the selected WiFi network

    >  It should be on the same network that you will be connect to jetson

    ```bash
    sudo nmcli device wifi connect <ssid_name> password <password>
    ```
4. Note down the WiFi IP returned by the following command.  We'll call this ``jetson_ip_address``
    
    ```bash
    ip route get 1.2.3.4 | awk '{print $7}'
    ```
5. Turn off the power safe mode to avoid losing the wifi connection

    ```bash
    sudo iw dev wlan0 set power_save off
    ```
### Step 4 - Connect to the Jetson over WiFi in ssh 

1. Unplug the micro USB cable from the Jetson Nano

2. Close the previous Jupyter Lab browser tab

3. Open a new terminal and connect to the jetson in ssh wirh the following command

    ```bash
    ssh jetson@<jetson_ip_address>
    ```
4. Sign in with the password ``jetson``

## Intalling librealsense and ros
---

### Step 1-2 - Realsense setup

> Right now we are using the version 2.28

1. Add a swapfile to jetson

    ```bash
    git clone https://github.com/jetsonhacksnano/installSwapfile
    cd installSwapfile
    ./installSwapfile.sh
    cd ..
    ```

2. Add the ubuntu keyserver
    ```bash
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F42ED6FBAB17C654
    ```

3. Update the repo
    ```bash
    sudo apt update
    ```

4. Install dependencies
    ```bash
    sudo apt install -y wget git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev python3-dev cmake
    ```

5. Download librealsense

    ```bash
    sudo wget https://github.com/IntelRealSense/librealsense/archive/v2.28.0.tar.gz -O librealsense.tar.gz
    ```

6. Untar librealsense

    ```bash
    tar xvzf librealsense.tar.gz
    ```

7. Go to librealsense folder and create the build folder 

    ```bash
    cd librealsense-2.28.0 && mkdir build && cd build
    ```

8. Run cmake build

    ```bash
     cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS:bool=true -DBUILD_WITH_CUDA:bool=true -DCMAKE_CUDA_COMPILER=/usr/local/cuda-10.0/bin/nvcc
    ```

9. Run Make with 4 cores

    ```bash
    make -j4
    ```

10. Install all the tools and libs

    ```bash
    sudo make install
    ```

11. Before using you need copy the udev rules for the realsense and restart udevadm

    ```bash
    cd ..
    sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && udevadm trigger
    ```


### Step 2 - ROS setup

> [ROS](https://github.com/JetsonHacksNano/installROS) and [RealsenseROS](https://github.com/JetsonHacksNano/installRealSenseROS) also have utility scripts you can run the ROS one with the follow commands

1. Install librealsense with the utility script
    ```bash
    git clone https://github.com/JetsonHacksNano/installROS
    cd installROS
    ./installROS.sh -p ros-melodic-desktop -p ros-melodic-rgbd-launch
    cd ..
    ```
### Step 3 - Realsense ROS setup

> Jetson Hacks provide a script to make a catkin workspace that use catkin_make as the script to install realsense ROS support also catkin tools we going to create the workspace using catkin tools

1. Install catkin tools

    ```bash
    sudo apt-get install python-catkin-tools python-rosinstall-generator -y
    ```
2. Create the workspace for realsense ROS

    ```bash
    mkdir -p ~/catkin_ws_realsense/src
    cd ~/catkin_ws_realsense
    catkin init
    cd ..
    ```
3. Install realsense ROS with the utility script

    ```bash
    git clone https://github.com/JetsonHacksNano/installRealSenseROS
    cd installRealSenseROS
    ./installRealSenseROS.sh catkin_ws_realsense
    ./setupNano.sh
    cd ..
    ```
4. Add the workspace to the PATH and the .bashrc file

    ```bash
    echo "source /home/jetson/catkin_ws_realsense/devel/setup.bash --extend" >> ~/.bashrc
    ```

## Intalling mavros
---

1. Create the workspace: unneeded if you already has workspace

    ```bash
    mkdir -p ~/catkin_ws_mavros/src
    cd ~/catkin_ws_mavros
    catkin init
    wstool init src
    ```
2. Install MAVLink
    > We use the Kinetic reference for all ROS distros as it's not distro-specific and up to date

    ```bash
    rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall
    ```
3. Install MAVROS: get source (upstream - released)
    >  Install latest source

    ```bash
    rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
    ```
4. Create workspace & deps

    ```bash
    wstool merge -t src /tmp/mavros.rosinstall
    wstool update -t src -j4
    rosdep install --from-paths src --ignore-src -y
    ```
5. Install GeographicLib datasets:

    ```bash
    sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
    ```
6. Build source

    ```bash
    catkin build
    ```
7. Add the workspace to the PATH and the .bashrc file

    ```bash
    echo "source /home/jetson/catkin_ws_mavros/devel/setup.bash --extend" >> ~/.bashrc
    ```
8. Set the rights to the tty

    ```bash
    sudo chmod 777 /dev/ttyACM0
    ```