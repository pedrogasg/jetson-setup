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

## Image and connection to the wifi
---

For the image and the WIFI we are going to use the [jetcard](https://github.com/NVIDIA-AI-IOT/jetcard) image and almost the same procedure that the [jetracer software](https://github.com/NVIDIA-AI-IOT/jetracer/blob/master/docs/software_setup.md) 

### Step 1 - Flash micro SD card

1. Download the JetCard SD card image [jetcard_v0p0p0.img](https://drive.google.com/open?id=1wXD1CwtxiH5Mz4uSmIZ76fd78zDQltW_) onto a Windows, Linux or Mac *desktop machine*
    
    > You can check it against this [md5sum](https://drive.google.com/open?id=1356ZBrYUWaTgbV50UMB1uCfWrNcd6PEF)

2. Insert a 32GB+ SD card into the desktop machine
3. Using [Etcher](https://www.balena.io/etcher/) select ``jetcard_v0p0p0.img`` and flash it onto the SD card
4. Remove the SD card from the desktop machine

> Please note, the password for the pre-built SD card is ``jetson``

### Step 2 - Power on and connect over USB

1. Insert the configured SD card into the Jetson Nano module

2. Power on by plugging the powersuply to the jetson

3. Connect your Windows, Linux, or Mac machine to the Jetson Nano via micro USB

4. On your Windows, Linux, or Mac machine, open a browser and navigate to ``192.168.55.1:8888``
5. Sign in using the default password ``jetson``

### Step 3 - Connect the Jetson to WiFi

1. Open a terminal in Jupyter Lab by clicking ``File`` -> ``New`` -> ``Terminal``

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
### Step 1 - Realsense setup

> To install librealsense you only need to use the utility scripts of jetson hacks for the [435D](https://www.jetsonhacks.com/2019/05/07/jetson-nano-realsense-tracking-camera/) or for the [265T](https://www.jetsonhacks.com/2019/05/16/jetson-nano-realsense-depth-camera/) they basically tell you to run the follow commands (if you need a newer version see Step 1-2

1. Add a swapfile to jetson

    ```bash
    git clone https://github.com/jetsonhacksnano/installSwapfile
    cd installSwapfile
    ./installSwapfile.sh
    cd ..
    ```

2. Install librealsense with the utility script
    ```bash
    git clone https://github.com/jetsonhacksnano/installLibrealsense
    cd installLibrealsense
    ./installLibrealsense.sh -c
    ./patchUbuntu.sh
    cd ..
    ```

### Step 1-2 - Realsense setup (solo)

> Right now we are using the version 2.28

1. Add the ubuntu keyserver
    ```bash
    sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F42ED6FBAB17C654
    ```

2. Update the repo
    ```bash
    sudo apt update
    ```

3. Install dependencies
    ```bash
    sudo apt install -y wget git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev cmake
    ```

4. Download librealsense

    ```bash
    sudo wget https://github.com/IntelRealSense/librealsense/archive/v2.28.0.tar.gz -O librealsense.tar.gz
    ```

5. Untar librealsense

    ```bash
    tar xvzf librealsense.tar.gz
    ```

6. Go to librealsense folder and create the build folder 

    ```bash
    cd librealsense-2.28.0 && mkdir build && cd build
    ```

7. Run cmake build

    ```bash
     cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS:bool=true -DBUILD_WITH_CUDA:bool=true -DCMAKE_CUDA_COMPILER=/usr/local/cuda-10.0/bin/nvcc
    ```

8. Run Make with 4 cores

    ```bash
    make -j4
    ```

9. Install all the tools and libs

    ```bash
    sudo make install
    ```

10. Before using you need copy the udev rules for the realsense and restart udevadm

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