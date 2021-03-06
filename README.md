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

1. Download the image in the [nvida site](https://developer.nvidia.com/embedded/downloads) not use the sdk manager, that is currently not working right know

2. Insert a 32GB+ SD card into the desktop machine

3. Using [Etcher](https://www.balena.io/etcher/) unzip and select the image downloaded before


### Step 2 - Power on and connect

1. Insert the configured SD card into the Jetson Nano module

2. Power on by plugging the powersuply to the jetson

3. Without a display plugged the jetson start in [headless mode]( https://www.jetsonhacks.com/2019/08/21/jetson-nano-headless-setup/), you can connect to the jetson with the follow command this will guide you to install ubuntu and create the user and the wifi connection

    ```bash
    screen /dev/ttyACM0 115200
    ```

4. Connect to the jetson through ssh

### Step 3 - Connect the Jetson to WiFi (If the wifi is not working)

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

### Step 3.1 - Connect the Jetson to WiFi (In wap2 entreprise)

1. Follow the 2 first steps

2. Create WPA2 Entreprise connexion

    ```bash
    nmcli con add type wifi ifname wlan0 con-name <connexion-name> ssid <ssid_name>
    nmcli con edit id <connexion-name>
    set ipv4.method auto
    set 802-1x.eap peap
    set 802-1x.phase2-auth mschapv2
    set 802-1x.identity <user_name>
    set 802-1x.password <password>
    set wifi-sec.key-mgmt wpa-eap
    save
    activate
    ```
3. Continue as step 3


### Step 4 - Connect to the Jetson over WiFi in ssh 

1. Unplug the the Jetson Nano HDMI if you use a display or the micro usb if you use the headless mode

3. Open a new terminal in your main machine and connect to the jetson in ssh with the following command

    ```bash
    ssh <user_name>@<jetson_ip_address>
    ```
4. Sign in with the password <user_password>

## Installing librealsense and ros
---

### Step 1 - Realsense setup

> Right now we are using the version 2.31

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
    sudo wget https://github.com/IntelRealSense/librealsense/archive/v2.31.0.tar.gz -O librealsense.tar.gz
    ```

6. Untar librealsense

    ```bash
    tar xvzf librealsense.tar.gz
    ```

7. Go to librealsense folder and create the build folder 

    ```bash
    cd librealsense-2.31.0 && mkdir build && cd build
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
> We install ros melodic and the dependencies direclty from the repo

1. Add ros source to the deb list
    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```

2. Add the key to the key server
    ```bash
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 
    ```

3. Update apt list
    ```bash
    sudo apt-get update
    ```

4. Install ros desktop and rgbd launch
    ```bash
    sudo apt-get install -y ros-melodic-desktop ros-melodic-rgbd-launch
    ```

5. Install and run rosdep
    ```bash
    sudo apt-get install -y python-rosdep
    sudo rosdep init
    rosdep update
    ```

6. Add setup bash to bashrc and source bash rc
    ```bash
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

7. Install dependencies for ros
    ```bash
    sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools  -y
    ```

8. Install dependecies for python3
    ```bash
    sudo apt-get -y install python3-pip python3-yaml
    sudo pip3 install rospkg catkin_pkg
    ```

### Step 3 - Realsense ROS setup

> The current master support the version 2.31 of realsence

1. Create the workspace for realsense ROS

    ```bash
    mkdir -p ~/catkin_ws_rs/src
    cd ~/catkin_ws_rs
    catkin init
    cd src
    ```

2. Clone realsense ros and ddynamic reconfigure
    ```bash
    git clone https://github.com/pal-robotics/ddynamic_reconfigure
    git clone https://github.com/IntelRealSense/realsense-ros.git
    ```

3. Checkout the branch 2.2.12 to go with our version and change CMakelist to works also with the 2.2.12 version
    ```bash
    cd realsense-ros/
    git checkout 2.2.12
    sed -i 's/find_package(realsense2 2.25.0)/find_package(realsense2 2.28.0)/' realsense2_camera/CMakeLists.txt 
    cd ../..
    ```

4. Use catkin to make the package
    ```bash
    catkin config -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
    catkin build
    ```

5. Add the workspace to the PATH and the .bashrc file

    ```bash
    echo "source /home/jetson/catkin_ws_rs/devel/setup.bash --extend" >> ~/.bashrc
    source ~/.bashrc
    ```

## Intalling mavros
---

1. Create the workspace: unneeded if you already have workspace

    ```bash
    mkdir -p ~/catkin_ws_mavros/src
    cd ~/catkin_ws_mavros
    catkin init
    wstool init src
    ```
2. Install MAVLink
    > We can use the melodic reference for all ROS distros as it's not distro-specific and is up to date `--rosdistro melodic` this example use the master

    ```bash
    rosinstall_generator --upstream-development mavlink | tee /tmp/mavros.rosinstall
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
    source ~/.bashrc
    ```
8. Set the rights to the ttyTHS in the udev rules

    ```bash
    sudo sh -c 'echo "KERNEL==\"ttyTHS*\", MODE=\"0666\""  > /etc/udev/rules.d/55-jetsonserial.rules'
    systemctl stop nvgetty
    systemctl disable nvgetty
    udevadm trigger (or reboot)
    ```

## Intalling tensorflow
---
> Right now we are using the version 1.14

1. Installing dependencies

    ```bash
    sudo apt-get install -y libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev
    ```

2. Installing pip and python dependencies

    ```bash
    sudo apt-get install -y python3-pip 
    sudo pip3 install -U pip testresources setuptools
    sudo pip3 install -U numpy==1.16.1 future==0.17.1 mock==3.0.5 h5py==2.9.0 keras_preprocessing==1.0.5 keras_applications==1.0.8 gast==0.2.2 enum34 futures protobuf

    ```

3. Installing official tensorflow for nano, build by Nvidia

    ```bash
    $ sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v43 tensorflow-gpu

    ```
