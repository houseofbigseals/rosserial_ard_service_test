
### Description
ROS package to use with arduino rosserial service example  

### How to build
```
cd ~/ros_catkin_ws/src
git clone https://github.com/houseofbigseals/rosserial_ard_service_test.git
cd ~/ros_catkin_ws/
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=rosserial_ard_service_test
```

### How to generate new ros_lib
NOTE! You have to build it on robot, not on pc.  
```
rosrun rosserial_arduino make_libraries.py ~/roslib_new rosserial_ard_service_test
```

### How to upload to onboard arduino via ArduinoIDE
Generate new ros_lib folder (or just unzip from /data in this repo) and add it to ArduinoIDE library path.  
Then open /arduino/main.ino from repo in ArduinoIDE, build and upload  

### How to upload to onboard arduino via PlatformIO with vscode
Import or create new platformio project for megaatmega2560  
Generate new ros_lib folder (or just unzip from /data in this repo) and add path to it to lib_extra_dirs in platformio.ini:  
```
[env:megaatmega2560]
platform = atmelavr
board = megaatmega2560
framework = arduino

lib_deps =

    # RECOMMENDED
     # Accept new functionality in a backwards compatible manner and patches
     arduino-libraries/Servo @ ^1.1.7

platform_packages =
    ; use GCC AVR 7.3.0+
    toolchain-atmelavr@>=1.70300.0

lib_ldf_mode = chain+  # do we need it?
lib_extra_dirs = /home/ramses/work/roslib_new/
```
Then compile and upload  

