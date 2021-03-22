cd ~/ros_catkin_ws/src
git clone https://github.com/houseofbigseals/rosserial_ard_service_test.git
cd ~/ros_catkin_ws/
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=rosserial_ard_service_test
