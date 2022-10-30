# 06 Launch File

### 1. Create Package
- Change directory into `~catkin_ws/src/urr_ros_python/` stack folder.
- Create a new package called `urr_p_06_launchfile_params`:
```bash
# Create package
catkin_create_pkg urr_p_06_launchfile_params
```

### 2. Create launch file
- Change directory into `~urr_p_06_launchfile_params/`.
```bash
#Change to package directory
roscd urr_p_06_launchfile_params
#Create launch folder
mkdir launch
#change to launch folder
cd launch
```
- write launch files
  ```bash
  touch urr_01_pub.launch
  ```
  `~/urr_01_pub.launch`
  ```xml
  <launch>
    <!-- Hello world publisher -->
    <node pkg="urr_p_01_pub_sub" type="publisher.py" name="publisher" output="log" />
    <!-- Custom msg publisher -->
    <node pkg="urr_p_02_custom_msg" type="customPublisher.py" name="customPublisher" output="log" />
  </launch>
  ```
  <br>

  ```bash
  touch urr_02_servers.launch
  ```
  `~/urr_02_servers.launch`
  ```xml
  <launch>
    <!-- including launch file -->
    <include file="$(find urr_p_06_launchfile_params)/launch/urr_01_pub.launch"></include>
    
    <!-- add two ints service server -->
    <node pkg="urr_p_04_custom_srv" type="add_two_ints_server.py" name="add_two_ints_server" output="screen" ></node>

    <!-- action server node -->
    <node pkg="urr_p_05_actions" type="action_server.py" name="action_server" output="screen" ></node>
  
  </launch>
  ```
  <br>

   ```bash
  touch urr_03_args_params.launch
  ```
  `~/urr_03_args_params.launch`
  ```xml
  <launch>
    <arg name="a" default="1"/>
	<arg name="b" default="2"/>

    <!-- params (more about params will be learn in ros navigation section)-->
    <param name="package_name" value="urr_p_06_launchfile_params"/>
    
    <!-- add two ints client node -->
    <node pkg="urr_p_04_custom_srv" type="add_two_ints_client.py" name="add_two_ints_client" output="screen" args="$(arg a) $(arg b)"/>

  </launch>
  ```
### 3. run launch file
- usage
  ```bash
  roslaunch <package_name> <launch_file_name>
  ```

- example
  ```bash
  roslaunch urr_p_06_launchfile_params urr_01_pub.launch 
  ```
  For more info about roslaunch [click here](http://wiki.ros.org/roslaunch)

### 4. Usefull commands
Cheat Sheet [click here](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
Ros Wiki [click here](http://wiki.ros.org/ROS/CommandLineTools)