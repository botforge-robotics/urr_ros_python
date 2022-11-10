# 03 Services(Python)
Here we'll create the service ("dht") which will return temp and humidity upon calling that service.

### 1. Create Package
- Change directory into `~catkin_ws/src/urr_ros_python/` stack folder.
- Create a new package called `urr_p_03_services` which depends on `rospy` :
```bash
# Create package
catkin_create_pkg urr_p_03_services rospy
```

For more info about catkin_package [click here](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

### 2. Creating Server node
- Change directory into `~urr_p_03_services/scripts/`.
```bash
#Change to package directory
roscd urr_p_03_services
#Create scripts folder
mkdir scripts
#change to scripts folder
cd scripts
```
- Write server node
```bash
# get the server python script from repo
wget https://gitlab.com/UnrealRobotics/urr_ros_python/-/tree/main/urr_p_03_services/scripts/server.py
# or
# creates file with name server.py
touch server.py
```
Write the Code
```python
#!/usr/bin/env python3

from std_srvs.srv import Trigger,TriggerResponse
import rospy

def handle_request(req):
    resp = TriggerResponse() #variable to store responce data
    # do some sensor data collection
    temperature = 26
    humdidity = 69
    # end data collection
    resp.success = True
    resp.message = "The temp is {}c and humidity is {}%.".format(temperature,humdidity)
    return resp

def dht_server():
    rospy.init_node('dht_server')
    s = rospy.Service('dht', Trigger, handle_request)
    print("Ready to collect temp. humid. data!")
    rospy.spin()

if __name__ == "__main__":
    dht_server()
```
### 3. Creating Client node
- Change directory into `~urr_p_03_services/scripts/`.
```bash
#Change to package directory
roscd urr_p_03_services
#Create scripts folder
mkdir scripts
#change to scripts folder
cd scripts
```
- Write client node
```bash
# get the client python script from repo
wget https://gitlab.com/UnrealRobotics/urr_ros_python/-/tree/main/urr_p_03_services/scripts/client.py
# or
# creates file with name client.py
touch client.py
```
Write the Code
```python
#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest

def client():
    rospy.wait_for_service('dht')
    try:
        dht_client = rospy.ServiceProxy('dht', Trigger)
        resp = dht_client(TriggerRequest())
        if(resp.success):
            print(resp.message)
        else:
            print("failed to get data")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    client()
```
### 4. Making script executable
```bash
roscd urr_p_03_services/scripts
# set permission to these files as executable 
chmod +x server.py
chmod +x client.py
```
### 5. Adding script to cmake
Edit the `catkin_install_python()` call in your CMakeLists.txt so it looks like the following:
```cmake
catkin_install_python(PROGRAMS scripts/server.py 
  scripts/client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
### 6. Building your nodes
```bash
roscd ; cd .. #~/catkin_ws
catkin_make
```
### 7. Running nodes
- ####  Running the server
   
    ```bash
    roscore # Make sure that a roscore is up and running
    rosrun urr_p_03_services server.py
    ```
- ####  Check dht service server is online and running
    type below command
     ```bash
    rosservice list
    ```
    output:
    ```bash
    ....
    /dht
    /dht_server/get_loggers
    /dht_server/set_logger_level
    ....
    ```
    you should have to see `/dht` in the ouput list.
- ####  get service type of `/dht` server uses
    type below command
     ```bash
    rosservice type /dht
    ```
    output:
    ```bash
    std_srvs/Trigger
    ```
    `/dht` server uses service of above type, where `std_srvs` is package name and `Trigger` is service name.

- ####  get `Trigger` service type details
    type below command
     ```bash
    rossrv show std_srvs/Trigger
    ```
    output:
    ```bash
    ---
    bool success
    string message
    ```
- #### Calling service through commandline
    type below command
   ```bash
    rosservice call /dht "{}" 
    ```
    output:
    ```bash
    success: True
    message: "The temp is 26c and humidity is 69%."
    ```
- ####  Running the client
   run client node
   ```bash
   rosrun urr_p_03_services client.py
   ```
    output:
   ```bash
    The temp is 26c and humidity is 69%.
   ```

When you are done, press Ctrl-C to terminate  the server node.


### 8. Usefull commands
Cheat Sheet [click here](https://gitlab.com/UnrealRobotics/urr_ros_python/-/blob/main/docs/ROScheatsheet.pdf)
Ros Wiki [click here](http://wiki.ros.org/ROS/CommandLineTools)