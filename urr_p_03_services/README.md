# 03 Services(Python)
Here we'll create the service ("add_two_ints_server") node which will receive two ints and return the sum.

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
wget https://gitlab.com/UnrealRobotics/urr_ros_python/-/tree/main/urr_p_03_services/scripts/add_two_ints_server.py
# or
# creates file with name add_two_ints_server.py
touch add_two_ints_server.py
```
Write the Code
```python
#!/usr/bin/env python3

from rospy_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
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
wget https://gitlab.com/UnrealRobotics/urr_ros_python/-/tree/main/urr_p_03_services/scripts/add_two_ints_client.py
# or
# creates file with name add_two_ints_client.py
touch add_two_ints_client.py
```
Write the Code
```python
#!/usr/bin/env python3

import sys
import rospy
from rospy_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
```
### 4. Making script executable
```bash
roscd urr_p_03_services/scripts
# set permission to these files as executable 
chmod +x add_two_ints_server.py
chmod +x add_two_ints_client.py
```
### 5. Adding script to cmake
Edit the `catkin_install_python()` call in your CMakeLists.txt so it looks like the following:
```cmake
catkin_install_python(PROGRAMS scripts/add_two_ints_server.py 
  scripts/add_two_ints_client.py
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
    rosrun urr_p_03_services add_two_ints_server.py
    ```
  
- ####  Running the client
   run client node to send 1,2 to server to add up and get result
   ```bash
   rosrun urr_p_03_services add_two_ints_client.py 1 2
   ```
    output:
   ```bash
    Requesting 1+2
    1 + 2 = 3
   ```

When you are done, press Ctrl-C to terminate  the server node.


### 8. Usefull commands
Cheat Sheet [click here](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
Ros Wiki [click here](http://wiki.ros.org/ROS/CommandLineTools)