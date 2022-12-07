# 04 Custom service type (Python)
Here we'll create new custom service type which will receive two ints and return the sum. And write server, client nodes to use our service type.

### 1. Create Package
- Change directory into `~catkin_ws/src/urr_ros_python/` stack folder.
- Create a new package called `urr_p_04_custom_srv` which depends on `message_generation, message_runtime and rospy` :
```bash
# Create package
catkin_create_pkg urr_p_04_custom_srv message_generation message_runtime rospy
# dependencies can also add later after creating package we will see further
```
For more info about catkin_package [click here](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)


### 2. Creating a srv
-  Let's define a new srv in the package
```bash
roscd urr_p_04_custom_srv   # Change directory
mkdir srv   #create 'srv' directory
touch srv/AddTwoInts.srv    # Create 'AddTwoInts.srv' file in srv folder
# Note: Naming convention of srv file starts with capital case.
```

- Now edit the `AddTwoInts.srv` file
```bash
#request definition
int64 a
int64 b
---
#responce definition
int64 sum
```
- Now add below line to `package.xml`  file
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>

<!-- Note: At build time, we need `message_generation`,
 while at runtime, we only need `message_runtime`. -->
```

- Now add below line to `CMakeLists.txt`  file
  - Add the *message_generation* dependency to the find_package 
  ```bash
  find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
  )
  ```
 
  - Uncomment add_service_files() by removing the # symbols and add AddTwoInts.srv file, such that it looks like this:
  ```bash
  add_service_files(
    FILES
    AddTwoInts.srv
  )
  ```
   - Add *message_runtime* to *CATKIN_DEPENDS*.
  ```bash
  catkin_package(
    ...
    CATKIN_DEPENDS rospy message_runtime ...
    ...)
  ```
  - Now uncomment *generate_messages()* function.
  ```bash
  generate_messages(
    DEPENDENCIES
  )
  ```

### 3. Build package and services
```bash
roscd ; cd ..   #~/catkin_ws
catkin_make
```
### 4. Using rossrv
- Let's make sure that ROS can see new service type.
- Use `rossrv list` to see available service types:
  ```bash
  rossrv list | grep Add 
  #list all service types and fillter type having Add in it.
  ```
  you can see output like this:
  ```bash
  urr_p_04_custom_srv/AddTwoInts
  ````
  where, *`urr_p_04_custom_srv` is package name* and *`AddTwoInts` is service type.*
- Use `rossrv show` to see service type details:
  ```bash
  rrossrv show urr_p_04_custom_srv/AddTwoInts
  ```
  you can see output like this:
  ```bash
  int64 a
  int64 b
  ---
  int64 sum
  ````

### 5. Custom service server node
`~add_two_ints_server.py`
```python
#!/usr/bin/env python3

from urr_p_04_custom_srv.srv import AddTwoInts,AddTwoIntsResponse
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
### 6. Custom service client node
`~add_two_ints_client.py`
```python
#!/usr/bin/env python3

import sys
import rospy
from urr_p_04_custom_srv.srv import *

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
    if len(sys.argv) < 3:
        print(usage())
        sys.exit(1)
    else:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
```
Don't forget to make the nodes executable:

 ```bash
 chmod +x customPublisher.py
 chmod +x customSubscriber.py
 ```
Don't forget to add scripts to CMAKELists.txt:
```cmake
catkin_install_python(PROGRAMS scripts/add_two_ints_server.py 
  scripts/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
### 7. Build
```bash
roscd ; cd .. #~/catkin_ws
catkin_make
```
### 8. Running nodes
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


### 9. Usefull commands
Cheat Sheet [click here](https://gitlab.com/botforge-robotics/urr_ros_python/-/blob/main/docs/ROScheatsheet.pdf)
Ros Wiki [click here](http://wiki.ros.org/ROS/CommandLineTools)