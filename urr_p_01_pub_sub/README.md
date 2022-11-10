# 01 Publisher-Subscriber(Python)
### 1. Create Package
- Change directory into `~catkin_ws/src/urr_ros_python/` stack folder.
- Now use the `catkin_create_pkg` script to create a new package called `urr_p_01_pub_sub` which depends on `std_msgs and rospy` :
```bash
# Create package
catkin_create_pkg urr_p_01_pub_sub std_msgs rospy
```
This will create a `urr_p_01_pub_sub` folder which contains a `package.xml` and a `CMakeLists.txt`, which have been partially filled out with the information you gave catkin_create_pkg.

*usage:*
```bash
# This is an example, do not try to run this
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
For more info about catkin_package [click here](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

### 2. Creating publisher node
- Change directory into `~urr_p_01_pub_sub/scripts/`.
```bash
#Change to package directory
roscd urr_p_01_pub_sub
#Create scripts folder
mkdir scripts
#change to scripts folder
cd scripts
```
- Write publisher node
```bash
# get the publisher python script from repo
wget https://gitlab.com/UnrealRobotics/urr_ros_python/-/tree/main/urr_p_01_pub_sub/scripts/publisher.py
# or
# creates file with name publisher.py
touch publisher.py
```
Write the Code
```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('talker', anonymous=False)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
### 3. Creating subscriber node
- Write subscriber node
```bash
# get the subscriber python script from repo
wget https://gitlab.com/UnrealRobotics/urr_ros_python/-/tree/main/urr_p_01_pub_sub/scripts/subscriber.py
# or
# creates file with name subscriber.py
touch subscriber.py
```
Write the Code
```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```
### 4. Making script executable
```bash
roscd urr_p_01_pub_sub/scripts
# set permission to these files as executable 
chmod +x publisher.py
chmod +x subscriber.py
```
### 5. Adding script to cmake
Edit the `catkin_install_python()` call in your CMakeLists.txt so it looks like the following:
```cmake
catkin_install_python(PROGRAMS scripts/publisher.py scripts/subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
### 6. Building your nodes
```bash
roscd ; cd ..   #~/catkin_ws
catkin_make
```
### 7. Running nodes
- ####  Running the Publisher
    Make sure that a roscore is up and running:
   ```bash
   roscore
   ```
    Make sure you have sourced your workspace's `setup.sh`

    ```bash
    cd ~/catkin_ws
    source ./devel/setup.bash
    ```
    Running nodes
    *usage:*
    ```bash
    rosrun <package_name> <executable>
    ```
    Let's run publisher:
    
    ```bash
    rosrun urr_p_01_pub_sub publisher.py
    ```
    you will see something similar to:
    ```bash
    [INFO] [WallTime: 1314931831.774057] hello world 1314931831.77
    [INFO] [WallTime: 1314931832.775497] hello world 1314931832.77
    [INFO] [WallTime: 1314931833.778937] hello world 1314931833.78
    [INFO] [WallTime: 1314931834.782059] hello world 1314931834.78
    ```
   
- ####  Running the Subscriber
    Let's run publisher:
   ```bash
   rosrun urr_p_01_pub_sub subscriber.py
   ```
    You will see something similar to:
   ```bash
    [INFO] [WallTime: 1314931969.258941] /listener heard hello world 1314931969.26
    [INFO] [WallTime: 1314931970.262246] /listener heard hello world 1314931970.26
    [INFO] [WallTime: 1314931971.266348] /listener heard hello world 1314931971.26
    [INFO] [WallTime: 1314931972.270429] /listener heard hello world 1314931972.27
   ```

When you are done, press Ctrl-C to terminate both the publisher and the subscriber.


### 8. Usefull commands
Cheat Sheet [click here](https://gitlab.com/UnrealRobotics/urr_ros_python/-/blob/main/docs/ROScheatsheet.pdf)
Ros Wiki [click here](http://wiki.ros.org/ROS/CommandLineTools)