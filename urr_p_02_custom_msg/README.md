# 02 Custom MSG
### 1. Create Package
- Change directory into `~catkin_ws/src/urr_ros_python/` stack folder.
- Create a new package called `urr_p_02_custom_msg` which depends on `message_generation, message_runtime and rospy` :
```bash
# Create package
catkin_create_pkg urr_p_02_custom_msg message_generation message_runtime rospy
# dependencies can also add later after creating package we will see further
```
For more info about catkin_package [click here](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

### 2. Creating a msg
-  Let's define a new msg in the package
```bash
roscd urr_p_02_custom_msg   # Change directory
mkdir msg   #create 'msg' directory
touch msg/Person.msg    # Create 'Person.msg' file in msg folder
# Note: Naming convention of msg file starts with capital case.
```

- Now edit the `Person.msg` file
```txt
string first_name
string last_name
uint8 age
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
  - Uncomment add_message_files() by removing the # symbols and add Person.msg file, such that it looks like this:
  ```bash
  add_message_files(
    FILES
    Person.msg
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

### 3. Build package and messages
```bash
roscd ; cd ..   #~/catkin_ws
catkin_make
```
### 4. Using rosmsg
- Let's make sure that ROS can see new message type.
- Use `rosmsg list` to see available messages:
  ```bash
  rosmsg list | grep urr 
  #list all messages and fillter messages having urr in it.
  ```
  you can see output like this:
  ```bash
  urr_p_02_custom_msg/Person
  ````
  where, *`urr_p_02_custom_msg` is package name* and *`Person` is message name.*
- Use `rosmsg show` to see message details:
  ```bash
  rosmsg show urr_p_02_custom_msg/Person
  ```
  you can see output like this:
  ```bash
  string first_name
  string last_name
  uint8 age
  ````
### 5. Using custom msg in publisher node
`~customPublisher.py`
```python
#!/usr/bin/env python3

import rospy
from urr_p_02_custom_msg.msg import Person

def publish():
    rospy.init_node('customPublisher', anonymous=False)  # initialize node
    pub = rospy.Publisher('person_details', Person, queue_size=10)  #create publisher of topic person_details
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        instructor = Person() #instance of person message type
        instructor.first_name = "chaitanya"
        instructor.last_name = "mandala"
        instructor.age = 27
        rospy.loginfo("published.. %s" % rospy.get_time())
        pub.publish(instructor) #publish
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
```
### 6. Using custom msg in subscriber node
`~customSubscriber.py`
```python
#!/usr/bin/env python3

import rospy
from urr_p_02_custom_msg.msg import Person

def callback(data):
    name = data.first_name + " "+ data.last_name
    age = data.age
    rospy.loginfo("Hi! I am {}, age {}.".format(name,age))
    
def subscriber():
    rospy.init_node('customSubscriber', anonymous=False)    #initialize node
    rospy.Subscriber("person_details", Person, callback)    #create subscriber to topic person_details
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()
```
Don't forget to make the nodes executable:

 ```bash
 chmod +x customPublisher.py
 chmod +x customSubscriber.py
 ```
### 7. Using custom message in another package
If you are using the new custom message defined in a different package, remember to add:

- to `package.xml`:
```xml
<build_depend>name_of_package_containing_custom_msg</build_depend>
<exec_depend>name_of_package_containing_custom_msg</exec_depend>
```
### 8. Usefull commands
Cheat Sheet [click here](https://gitlab.com/botforge-robotics/urr_ros_python/-/blob/main/docs/ROScheatsheet.pdf)
Ros Wiki [click here](http://wiki.ros.org/ROS/CommandLineTools)