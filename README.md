# UnReal Robotics ROS (Python)

### Create catkin worksapce
Let's create and build a catkin workspace:
```bash
cd ~/Desktop
mkdir -p catkin_ws/src
catkin_make
```
Now, if you look in your current directory you should now have a 'build' and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup.*sh files. Sourcing any of these files will overlay this workspace on top of your environment.
### Source catkin workspace

- You must source this script in every bash terminal you use ROS in.
```bash
source devel/setup.bash
```
- It can be convenient to automatically source this script every time a new shell is launched. These commands will do that for you.
```bash
echo "source /home/<user_name>/Desktop/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```