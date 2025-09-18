# Nodes

`roscore` : starts a ROS master  
- Only one ROS master can be active at a time per system.  
- Kill it using `Ctrl+C`.

`gedit ~/.bashrc` : check the bashrc settings/environment  
- Should contain this line: `source /opt/ros/noetic/setup.bash`

`rosrun <package_name> <executable_file>` : start a new node from a package  
- Requires `roscore` to already be running.  
- Running `rosrun <package_name>` then pressing **double Tab** lists all executable files in that package.

`rqt_graph` : ROS tool that shows a graph of how nodes are connected  

`ros<type> list` : lists running entities of that type  
- Examples: `rostopic list`, `rosnode list`, `rosservice list`, `rosparam list`

`rosnode kill /node_name` : kills a running node  

`rostopic echo /topicname` : prints the data passing through a topic (useful for debugging)  

`rosmsg show <package>/<message>` : shows the fields (names and types) of a message definition  


# Workspace

`catkin_make` : compiles packages inside the workspace  
- Must be run in the workspace root (where `src` exists)  
- Creates `build` and `devel` folders  
- Recommended after adding or modifying a package  

`source ~/catkin_ws/devel/setup.bash` : sources the workspace  
- Adds your workspace to `ROS_PACKAGE_PATH`  
- Adds Python modules to `PYTHONPATH`  
- Usually added to `.bashrc` so it runs automatically in every terminal  

`catkin_create_pkg <pkg_name> [dependencies]` : creates a new package in the `src` folder  
- Dependencies examples: `rospy`, `roscpp`, `std_msgs`

`code .` : open the folder in VS Code  

`chmod +x <file_name.ext>` : make a file executable  


# Files

`#!/usr/bin/env python3` : shebang line telling Linux to run the file with Python 3  


# Rospy

`rospy.init_node("node_name", anonymous=bool)` : starts the node  
- `anonymous=True` appends a random number to the node name to avoid name clashes  

`rospy.loginfo("message")` : prints the message with timestamp  

`rospy.logwarn("message")` : prints the message in yellow  

`rospy.logerr("message")` : prints the message in red  

`rospy.is_shutdown()` : returns True if the node is shutting down  

`rospy.Publisher("topic_name", type, queue_size=n)` : creates a publisher  
- Defines the topic and message type  
- `queue_size` sets the outgoing message buffer size  
- When the buffer is full, old messages are dropped  
- Useful since publishers may send faster than subscribers can process  

`publisher.publish(msg)` : publishes a message on the topic  

`rospy.Subscriber("topic_name", type, callback_function)` : creates a subscriber to a topic  

`rospy.set_param("param_name", value)` : sets a parameter  

`rospy.get_param("name", default_value)` : gets a parameter  
- Returns `default_value` if not set  

`rospy.has_param("name")` : checks if a parameter exists  

`rospy.delete_param("name")` : deletes a parameter  

`rospy.wait_for_service("service_name", timeout=None)` : blocks until the service is available  
- If `timeout` is given, waits up to that duration before raising an exception  

`rospy.Service("service_name", ServiceType, callback_function)` : creates a service  

`rospy.ServiceProxy("service_name", ServiceType, persistent=False)` : creates a handle to call a service  
- `persistent=False` → opens/closes TCP connection every call  
- `persistent=True` → keeps the connection open (better for repeated calls)  



