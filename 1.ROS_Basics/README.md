The goal of this mini-project was to learn the basics of ROS (Robot Operating System). ROS is the software layer between specific robot programs and the robot. In our case, we are using the NAO robot.  ROS was developed over many years and it now has a collection of useful libraries and tools for robotic applications. In this mini-project, I learned the following ROS basics:
• Creating a workspace and ROS package
• Concept of ROS: nodes and their communication with each other
• Basic message types of the NAO robot

ROS Basics:
The main concept of ROS: Communication between nodes.
A node is a process doing some computation. It can receive messages containing input data,
process it, and send messages containing the results. Applied to a robot for example, one node
can query a specific sensor (e.g. tactile sensor), one node can do some higher-level planning or
computation, and another node can send and receive motor data (e.g. position, velocity, etc.). Note
that one node does not have to “know” about the existence of another node, in order to
communicate with it. For node-to-node communication, it is important that a message topic is
defined first. Consider two nodes N1 and N2. N1 sends (`publishes’) a message on the topic T. If
N2 has `subscribed’ to the same topic T, it will receive the message sent by N1. In ROS, you can
use pre-defined message structures. Pre-defined messages are data types like int, float, arrays of
these basic types, etc. Nevertheless, you can also define your own message structures (custom
messages). You can define the topic names by yourself.

embed picture of ros nodes

Task:
The following simple response to keyboard input was implemented: When writing on the keyboard and
pressing Enter, the keyboard_node sends a message containing the typed sentence. The
central_node receives that message and displays its content on the terminal window. The communication between the nodes was visualized with rqt_graph.
