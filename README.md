#2.12 Final Project Initial Repo

##Structure
In 'catkin_ws/src':
<dl>
<dt>me212bot</dt>
<dd>High Level code, such as the navigation commander for Apriltag following</dd>
<dt>me212base</dt>
<dd>Low Level control code, receives velocity messages and sends them through serial to arduino.  Also contains arduino code.</dd>
<dt>me212cv</dt>
<dd>Computer vision code</dd>
<dt>me212arm</dt>
<dd>Dynamixel, IK and RRT code for use with the arms in Lab 5</dd>
</dl>

In 'software/config':
<dl>
<dt>environment.sh</dt>
<dd>Required as-is, essentially points your terminal to the files it needs to work with ROS</dd>
<dt>procman.pmd</dt>
<dd>procman config file, adjust this to make your pman show the nodes and launch files you need</dd>
</dl>

