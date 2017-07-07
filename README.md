# DVS Gazebo Plugin
This Gazebo plugin simulates a DVS

The code is modified from:
https://github.com/HBPNeurorobotics/gazebo_dvs_plugin
which you should look at for instruction on how to install and how to use

I used the code above for the interface with gazebo. 
I made some improvements (or at least think so):
- correct the event generation model (no logarithm anywhere in their version)
- linear interpolation of the time so the events are not simply at the time of the last frame
- possibility to save the events in a csv file

Example of sdf for the use of the plugin
<plugin name="camera_controller" filename="libgazebo_dvs_plugin.so">
    <cameraName>dvs</cameraName>
    <eventThreshold>0.2</eventThreshold>
    <cameraInfoTopicName>camera_info</cameraInfoTopicName>
    <save_csv>false</save_csv>
    <csv_address>/home/.../events.csv</csv_address>
    <!-- <eventsTopicName>events</eventsTopicName> -->
</plugin>

PROBLEMS
Too many cycles
Dynamic allocation of memory can be improved
