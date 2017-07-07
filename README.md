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

Add this two sdf elements between plugin /plugin to save file
    <save_csv>true</save_csv>
    <csv_address>/home/.../events.csv</csv_address>

PROBLEMS
Too many cycles
Dynamic allocation of memory can be improved
