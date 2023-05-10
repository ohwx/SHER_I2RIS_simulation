# SHER_I2RIS_simulation

# Description

The project aims to implement the simulation of the combined teleoperation system in Asynchronous multi-body framework (AMBF). Simulation is one of the common means of testing robots, it enables researchers to quickly develop, validate and test control algorithms without worrying about damaging the robot. To achieve the goal, the model of the snake robot should be built in Blender according to the CAD drawing and then connected to eye robot in the simulation. Meanwhile, an AMBF plugin will be developed to manipulate the snake robot(Part C) attached to SHER(Part B) and interact with an OCT Scan of the eye.In addition, we also aim to develop a controller using haptic device(Part D) in AMBF and further test on the real robot.

![image](https://raw.githubusercontent.com/ohwx/SHER_I2RIS_simulation/main/%2B%2B%2B%2BISMR22_0012_MS.jpg)

# Installation Instructions:
## Install and Source AMBF 2.0

Build and source AMBF as per the instructions on AMBFs wiki: https://github.com/WPI-AIM/ambf/wiki/Installing-AMBF.

## Install Blender

Download the Linux version for your architecture and uncompress the file to the desired location from blender.org. Run it from the terminal.
```bash
cd <blender_path>/blender-3.4.1-linux-x64
./blender
```

# Build simulation model in Blender
## How to Install AMBF Add-on
You can refer to the detailed tutorial at https://github.com/WPI-AIM/ambf_addon.
To install the AMBF add-on, open Blender and go to the top menu bar. Find `Scripting` and import `ambf_addon.py`, then click `Run`. The AMBF plug-in will be successfully added. If it does not automatically pop up, you can open the right-side menu by pressing `n`.

## How to Use Blender to Create Simulation Models
First, import your model's parts and assemble them. You can press `g` and choose different snapping methods, then adjust the location. If you used rotation or scaling during assembly, press `ctrl+A` and choose `Rotation/Scaling` to apply the changes to the model.

After assembly, enable AMBF rigid body for each component in the AMBF add-on. Then, in `OPTIONAL HELPERS`, find `Create Joint` and select the appropriate type, parent, and child. You also need to adjust the joint's position in the model.

Once you have generated the joints, you can save the meshes in section C of the add-on and save the ADF in section D. Then, run the generated `.yaml` file in the terminal.
```bash
cd ambf/bin/lin-x86_64/
./ambf_simulator -a <yaml.file_path>
```

# Running the Controller 
The plugin is launched on top of the AMBF simulator along with other AMBF bodies, described by AMBF Description Format files (ADFs), as will be demonstrated below. Note that the executable binary,`ambf_simulator`, is located in `ambf/bin/lin-x86_64` if you are using Linux. 

Below are instructions as to how to load different volume and camera options. The -l tag allows user to run indexed multibodies that can be found in the `launch.yaml`. More info on launching the simulator can be found in the AMBF Wiki:  

https://github.com/WPI-AIM/ambf/wiki/Launching-the-Simulator  
https://github.com/WPI-AIM/ambf/wiki/Command-Line-Arguments  

You should follow the steps:
1. Open the AMBF simulator
```bash
cd ambf/bin/lin-x86_64/
ambf_simulator --launch_file ~/catkin_ws/src/sher_snake/launch.yaml -l 3 # 3 is SHER_and_I2RIS model
```
2. Connect to the input device. Take the 3D Connextion for an example, after you successfully install from http://wiki.ros.org/spacenav_node.
Run the command in terminal:
`roslaunch spacenav_node classic.launch`
Or if you want to get the data in python
```bash
import rospy
from sensor_msgs.msg import Joy

def callback(data):
    # Extract the spacenav/offset values from the Joy message
    translation = data.axes[:3]
    rotation = data.axes[3:]

    # Print the spacenav/offset values
    print("Translation: (%f, %f, %f)" % tuple(translation))
    print("Rotation: (%f, %f, %f)" % tuple(rotation))

rospy.init_node('spacenav_listener')  # Initialize the ROS node
rospy.Subscriber("/spacenav/joy", Joy, callback)  # Subscribe to the spacenav/joy topic

rospy.spin()  # Enter the ROS event loop
```

4. Running the python code within your workspace 
`~/<your_workspace_path>/scr/sher_snake/Scripts/control_3Dconnexion.py`
5. Then you can move the input device and when you quit the code, you can see the plot of error of actual and theroatical velocity error.


