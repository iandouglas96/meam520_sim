import numpy as np
import os

# default block positions (x, y, z)
dynamic_block_defaults = np.array([[-0.05, -0.03, 0.15], 
                                   [0.05, 0.06, 0.15],
                                   [-0.03, 0.05, 0.15],
                                   [0.04, -0.03, 0.15],
                                   [0.00, 0.00, 0.15]])
static_block_defaults = np.array([[-0.08, 0.22, 0.10], 
                                  [-0.13, 0.28, 0.10],
                                  [0.01, 0.376, 0.10],
                                  [-0.04, -0.373, 0.10]])

# launch file boilerplate
header = \
"""<?xml version="1.0"?>
<launch>

    <arg name="delay" value="3" />
"""

footer = \
"""
</launch>"""

# generate xml for a block object given specified arguments
def block_xml(id, pose, type):
    block_template = \
    """
    <include file="$(find al5d_gazebo)/launch/cube.launch" ns="%s">
        <arg name="namespace" value="%s" />
        <arg name="x" value="%0.3f"/>
        <arg name="y" value="%0.3f"/>
        <arg name="z" value="%0.3f"/>
        <arg name="roll" value="%0.3f"/>
        <arg name="pitch" value="%0.3f"/>
        <arg name="yaw" value="%0.3f"/>
        <arg name="color" default="%s"/>
    </include>
    """
    colors = {"red": "0.5725\ 0.0667\ 0.109\ 1",
              "blue": "0.1137\ 0.2078\ 0.3412\ 1",
              "dynamic": ".133\ .043\ .212\ 1"}
    name = "static" + str(id) if type != "dynamic" else type + str(id)
    return block_template % ((name, name, ) + pose + tuple([colors[type]]))

# function to generate random 6-DOF pose for block
def get_random_pose(xyz):
    return (xyz[0], xyz[1], xyz[2], 0, 0, 0)

# given a 6-DOF pose mirror across the x=y plane
def mirror(pose):
    return (-pose[0], -pose[1], -pose[2], pose[3], pose[4], pose[5])

# write the launch file
directory = os.getcwd()
with open(os.path.join(directory, 'objects.launch'), 'w') as f:
    f.write(header)
    for i in range(len(dynamic_block_defaults)):
        random_pose = get_random_pose(dynamic_block_defaults[i])
        f.write("\n\t<!-- DYNAMIC BLOCK -->")
        f.write(block_xml(i + 1, random_pose, "dynamic"))
    for i in range(len(static_block_defaults)):
        random_pose = get_random_pose(static_block_defaults[i])
        f.write("\n\t<!-- STATIC BLOCK: BLUE TEAM -->")
        f.write(block_xml(2 * i + 1, random_pose, "blue"))
        f.write("\n\t<!-- STATIC BLOCK: RED TEAM -->")
        f.write(block_xml(2 * i + 2, mirror(random_pose), "red"))
    f.write(footer)


print(block_xml(i + 1, random_pose, "dynamic"))
