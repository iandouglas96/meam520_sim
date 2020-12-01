import numpy as np
from math import pi
from random import shuffle
from os.path import dirname, abspath, join

# nominal block positions (x, y, z)
turntable_block_nom_xyz = np.array([[-0.06, 0.00, 0.15], 
                                    [0.06, 0.00, 0.15],
                                    [-0.00, 0.06, 0.15],
                                    [0.00, -0.06, 0.15],
                                    [0.00, 0.00, 0.15]])

platform_a_block_nom_xyz = np.array([[-0.125, 0.275, 0.10], 
                                     [-0.075, 0.275, 0.10],
                                     [-0.125, 0.225, 0.10],
                                     [-0.075, 0.225, 0.10]])

platform_b_block_nom_xyz = np.array([[-0.025, 0.4, 0.10], 
                                     [-0.025, 0.35, 0.10],
                                     [0.025, 0.4, 0.10],
                                     [0.025, 0.35, 0.10]])


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
def get_random_pose(xyz, perturb_radius):
    noise = np.random.uniform(-perturb_radius, perturb_radius, size=2)
    rp = np.random.choice(4, 2) * pi / 2
    #print(rp)
    yaw = np.random.uniform(0, pi)
    print(yaw)
    return (xyz[0] + noise[0], xyz[1] + noise[1], xyz[2], rp[0], rp[1], yaw)

# given a 6-DOF pose mirror across the x=y plane
def mirror(pose):
    return (-pose[0], -pose[1], pose[2], pose[3], pose[4], pose[5] + pi)

# write the launch file
parent_directory = dirname(dirname(abspath(__file__)))
with open(join(parent_directory, 'launch', 'objects.launch'), 'w') as f:
    f.write(header)

    # write turntable xml
    for i in range(len(turntable_block_nom_xyz)):
        random_pose = get_random_pose(turntable_block_nom_xyz[i], 0.018)
        f.write("\n\t<!-- DYNAMIC BLOCK -->")
        f.write(block_xml(i + 1, random_pose, "dynamic"))
    
    # generate platform xml
    np.random.shuffle(platform_a_block_nom_xyz)
    np.random.shuffle(platform_b_block_nom_xyz)
    platform_block_nom_xyz = np.vstack((platform_a_block_nom_xyz[:2], 
                                        platform_b_block_nom_xyz[:2]))
    for i in range(len(platform_block_nom_xyz)):
        random_pose = get_random_pose(platform_block_nom_xyz[i], 0.013)
        f.write("\n\t<!-- STATIC BLOCK: BLUE TEAM -->")
        f.write(block_xml(2 * i + 1, random_pose, "blue"))
        f.write("\n\t<!-- STATIC BLOCK: RED TEAM -->")
        f.write(block_xml(2 * i + 2, mirror(random_pose), "red"))

    f.write(footer)
