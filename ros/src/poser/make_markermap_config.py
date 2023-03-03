# -*- coding: utf-8 -*-
"""
Created on Fri May 20 23:40:12 2022

@author: 7450339
"""

import yaml
import numpy as np
import ipdb


indices = [
    33,
    158,
    26,
    12,
    104,
    200,
    248,
    87,
    195]
upper_left_corners = [
    [0.,0.],
    [0.5524,0.0389],
    [0.7441,0.0389],
    [0.5524,-0.1484],
    [0.7441,-0.1484],
    [0.5524,-0.3373],
    [0.6874,-0.3373],
    [0.6223,-0.4397],
    [0.7556,-0.4397]
    ]

sizes = [
    0.4877,
    0.1651,
    0.1651,
    0.1651,
    0.1651,
    0.0762,
    0.0762,
    0.0762,
    0.0762]

d = {"aruco_bc_dict":"ARUCO_MIP_36h12", "aruco_bc_nmarkers":len(indices), "aruco_bc_mInfoType":1, "aruco_bc_markers": []}
for index, ul, size in zip(indices, upper_left_corners, sizes):
    
    corners = [
        [ul[0], ul[1], 0.],
        [ul[0]+size, ul[1], 0.],
        [ul[0]+size, ul[1]-size, 0.],
        [ul[0], ul[1]-size, 0.]
        ]
    d["aruco_bc_markers"].append({"id":index, "corners":corners})
    pass
print(yaml.dump(d, width=50, default_flow_style=True))

with open("Configuration.yml",'w') as f:
    #f.write("%YAML:1.0\n---")
    yaml.dump(d,f, version=(1,2), default_flow_style=True)

    
