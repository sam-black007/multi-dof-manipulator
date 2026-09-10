# sam-black007 6-DOF Robot Description

## Author

- **Name**: sam-black007
- **Repository**: multi-dof-manipulator
- **License**: Proprietary - all rights reserved

## Robot Overview

- **Name**: owr_6dof
- **DOF**: 6
- **Robot**: OWR 6-DOF robotic arm

## Links

- base_link
- BS_Link
- SE_Link
- EW1_Link
- W1W2_Link
- W2W3_Link
- W3EEF_Link
- EEF_Link

## Joints

- BJ: type=revolute, axis=[0,0,1], limits=[-120°, 120°], effort=200, vel=5
- SJ: type=revolute, axis=[0,1,0], limits=[-90°, 90°], effort=200, vel=5
- EJ: type=revolute, axis=[0,1,0], limits=[-225°, 60°], effort=200, vel=5
- W1J: type=revolute, axis=[1,0,0], limits=[-90°, 90°], effort=200, vel=5
- W2J: type=revolute, axis=[0,1,0], limits=[-60°, 150°], effort=200, vel=5
- W3J: type=revolute, axis=[1,0,0], limits=[-180°, 180°], effort=200, vel=5

## Masses & Inertias

- base_link: mass=2.004, inertia=[0.00560, 0.00632, 0.00560]
- BS_Link: mass=1.976, inertia=[0.00634, 0.00431, 0.00588]
- SE_Link: mass=6.924, inertia=[0.13565, 0.14188, 0.01733]
- EW1_Link: mass=1.641, inertia=[0.00433, 0.00315, 0.00467]
- W1W2_Link: mass=2.384, inertia=[0.00656, 0.02573, 0.02828]
- W2W3_Link: mass=2.168, inertia=[0.00982, 0.00958, 0.00386]
- W3EEF_Link: mass=0.543, inertia=[0.000737, 0.000548, 0.000548]
- EEF_Link: collision=box 0.001m

## Meshes

- Collision (STL, 7 files): base_link.STL, BS_Link.STL, EW1_Link.STL, SE_Link.STL, W12_Link.STL, W23_Link.STL, W3Eff_Link.STL
- Visual (DAE, 7 files): base.dae, bs.dae, be.dae, EW1.dae, w12.dae, W23.dae, w3eff.dae
- Gripper: 11 files (STL + DAE)
- Objects: table.STL, table1.STL
- Factory setup: setup.STL, setup1.STL

## URDF/Xacro Files

- owr.urdf.xacro (main)
- owr_robot.urdf.xacro (robot definition)
- owr.gazebo.xacro
- owr.transmission.xacro
- common.gazebo.xacro

## Package Paths

- file:// URDF local references
- config/owr_6dof.yaml robot configuration

## Redistribution Status

- **Proprietary** - all rights reserved to sam-black007