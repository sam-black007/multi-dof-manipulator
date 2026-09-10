# sam-black007 6-DOF Robot Description

**Repository**: multi-dof-manipulator  
**Author**: sam-black007  
**License**: Proprietary - all rights reserved

This repository provides Python configuration and URDF description for the
6-DOF OWR robotic arm. All physical parameters, joint limits, mesh files, and
URDF description are created and owned by sam-black007.

## Directory Structure

- `urdf/` — Simulator-portable URDF description.
- `meshes/` — Collision STL mesh files.
- `README.md` — This file.

## Configuration

Robot parameters are defined in `config/owr_6dof.yaml`.

## Usage

The URDF and configuration files in this repository can be used directly with
simulation adapters (OmniSim, Gazebo, Isaac).

## Notes

- All physical parameters (masses, inertias, joint limits, mesh paths) are
  defined in the configuration and URDF files.
- DH parameters cannot be unambiguously derived from the URDF joint transforms;
  see the Python configuration for the documented issue.

---
*This work: sam-black007/multi-dof-manipulator. All rights reserved.*