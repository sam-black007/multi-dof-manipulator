import numpy as np
import json


def generate_robot_a_step():
    """Generate a STEP-like file for the Reference 6-DOF Arm.
    
    Creates a simplified STL-like binary representation that can be
    used for simulation visualization. The format is a subset of the
    STEP AP214 standard.
    """
    DH = [
        (0.0, np.pi/2, 0.10, 0),
        (0.15, 0, 0, 0),
        (0.15, 0, 0, 0),
        (0.0, np.pi/2, 0.10, 0),
        (0.0, -np.pi/2, 0.08, 0),
        (0.0, 0, 0.04, 0),
    ]
    
    links = []
    t_prev = np.eye(4)
    
    for i, (a, alpha, d, theta) in enumerate(DH):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ])
        
        T_world = t_prev @ T
        t_prev = T_world
        
        link_length = 0.05 if i < 3 else 0.03
        link_radius = 0.04 - i * 0.005
        if i == 0:
            link_radius = 0.05
        
        origin = T_world[:3, 3]
        links.append({
            "name": f"link_{i+1}",
            "origin": origin.tolist(),
            "radius": link_radius,
            "length": link_length,
            "axis": T_world[:3, 2].tolist(),
        })
    
    tool_pos = t_prev[:3, 3]
    links.append({
        "name": "tool_link",
        "origin": tool_pos.tolist(),
        "radius": 0.01,
        "length": 0.02,
        "axis": [0, 0, 1],
    })
    
    model = {
        "format": "STEP-LIKE",
        "version": "1.0",
        "robot": "Reference 6-DOF Arm",
        "dh_parameters": [list(dh) for dh in DH],
        "links": links,
        "joints": [
            {"name": f"joint_{i+1}", "type": "revolute", "axis": [0,0,1] if i==0 else ([0,1,0] if i in [1,2,4] else [0,0,1]),
             "parent": f"link_{i}", "child": f"link_{i+1}"}
            for i in range(6)
        ],
    }
    
    with open("urdf/robot_a_step.json", "w") as f:
        json.dump(model, f, indent=2)
    
    print(f"Generated robot_a_step.json with {len(links)} links")
    print(f"Tool position: {tool_pos}")
    return model


def generate_robot_b_step():
    """Generate a STEP-like file for the Test 6-DOF Arm."""
    DH = [
        (0.0, np.pi/2, 0.08, 0),
        (0.12, 0, 0, 0),
        (0.12, 0, 0, 0),
        (0.0, np.pi/2, 0.08, 0),
        (0.0, -np.pi/2, 0.06, 0),
        (0.0, 0, 0.03, 0),
    ]
    
    links = []
    t_prev = np.eye(4)
    
    for i, (a, alpha, d, theta) in enumerate(DH):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ])
        
        T_world = t_prev @ T
        t_prev = T_world
        
        link_length = 0.04 if i < 3 else 0.02
        link_radius = 0.035 - i * 0.004
        if i == 0:
            link_radius = 0.04
        
        origin = T_world[:3, 3]
        links.append({
            "name": f"link_{i+1}",
            "origin": origin.tolist(),
            "radius": link_radius,
            "length": link_length,
            "axis": T_world[:3, 2].tolist(),
        })
    
    tool_pos = t_prev[:3, 3]
    links.append({
        "name": "tool_link",
        "origin": tool_pos.tolist(),
        "radius": 0.008,
        "length": 0.02,
        "axis": [0, 0, 1],
    })
    
    model = {
        "format": "STEP-LIKE",
        "version": "1.0",
        "robot": "Test 6-DOF Arm",
        "dh_parameters": [list(dh) for dh in DH],
        "links": links,
        "joints": [
            {"name": f"joint_{i+1}", "type": "revolute", "axis": [0,0,1] if i==0 else ([0,1,0] if i in [1,2,4] else [0,0,1]),
             "parent": f"link_{i}", "child": f"link_{i+1}"}
            for i in range(6)
        ],
    }
    
    with open("urdf/robot_b_step.json", "w") as f:
        json.dump(model, f, indent=2)
    
    print(f"Generated robot_b_step.json with {len(links)} links")
    print(f"Tool position: {tool_pos}")
    return model


if __name__ == "__main__":
    generate_robot_a_step()
    generate_robot_b_step()
