#!/usr/bin/env python3

import os
import omni.usd
import omni.isaac.urdf as urdf_extension

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(current_dir, "five_dof_arm/urdf/five_dof_arm.urdf")

# Import the URDF
import_config = urdf_extension.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.self_collision = False
import_config.create_physics_scene = True
import_config.import_inertia_tensor = True

urdf_interface = urdf_extension.acquire_urdf_interface()
result, prim_path = urdf_interface.import_urdf(urdf_path, import_config)

if result:
    print(f"Successfully imported URDF to prim path: {prim_path}")
else:
    print("Failed to import URDF") 