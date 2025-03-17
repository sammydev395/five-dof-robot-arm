#!/usr/bin/env python3

import os
import sys
from omni.isaac.kit import SimulationApp

# Start Isaac Sim headless
simulation_app = SimulationApp({"headless": False})

# Wait until startup is complete
import omni.kit
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions
from omni.isaac.core.utils.stage import add_reference_to_stage

# Enable required extensions
extensions.enable_extension("omni.isaac.urdf")
extensions.enable_extension("omni.isaac.ros_bridge")

# Import URDF
import omni.isaac.urdf as urdf_extension

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(current_dir, "five_dof_arm/urdf/five_dof_arm.urdf")

# Create simulation context
simulation_context = SimulationContext(physics_dt=1.0/60.0, rendering_dt=1.0/60.0, stage_units_in_meters=1.0)

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
    simulation_app.close()
    sys.exit(1)

# Keep the application running
while simulation_app.is_running():
    simulation_context.step(render=True)
    if not simulation_app.is_running():
        break

# Cleanup and close the application
simulation_app.close() 