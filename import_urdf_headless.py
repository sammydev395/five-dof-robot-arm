#!/usr/bin/env python3

import os
import sys
import time
from omni.isaac.kit import SimulationApp

# Start Isaac Sim headless
simulation_app = SimulationApp({"headless": True})

# Wait until startup is complete
import omni.kit
import omni.usd
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import extensions

# Enable required extensions
extensions.enable_extension("omni.isaac.urdf")

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
    
    # Save the USD file
    output_path = os.path.join(current_dir, "five_dof_arm.usd")
    omni.usd.get_context().save_as_stage(output_path, None)
    print(f"Saved USD file to: {output_path}")
else:
    print("Failed to import URDF")

# Run a few simulation steps to ensure everything is loaded
for _ in range(10):
    simulation_context.step()
    time.sleep(0.1)

# Cleanup and close the application
simulation_app.close() 