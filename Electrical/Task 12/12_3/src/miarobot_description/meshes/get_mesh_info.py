import trimesh
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
mesh_path = os.path.join(script_dir, "miarobot_colored.dae")
mesh = trimesh.load(mesh_path)

# Get bounding box dimensions
length_x, length_y, length_z = mesh.extents
max_dim = mesh.extents.max()

print(f"X: {length_x:.3f}")
print(f"Y: {length_y:.3f}")
print(f"Z: {length_z:.3f}")
print(f"Model length: {max_dim:.3f}")

#
# Width (X): 0.390
# Depth (Y): 0.347
# Height (Z): 0.265
# Model length (max dimension): 0.390
# Units: unknown (assume meters)