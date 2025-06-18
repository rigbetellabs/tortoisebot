import os
import trimesh

def guess_primitive_type(mesh):
    # Naive guess: if it's roughly cylindrical, guess cylinder; otherwise box
    extents = mesh.extents
    ratios = sorted(extents)
    if abs(ratios[0] - ratios[1]) < 0.05 and ratios[2] > ratios[0] * 1.5:
        return "cylinder"
    else:
        return "box"

def create_urdf_collision_tag(name, mesh):
    primitive = guess_primitive_type(mesh)
    center = mesh.bounding_box.centroid
    urdf = f"<!-- {name} -->\n"

    if primitive == "box":
        size = mesh.extents
        urdf += f"""<collision name="{name}_collision">
  <origin xyz="{center[0]:.4f} {center[1]:.4f} {center[2]:.4f}" rpy="0 0 0"/>
  <geometry>
    <box size="{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}"/>
  </geometry>
</collision>\n"""
    elif primitive == "cylinder":
        height = mesh.extents[2]
        radius = max(mesh.extents[0], mesh.extents[1]) / 2
        urdf += f"""<collision name="{name}_collision">
  <origin xyz="{center[0]:.4f} {center[1]:.4f} {center[2]:.4f}" rpy="0 0 0"/>
  <geometry>
    <cylinder radius="{radius:.4f}" length="{height:.4f}"/>
  </geometry>
</collision>\n"""
    return urdf

def process_meshes_in_dir(directory):
    for filename in os.listdir(directory):
        if filename.endswith(".dae"):
            path = os.path.join(directory, filename)
            try:
                mesh = trimesh.load(path)
                if not isinstance(mesh, trimesh.Trimesh):
                    mesh = mesh.dump().sum()  # merge multiple geometries
                print(create_urdf_collision_tag(filename[:-4], mesh))
            except Exception as e:
                print(f"Failed to process {filename}: {e}")

# Example usage
process_meshes_in_dir("/home/tars/ros2_ws/src/tortoisebot-ros2-humble/tortoisebot_description/models/meshes")
